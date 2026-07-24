"""
ROS2 MCP HTTP Server.

FastAPI приложение + rclpy нода в одном процессе.
ROS executor крутится в отдельном потоке; вызовы диспетчеризуются
в thread-pool и ждут rclpy-future через done-callback + Event
(spin_until_future_complete нельзя — нода уже в фоновом executor'е).

Инструменты покрывают почти весь `ros2` CLI:
  граф   — list_nodes/topics/services/actions, node_info, topic_info,
           service_type, action_info, interface_show/proto
  чтение — topic_echo_once, topic_hz, param_list/get/dump
  запись — topic_publish, service_call, action_send_goal, param_set   ← gated
  файлы  — file_list/read (safe), file_write                          ← gated
  shell  — shell_exec (colcon build, find, tree, rm, touch, ...)       ← gated
  LLM/VLM— llm_query, vlm_query (если interfaces собраны)

Все «пишущие» инструменты проходят через policy-движок (mcp_policy.yaml),
который задаёт границы дозволенного. См. _Policy ниже и сам yaml-файл.
"""

import os
import re
import json
import time
import array
import base64
import asyncio
import subprocess
import threading
import traceback

try:
    import numpy as np
except Exception:
    np = None
from pathlib import Path
from typing import Any

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from rosidl_runtime_py.utilities import get_message, get_service, get_action
from rosidl_runtime_py import message_to_ordereddict, set_message_fields

from rcl_interfaces.srv import (
    ListParameters, GetParameters, SetParameters, DescribeParameters,
)
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

try:
    from vlm_interfaces.action import VlmQuery
    HAS_VLM = True
except ImportError:
    HAS_VLM = False

try:
    from llm_interfaces.action import LlmQuery
    HAS_LLM = True
except ImportError:
    HAS_LLM = False

ALLOWED_ROOTS = [
    Path('/home/sverk'),
    Path('/tmp'),
    Path('/models'),
    Path('/shared'),
    Path('/policy'),
]

DEFAULT_TIMEOUT = 15.0


# ════════════════════════════════════════════════════════════════════════════
#  Policy — границы дозволенного для «пишущих» инструментов
# ════════════════════════════════════════════════════════════════════════════
#
# Загружается из MCP_POLICY_FILE (yaml). Горячая перезагрузка по mtime —
# правь файл на хосте, перезапускать контейнер не нужно.
#
# Категории «пишущих» инструментов и что считается их "target":
#   topic_publish     → имя топика
#   service_call      → имя сервиса
#   action_send_goal  → имя action
#   param_set         → имя ноды
#   file_write        → путь файла
#
# mode:
#   read_only — все пишущие инструменты запрещены (и скрыты из /tools)
#   guarded   — пишущий вызов разрешён, только если target подходит под allow
#               (и не подходит под deny). Пустой allow => запрещено всё.
#   full      — пишущие разрешены; проверяется только deny (allow игнорируется)
#
# tools: { <tool_name>: true|false } — жёсткий per-tool оверрайд поверх mode.
# require_confirm: true — пишущий вызов требует arguments._confirm == true.

WRITE_TOOLS = {
    'ros2_topic_publish':    'topic_publish',
    'ros2_service_call':     'service_call',
    'ros2_action_send_goal': 'action_send_goal',
    'ros2_param_set':        'param_set',
    'file_write':            'file_write',
    'shell_exec':            'shell_exec',
}

_BUILTIN_POLICY = {
    # Безопасный дефолт, если файл политики не найден: читать можно, писать нельзя.
    'mode': 'read_only',
    'require_confirm': False,
    'tools': {},
    'rules': {},
}


class PolicyError(PermissionError):
    """Действие запрещено политикой."""


class Policy:
    def __init__(self, path: str | None):
        self.path = path
        self._mtime = 0.0
        self._data = dict(_BUILTIN_POLICY)
        self._source = 'builtin-default (read_only)'
        self.reload(force=True)

    # ── загрузка / горячая перезагрузка ──────────────────────────
    def reload(self, force: bool = False) -> None:
        if not self.path or not os.path.isfile(self.path):
            if force:
                self._data = dict(_BUILTIN_POLICY)
                self._source = 'builtin-default (read_only)'
            return
        try:
            mtime = os.path.getmtime(self.path)
            if not force and mtime == self._mtime:
                return
            with open(self.path, 'r') as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                raise ValueError('policy root must be a mapping')
            self._data = data
            self._mtime = mtime
            self._source = self.path
        except Exception as e:  # битый файл — не падаем, откатываемся в read_only
            self._data = dict(_BUILTIN_POLICY)
            self._source = f'{self.path} (INVALID: {e}) -> read_only fallback'

    # ── доступ ───────────────────────────────────────────────────
    @property
    def mode(self) -> str:
        return str(self._data.get('mode', 'read_only')).lower()

    @property
    def require_confirm(self) -> bool:
        return bool(self._data.get('require_confirm', False))

    def tool_override(self, tool: str):
        # `tools:` with only commented entries parses as None -> guard with `or {}`.
        return (self._data.get('tools') or {}).get(tool, None)

    def _rule(self, category: str) -> dict:
        return (self._data.get('rules') or {}).get(category) or {}

    def summary(self) -> dict:
        return {
            'source': self._source,
            'mode': self.mode,
            'require_confirm': self.require_confirm,
            'tools': self._data.get('tools') or {},
            'rules': self._data.get('rules') or {},
        }

    def tool_enabled(self, tool: str) -> bool:
        """Показывать ли инструмент в /tools (и разрешён ли он в принципе)."""
        ov = self.tool_override(tool)
        if ov is not None:
            return bool(ov)
        if tool in WRITE_TOOLS:
            return self.mode != 'read_only'
        return True  # read-only инструменты всегда доступны

    def check(self, tool: str, target: str, args: dict) -> None:
        """Бросает PolicyError, если вызов запрещён."""
        if tool not in WRITE_TOOLS:
            return  # чтение всегда можно

        ov = self.tool_override(tool)
        if ov is False:
            raise PolicyError(f'tool {tool!r} disabled by policy (tools.{tool}=false)')

        # mode-гейт (per-tool override может явно включить в read_only)
        if ov is not True:
            if self.mode == 'read_only':
                raise PolicyError(f'tool {tool!r} blocked: policy mode=read_only')

        if self.require_confirm and not args.get('_confirm', False):
            raise PolicyError(
                f'tool {tool!r} requires confirmation: re-call with '
                f'arguments._confirm=true (policy require_confirm=true)'
            )

        category = WRITE_TOOLS[tool]
        rule = self._rule(category)
        allow = rule.get('allow') or []
        deny = rule.get('deny') or []

        for pat in deny:
            if re.search(pat, target):
                raise PolicyError(
                    f'{category} on {target!r} denied by policy '
                    f'(rules.{category}.deny ~ {pat!r})'
                )

        if self.mode == 'full' or ov is True:
            return  # full / явно включённый инструмент: deny уже проверен

        # guarded: нужно совпасть с allow
        for pat in allow:
            if re.search(pat, target):
                return
        raise PolicyError(
            f'{category} on {target!r} not allowed by policy '
            f'(mode=guarded, no rules.{category}.allow match)'
        )


_policy: Policy | None = None


# ════════════════════════════════════════════════════════════════════════════
#  ROS2 нода
# ════════════════════════════════════════════════════════════════════════════

class McpBridgeNode(Node):

    def __init__(self):
        super().__init__('ros2_mcp_server')
        self._ac: dict[str, ActionClient] = {}
        if HAS_VLM:
            self._ac['vlm'] = ActionClient(self, VlmQuery, '/vlm/query')
        if HAS_LLM:
            self._ac['llm'] = ActionClient(self, LlmQuery, '/llm/query')
        self.get_logger().info(f'MCP bridge ready  VLM={HAS_VLM}  LLM={HAS_LLM}')

    # ── ожидание rclpy-future (фоновый executor его докрутит) ─────
    def _wait(self, future, timeout: float = DEFAULT_TIMEOUT):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise TimeoutError(f'ROS2 future timed out after {timeout}s')
        return future.result()

    # ── QoS: подстроиться под издателей/подписчиков топика ────────
    def _match_qos(self, topic: str, peer: str, override: dict | None) -> QoSProfile:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        infos = (self.get_publishers_info_by_topic(topic) if peer == 'pub'
                 else self.get_subscriptions_info_by_topic(topic))
        if infos:
            p = infos[0].qos_profile
            qos.reliability = p.reliability
            qos.durability = p.durability
        if override:
            rel = override.get('reliability')
            if rel:
                qos.reliability = (ReliabilityPolicy.BEST_EFFORT
                                   if rel == 'best_effort' else ReliabilityPolicy.RELIABLE)
            dur = override.get('durability')
            if dur:
                qos.durability = (DurabilityPolicy.TRANSIENT_LOCAL
                                  if dur == 'transient_local' else DurabilityPolicy.VOLATILE)
            if 'depth' in override:
                qos.depth = int(override['depth'])
        return qos

    def _resolve_topic_type(self, topic: str, given: str | None) -> str:
        if given:
            return given
        m = dict(self.get_topic_names_and_types())
        if topic not in m or not m[topic]:
            raise ValueError(f'Cannot resolve type for {topic}; pass "type" explicitly')
        return m[topic][0]

    # ── ГРАФ ──────────────────────────────────────────────────────
    def node_info(self, name: str) -> dict:
        # Build from per-topic ENDPOINT info (which carries node_name/namespace)
        # instead of the by-node graph API: get_*_by_node() only works for the
        # local participant under rmw_fastrtps without the ros2 daemon and
        # returns "nonexistent node" for remote nodes. Endpoint discovery works
        # cross-container, so this reconstruction covers the whole graph.
        target = name if name.startswith('/') else '/' + name
        names_ns = {((nsp.rstrip('/') + '/' + nm) if nsp != '/' else '/' + nm)
                    for nm, nsp in self.get_node_names_and_namespaces()}
        if target not in names_ns:
            raise ValueError(f'node {name!r} not found in graph')

        def ep_node(ep) -> str:
            ns = ep.node_namespace
            return (ns.rstrip('/') + '/' + ep.node_name) if ns != '/' else '/' + ep.node_name

        pubs, subs = [], []
        for topic, types in self.get_topic_names_and_types():
            if any(ep_node(e) == target for e in self.get_publishers_info_by_topic(topic)):
                pubs.append({'name': topic, 'types': types})
            if any(ep_node(e) == target for e in self.get_subscriptions_info_by_topic(topic)):
                subs.append({'name': topic, 'types': types})
        # Services/clients aren't exposed with endpoint node info cross-participant;
        # match by the conventional node-name prefix (best effort).
        services = [{'name': s, 'types': t}
                    for s, t in self.get_service_names_and_types()
                    if s == target or s.startswith(target + '/')]
        return {'node': target, 'publishers': pubs, 'subscribers': subs,
                'services': services}

    def topic_info(self, topic: str) -> dict:
        m = dict(self.get_topic_names_and_types())
        pubs = self.get_publishers_info_by_topic(topic)
        subs = self.get_subscriptions_info_by_topic(topic)
        def qd(info):
            q = info.qos_profile
            return {'node': info.node_name, 'namespace': info.node_namespace,
                    'reliability': q.reliability.name, 'durability': q.durability.name}
        return {
            'topic': topic,
            'types': m.get(topic, []),
            'publisher_count': len(pubs),
            'subscriber_count': len(subs),
            'publishers': [qd(i) for i in pubs],
            'subscribers': [qd(i) for i in subs],
        }

    def list_actions(self) -> dict:
        # action = сервис */_action/send_goal + топики */_action/feedback,status
        svc = dict(self.get_service_names_and_types())
        actions = {}
        for s, types in svc.items():
            if s.endswith('/_action/send_goal'):
                base = s[: -len('/_action/send_goal')]
                # offboard_interfaces/action/Calibrate_SendGoal -> .../Calibrate
                atype = re.sub(r'_SendGoal$', '', types[0]) if types else ''
                actions[base] = atype
        return {'actions': [{'name': k, 'type': v} for k, v in sorted(actions.items())]}

    def action_info(self, name: str) -> dict:
        base = name.rstrip('/')
        sg = base + '/_action/send_goal'
        svc = dict(self.get_service_names_and_types())
        atype = ''
        if sg in svc and svc[sg]:
            atype = re.sub(r'_SendGoal$', '', svc[sg][0])
        return {'action': base, 'type': atype,
                'available': sg in svc}

    # ── ЧТЕНИЕ ────────────────────────────────────────────────────
    def echo_once(self, topic: str, timeout: float, qos_override: dict | None,
                  full: bool = False, include_image: bool = False) -> dict:
        type_str = self._resolve_topic_type(topic, None)
        msg_type = get_message(type_str)
        qos = self._match_qos(topic, 'pub', qos_override)
        received: list = []
        event = threading.Event()
        sub = self.create_subscription(
            msg_type, topic, lambda m: (received.append(m), event.set()), qos)
        try:
            ok = event.wait(timeout=timeout)
        finally:
            self.destroy_subscription(sub)
        if not ok:
            raise TimeoutError(f'No message on {topic} within {timeout}s')
        # full=True → полный дамп; иначе компактная сводка (тяжёлые типы +
        # длинные массивы сворачиваются — см. _compact).
        data = (message_to_ordereddict(received[0]) if full
                else _compact(received[0], include_image=include_image))
        return {'topic': topic, 'type': type_str, 'compact': not full, 'data': data}

    def topic_hz(self, topic: str, duration: float, qos_override: dict | None) -> dict:
        type_str = self._resolve_topic_type(topic, None)
        msg_type = get_message(type_str)
        qos = self._match_qos(topic, 'pub', qos_override)
        stamps: list = []
        sub = self.create_subscription(
            msg_type, topic, lambda _m: stamps.append(time.monotonic()), qos)
        try:
            time.sleep(duration)
        finally:
            self.destroy_subscription(sub)
        n = len(stamps)
        rate = (n - 1) / (stamps[-1] - stamps[0]) if n >= 2 else 0.0
        return {'topic': topic, 'type': type_str, 'samples': n,
                'window_s': duration, 'rate_hz': round(rate, 3)}

    # ── ЗАПИСЬ ────────────────────────────────────────────────────
    def publish(self, topic: str, values: dict, type_str: str | None,
                count: int, rate_hz: float, qos_override: dict | None) -> dict:
        type_str = self._resolve_topic_type(topic, type_str)
        msg_type = get_message(type_str)
        qos = self._match_qos(topic, 'sub', qos_override)
        pub = self.create_publisher(msg_type, topic, qos)
        try:
            period = 1.0 / rate_hz if rate_hz > 0 else 0.0
            sent = 0
            for _ in range(max(1, count)):
                msg = msg_type()
                set_message_fields(msg, values)
                pub.publish(msg)
                sent += 1
                if sent < count and period:
                    time.sleep(period)
            time.sleep(0.05)  # дать RMW отправить
        finally:
            self.destroy_publisher(pub)
        return {'topic': topic, 'type': type_str, 'published': sent}

    def service_call(self, service: str, values: dict, type_str: str | None,
                     timeout: float) -> dict:
        if not type_str:
            m = dict(self.get_service_names_and_types())
            if service not in m or not m[service]:
                raise ValueError(f'Cannot resolve type for {service}; pass "type"')
            type_str = m[service][0]
        srv = get_service(type_str)
        client = self.create_client(srv, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'service {service} unavailable')
            req = srv.Request()
            set_message_fields(req, values)
            fut = client.call_async(req)
            resp = self._wait(fut, timeout)
        finally:
            self.destroy_client(client)
        return {'service': service, 'type': type_str,
                'response': message_to_ordereddict(resp)}

    def action_send_goal(self, action: str, values: dict, type_str: str | None,
                         timeout: float) -> dict:
        if not type_str:
            info = self.action_info(action)
            type_str = info['type']
        if not type_str:
            raise ValueError(f'Cannot resolve type for {action}; pass "type"')
        act = get_action(type_str)
        ac = ActionClient(self, act, action)
        try:
            if not ac.wait_for_server(timeout_sec=timeout):
                raise TimeoutError(f'action server {action} unavailable')
            goal = act.Goal()
            set_message_fields(goal, values)
            gh = self._wait(ac.send_goal_async(goal), timeout)
            if not gh.accepted:
                raise RuntimeError('goal rejected by server')
            res = self._wait(gh.get_result_async(), timeout)
        finally:
            ac.destroy()
        return {'action': action, 'type': type_str,
                'status': int(res.status),
                'result': message_to_ordereddict(res.result)}

    # ── ПАРАМЕТРЫ (через сервисы ноды) ────────────────────────────
    def _param_client(self, node: str, srv_type, suffix: str):
        service = node.rstrip('/') + suffix
        cli = self.create_client(srv_type, service)
        if not cli.wait_for_service(timeout_sec=5.0):
            self.destroy_client(cli)
            raise TimeoutError(f'{service} unavailable (node down or no params?)')
        return cli, service

    def param_list(self, node: str) -> dict:
        cli, _ = self._param_client(node, ListParameters, '/list_parameters')
        try:
            req = ListParameters.Request()
            resp = self._wait(cli.call_async(req))
        finally:
            self.destroy_client(cli)
        return {'node': node, 'names': list(resp.result.names)}

    def param_get(self, node: str, names: list[str]) -> dict:
        cli, _ = self._param_client(node, GetParameters, '/get_parameters')
        try:
            req = GetParameters.Request(names=names)
            resp = self._wait(cli.call_async(req))
        finally:
            self.destroy_client(cli)
        return {'node': node,
                'values': {n: _pv_to_py(v) for n, v in zip(names, resp.values)}}

    def param_dump(self, node: str) -> dict:
        names = self.param_list(node)['names']
        if not names:
            return {'node': node, 'values': {}}
        return self.param_get(node, names)

    def param_set(self, node: str, values: dict) -> dict:
        params = []
        for name, val in values.items():
            p = Parameter(name=name, value=_py_to_pv(val))
            params.append(p)
        cli, _ = self._param_client(node, SetParameters, '/set_parameters')
        try:
            req = SetParameters.Request(parameters=params)
            resp = self._wait(cli.call_async(req))
        finally:
            self.destroy_client(cli)
        return {'node': node, 'results': [
            {'name': list(values.keys())[i], 'successful': r.successful,
             'reason': r.reason}
            for i, r in enumerate(resp.results)]}


# ── ParameterValue <-> python ───────────────────────────────────────────────

def _pv_to_py(v: ParameterValue):
    t = v.type
    return {
        ParameterType.PARAMETER_NOT_SET: None,
        ParameterType.PARAMETER_BOOL: v.bool_value,
        ParameterType.PARAMETER_INTEGER: v.integer_value,
        ParameterType.PARAMETER_DOUBLE: v.double_value,
        ParameterType.PARAMETER_STRING: v.string_value,
        ParameterType.PARAMETER_BYTE_ARRAY: list(v.byte_array_value),
        ParameterType.PARAMETER_BOOL_ARRAY: list(v.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: list(v.integer_array_value),
        ParameterType.PARAMETER_DOUBLE_ARRAY: list(v.double_array_value),
        ParameterType.PARAMETER_STRING_ARRAY: list(v.string_array_value),
    }.get(t, None)


def _py_to_pv(val) -> ParameterValue:
    pv = ParameterValue()
    if isinstance(val, bool):
        pv.type = ParameterType.PARAMETER_BOOL; pv.bool_value = val
    elif isinstance(val, int):
        pv.type = ParameterType.PARAMETER_INTEGER; pv.integer_value = val
    elif isinstance(val, float):
        pv.type = ParameterType.PARAMETER_DOUBLE; pv.double_value = val
    elif isinstance(val, str):
        pv.type = ParameterType.PARAMETER_STRING; pv.string_value = val
    elif isinstance(val, list):
        if all(isinstance(x, bool) for x in val):
            pv.type = ParameterType.PARAMETER_BOOL_ARRAY; pv.bool_array_value = val
        elif all(isinstance(x, int) for x in val):
            pv.type = ParameterType.PARAMETER_INTEGER_ARRAY; pv.integer_array_value = val
        elif all(isinstance(x, (int, float)) for x in val):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(x) for x in val]
        else:
            pv.type = ParameterType.PARAMETER_STRING_ARRAY
            pv.string_array_value = [str(x) for x in val]
    else:
        pv.type = ParameterType.PARAMETER_STRING; pv.string_value = str(val)
    return pv


# ── interface show / proto (без ROS графа) ──────────────────────────────────

def _interface_fields(msg_cls, depth=0, max_depth=6) -> Any:
    if depth > max_depth or not hasattr(msg_cls, 'get_fields_and_field_types'):
        return str(msg_cls)
    out = {}
    fields = msg_cls.get_fields_and_field_types()
    for fname, ftype in fields.items():
        out[fname] = ftype
    return out


def _proto_instance(type_str: str) -> dict:
    kind = type_str.split('/')[1] if '/' in type_str else 'msg'
    if kind == 'srv':
        cls = get_service(type_str)
        return {'request': message_to_ordereddict(cls.Request())}
    if kind == 'action':
        cls = get_action(type_str)
        return {'goal': message_to_ordereddict(cls.Goal())}
    cls = get_message(type_str)
    return message_to_ordereddict(cls())


def _interface_show(type_str: str) -> dict:
    kind = type_str.split('/')[1] if '/' in type_str else 'msg'
    if kind == 'srv':
        cls = get_service(type_str)
        return {'type': type_str,
                'request': _interface_fields(cls.Request),
                'response': _interface_fields(cls.Response)}
    if kind == 'action':
        cls = get_action(type_str)
        return {'type': type_str,
                'goal': _interface_fields(cls.Goal),
                'result': _interface_fields(cls.Result),
                'feedback': _interface_fields(cls.Feedback)}
    cls = get_message(type_str)
    return {'type': type_str, 'fields': _interface_fields(cls)}


# ════════════════════════════════════════════════════════════════════════════
#  Компактная сериализация сообщений (по мотивам rosboard ros2dict)
# ════════════════════════════════════════════════════════════════════════════
#
# message_to_ordereddict дампит сырьё — echo картинки/облака точек/скана
# раздувает контекст LLM. Здесь тяжёлые сенсорные типы сворачиваются в сводку
# (+опционально JPEG для картинок), а длинные числовые массивы обрезаются.
# full=True в echo возвращается к полному message_to_ordereddict.

_ARRAY_CAP = 64  # макс. элементов числового массива, что инлайнится


def _cap_seq(seq: list, cap: int, include_image: bool):
    if not seq:
        return []
    if isinstance(seq[0], (int, float, bool)) and len(seq) > cap:
        return {'_array_len': len(seq), 'head': list(seq[:cap]), 'truncated': True}
    return [_compact(x, cap, include_image) for x in seq]


def _compact(msg, cap: int = _ARRAY_CAP, include_image: bool = False):
    if msg is None or isinstance(msg, (str, bool, int, float)):
        return msg
    if isinstance(msg, bytes):
        return {'_bytes': len(msg)}
    if isinstance(msg, array.array):
        return _cap_seq(msg.tolist(), cap, include_image)
    if np is not None and isinstance(msg, np.ndarray):
        return _cap_seq(msg.tolist(), cap, include_image)
    if isinstance(msg, (list, tuple)):
        return _cap_seq(list(msg), cap, include_image)
    if not hasattr(msg, 'get_fields_and_field_types'):
        return str(msg)

    name = type(msg).__name__
    mod = type(msg).__module__ or ''
    if 'sensor_msgs' in mod and name in ('Image', 'CompressedImage'):
        return _summ_image(msg, include_image)
    if 'sensor_msgs' in mod and name == 'PointCloud2':
        return _summ_pointcloud(msg)
    if 'sensor_msgs' in mod and name == 'LaserScan':
        return _summ_laserscan(msg, cap)
    if 'nav_msgs' in mod and name == 'OccupancyGrid':
        return _summ_occgrid(msg)

    return {f: _compact(getattr(msg, f), cap, include_image)
            for f in msg.get_fields_and_field_types()}


def _summ_image(msg, include_image: bool) -> dict:
    compressed = type(msg).__name__ == 'CompressedImage'
    out = {'_type': type(msg).__name__,
           'header': _compact(msg.header),
           'size_bytes': len(bytes(msg.data))}
    if compressed:
        out['format'] = msg.format
    else:
        out.update({'width': msg.width, 'height': msg.height,
                    'encoding': msg.encoding, 'step': msg.step})
    if include_image:
        jpeg = _to_jpeg_b64(msg, compressed)
        if jpeg:
            out['jpeg_base64'] = jpeg
            out['jpeg_bytes'] = len(jpeg) * 3 // 4
    return out


def _summ_pointcloud(msg) -> dict:
    return {'_type': 'PointCloud2', 'header': _compact(msg.header),
            'width': msg.width, 'height': msg.height,
            'num_points': msg.width * msg.height,
            'point_step': msg.point_step, 'row_step': msg.row_step,
            'is_dense': msg.is_dense, 'size_bytes': len(bytes(msg.data)),
            'fields': [{'name': f.name, 'offset': f.offset,
                        'datatype': f.datatype, 'count': f.count}
                       for f in msg.fields]}


def _summ_laserscan(msg, cap: int) -> dict:
    ranges = list(msg.ranges)
    return {'_type': 'LaserScan', 'header': _compact(msg.header),
            'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min, 'range_max': msg.range_max,
            'num_ranges': len(ranges),
            'ranges_head': [round(float(r), 3) for r in ranges[:cap]],
            'ranges_truncated': len(ranges) > cap}


def _summ_occgrid(msg) -> dict:
    return {'_type': 'OccupancyGrid', 'header': _compact(msg.header),
            'width': msg.info.width, 'height': msg.info.height,
            'num_cells': msg.info.width * msg.info.height,
            'resolution': msg.info.resolution,
            'origin': _compact(msg.info.origin),
            'size_bytes': len(bytes(msg.data))}


def _image_to_cv2(msg):
    enc = (msg.encoding or '').lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == 'rgb8' else img
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.width)
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
        return img[:, :, ::-1] if enc == 'rgba8' else img
    return None  # неизвестная кодировка → только метаданные


def _to_jpeg_b64(msg, compressed: bool, max_dim: int = 640, quality: int = 50):
    if np is None:
        return None
    try:
        import cv2
        if compressed:
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8),
                               cv2.IMREAD_COLOR)
        else:
            img = _image_to_cv2(msg)
        if img is None:
            return None
        h, w = img.shape[:2]
        scale = min(1.0, max_dim / max(h, w))
        if scale < 1.0:
            img = cv2.resize(img, (int(w * scale), int(h * scale)))
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf.tobytes()).decode() if ok else None
    except Exception:
        return None


# ════════════════════════════════════════════════════════════════════════════
#  Глобальный state + FastAPI
# ════════════════════════════════════════════════════════════════════════════

_node: McpBridgeNode | None = None
_executor: MultiThreadedExecutor | None = None
_ros_thread: threading.Thread | None = None


from contextlib import asynccontextmanager


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global _node, _executor, _ros_thread, _policy
    _policy = Policy(os.environ.get('MCP_POLICY_FILE', '/policy/mcp_policy.yaml'))
    print(f'[policy] {_policy.summary()["source"]}  mode={_policy.mode}')
    rclpy.init()
    _node = McpBridgeNode()
    _executor = MultiThreadedExecutor(num_threads=6)
    _executor.add_node(_node)
    _ros_thread = threading.Thread(target=_executor.spin, daemon=True)
    _ros_thread.start()
    yield
    _executor.shutdown()
    _node.destroy_node()
    rclpy.shutdown()


app = FastAPI(title='sverk-ros2-mcp', version='0.4.0', lifespan=lifespan)


# ── Каталог инструментов ────────────────────────────────────────────────────

_O = {'type': 'object'}
TOOLS = [
    # ---- граф (read) ----
    {'name': 'ros2_list_nodes', 'category': 'read',
     'description': 'List all active ROS2 nodes.', 'inputSchema': _O},
    {'name': 'ros2_list_topics', 'category': 'read',
     'description': 'List all active ROS2 topics with their message types.', 'inputSchema': _O},
    {'name': 'ros2_list_services', 'category': 'read',
     'description': 'List all active ROS2 services with their types.', 'inputSchema': _O},
    {'name': 'ros2_list_actions', 'category': 'read',
     'description': 'List all active ROS2 actions with their types.', 'inputSchema': _O},
    {'name': 'ros2_node_info', 'category': 'read',
     'description': 'Publishers/subscribers/services/clients of a node.',
     'inputSchema': {'type': 'object', 'required': ['node'],
                     'properties': {'node': {'type': 'string'}}}},
    {'name': 'ros2_topic_info', 'category': 'read',
     'description': 'Type, publisher/subscriber count and QoS of a topic.',
     'inputSchema': {'type': 'object', 'required': ['topic'],
                     'properties': {'topic': {'type': 'string'}}}},
    {'name': 'ros2_action_info', 'category': 'read',
     'description': 'Type and availability of an action.',
     'inputSchema': {'type': 'object', 'required': ['action'],
                     'properties': {'action': {'type': 'string'}}}},
    {'name': 'ros2_interface_show', 'category': 'read',
     'description': 'Field layout of a msg/srv/action type, e.g. '
                    '"std_srvs/srv/Trigger" or "geometry_msgs/msg/Twist".',
     'inputSchema': {'type': 'object', 'required': ['type'],
                     'properties': {'type': {'type': 'string'}}}},
    {'name': 'ros2_interface_proto', 'category': 'read',
     'description': 'A ready-to-fill prototype (all-default instance) of a '
                    'msg/srv/action type — use it to build publish/call/goal payloads.',
     'inputSchema': {'type': 'object', 'required': ['type'],
                     'properties': {'type': {'type': 'string'}}}},
    # ---- чтение состояния ----
    {'name': 'ros2_topic_echo_once', 'category': 'read',
     'description': 'Read one message from a topic (QoS auto-matched). '
                    'Example: /aruco/world_pose, /fmu/out/vehicle_status. '
                    'By default returns a COMPACT form: heavy sensor types '
                    '(Image/PointCloud2/LaserScan/OccupancyGrid) are summarised and '
                    'long numeric arrays are capped, so echoing a camera/LiDAR topic '
                    'stays small. full=true returns the raw full message; '
                    'include_image=true embeds a downscaled JPEG (base64) for image topics.',
     'inputSchema': {'type': 'object', 'required': ['topic'],
                     'properties': {'topic': {'type': 'string'},
                                    'timeout': {'type': 'number', 'default': 5.0},
                                    'qos': {'type': 'object'},
                                    'full': {'type': 'boolean', 'default': False},
                                    'include_image': {'type': 'boolean', 'default': False}}}},
    {'name': 'ros2_topic_hz', 'category': 'read',
     'description': 'Measure publish rate of a topic over a time window.',
     'inputSchema': {'type': 'object', 'required': ['topic'],
                     'properties': {'topic': {'type': 'string'},
                                    'duration': {'type': 'number', 'default': 3.0},
                                    'qos': {'type': 'object'}}}},
    {'name': 'ros2_param_list', 'category': 'read',
     'description': 'List parameter names of a node.',
     'inputSchema': {'type': 'object', 'required': ['node'],
                     'properties': {'node': {'type': 'string'}}}},
    {'name': 'ros2_param_get', 'category': 'read',
     'description': 'Get parameter values of a node.',
     'inputSchema': {'type': 'object', 'required': ['node', 'names'],
                     'properties': {'node': {'type': 'string'},
                                    'names': {'type': 'array', 'items': {'type': 'string'}}}}},
    {'name': 'ros2_param_dump', 'category': 'read',
     'description': 'Get ALL parameters of a node.',
     'inputSchema': {'type': 'object', 'required': ['node'],
                     'properties': {'node': {'type': 'string'}}}},
    # ---- запись (policy-gated) ----
    {'name': 'ros2_topic_publish', 'category': 'write',
     'description': 'Publish a message to a topic. Build "values" from '
                    'ros2_interface_proto. Optional count/rate_hz for streaming. '
                    'GATED by policy (topic_publish).',
     'inputSchema': {'type': 'object', 'required': ['topic', 'values'],
                     'properties': {'topic': {'type': 'string'},
                                    'values': {'type': 'object'},
                                    'type': {'type': 'string'},
                                    'count': {'type': 'integer', 'default': 1},
                                    'rate_hz': {'type': 'number', 'default': 0.0},
                                    'qos': {'type': 'object'},
                                    '_confirm': {'type': 'boolean'}}}},
    {'name': 'ros2_service_call', 'category': 'write',
     'description': 'Call a service with a request dict. GATED by policy (service_call).',
     'inputSchema': {'type': 'object', 'required': ['service'],
                     'properties': {'service': {'type': 'string'},
                                    'values': {'type': 'object', 'default': {}},
                                    'type': {'type': 'string'},
                                    'timeout': {'type': 'number', 'default': 15.0},
                                    '_confirm': {'type': 'boolean'}}}},
    {'name': 'ros2_action_send_goal', 'category': 'write',
     'description': 'Send an action goal and wait for the result. '
                    'GATED by policy (action_send_goal).',
     'inputSchema': {'type': 'object', 'required': ['action'],
                     'properties': {'action': {'type': 'string'},
                                    'values': {'type': 'object', 'default': {}},
                                    'type': {'type': 'string'},
                                    'timeout': {'type': 'number', 'default': 30.0},
                                    '_confirm': {'type': 'boolean'}}}},
    {'name': 'ros2_param_set', 'category': 'write',
     'description': 'Set node parameters from a {name: value} dict. '
                    'GATED by policy (param_set).',
     'inputSchema': {'type': 'object', 'required': ['node', 'values'],
                     'properties': {'node': {'type': 'string'},
                                    'values': {'type': 'object'},
                                    '_confirm': {'type': 'boolean'}}}},
    {'name': 'shell_exec', 'category': 'write',
     'description': 'Run a shell command inside the MCP container with the ROS2 '
                    'environment pre-sourced. Covers colcon build, source, find, '
                    'tree, rm, touch, mkdir, ls, cat, grep, etc. stdin is closed, so '
                    'interactive TUIs (nano, htop, vi, less) exit immediately — use '
                    'file_write to edit files and "top -b -n1"/ps/free for processes. '
                    'Runs in sverk_mcp (NOT the drone container); /home/sverk is a '
                    'shared volume with SITL, so workspace/file ops are shared, but '
                    'processes are the MCP container\'s. GATED by policy (shell_exec).',
     'inputSchema': {'type': 'object', 'required': ['command'],
                     'properties': {'command': {'type': 'string'},
                                    'cwd': {'type': 'string', 'default': '/home/sverk'},
                                    'timeout': {'type': 'number', 'default': 60.0},
                                    'source_ros': {'type': 'boolean', 'default': True},
                                    '_confirm': {'type': 'boolean'}}}},
    # ---- файлы ----
    {'name': 'file_list', 'category': 'read',
     'description': 'List files/dirs at a path (allowed roots only).',
     'inputSchema': {'type': 'object', 'required': ['path'],
                     'properties': {'path': {'type': 'string'}}}},
    {'name': 'file_read', 'category': 'read',
     'description': 'Read a text file (allowed roots only).',
     'inputSchema': {'type': 'object', 'required': ['path'],
                     'properties': {'path': {'type': 'string'},
                                    'max_bytes': {'type': 'integer', 'default': 65536}}}},
    {'name': 'file_write', 'category': 'write',
     'description': 'Write a text file (allowed roots only). GATED by policy (file_write).',
     'inputSchema': {'type': 'object', 'required': ['path', 'content'],
                     'properties': {'path': {'type': 'string'},
                                    'content': {'type': 'string'},
                                    '_confirm': {'type': 'boolean'}}}},
    # ---- служебное ----
    {'name': 'policy_show', 'category': 'read',
     'description': 'Show the active MCP policy (mode, rules, limits).',
     'inputSchema': _O},
    # ---- LLM/VLM (если собраны) ----
    {'name': 'llm_query', 'category': 'read',
     'description': 'Send a text prompt to the on-board LLM.',
     'inputSchema': {'type': 'object', 'required': ['prompt'],
                     'properties': {'prompt': {'type': 'string'},
                                    'system_prompt': {'type': 'string', 'default': ''},
                                    'max_tokens': {'type': 'integer', 'default': 256},
                                    'temperature': {'type': 'number', 'default': 0.7},
                                    'use_npu': {'type': 'boolean', 'default': False}}}},
    {'name': 'vlm_query', 'category': 'read',
     'description': 'Ask the on-board VLM about an image file.',
     'inputSchema': {'type': 'object', 'required': ['image_path', 'query'],
                     'properties': {'image_path': {'type': 'string'},
                                    'query': {'type': 'string'},
                                    'use_npu': {'type': 'boolean', 'default': False}}}},
]


def _visible_tools() -> list:
    _policy.reload()
    out = []
    for t in TOOLS:
        if t['name'] == 'vlm_query' and not HAS_VLM:
            continue
        if t['name'] == 'llm_query' and not HAS_LLM:
            continue
        if not _policy.tool_enabled(t['name']):
            continue
        out.append({k: t[k] for k in ('name', 'description', 'inputSchema')})
    return out


# ── Эндпоинты ────────────────────────────────────────────────────────────────

@app.get('/healthz')
async def healthz():
    return {'ok': True, 'node_ready': _node is not None,
            'policy_mode': _policy.mode if _policy else None}


@app.get('/tools')
async def list_tools():
    return {'tools': _visible_tools()}


@app.get('/policy')
async def get_policy():
    _policy.reload()
    return _policy.summary()


class CallRequest(BaseModel):
    name: str
    arguments: dict = {}


@app.post('/call')
async def call_tool(req: CallRequest):
    if _node is None:
        raise HTTPException(503, 'ROS2 node not initialised')
    try:
        result = await asyncio.get_event_loop().run_in_executor(
            None, _dispatch_sync, req.name, req.arguments)
        return {'content': [{'type': 'text',
                             'text': json.dumps(result, ensure_ascii=False, default=str)}]}
    except PolicyError as e:
        raise HTTPException(403, str(e))
    except (ValueError, FileNotFoundError, KeyError) as e:
        raise HTTPException(400, f'{type(e).__name__}: {e}')
    except PermissionError as e:
        raise HTTPException(403, str(e))
    except TimeoutError as e:
        raise HTTPException(504, str(e))
    except Exception as e:
        traceback.print_exc()
        raise HTTPException(500, f'{type(e).__name__}: {e}')


# ── Диспетчер ────────────────────────────────────────────────────────────────

def _policy_target(name: str, args: dict) -> str:
    return {
        'ros2_topic_publish': args.get('topic', ''),
        'ros2_service_call': args.get('service', ''),
        'ros2_action_send_goal': args.get('action', ''),
        'ros2_param_set': args.get('node', ''),
        'file_write': args.get('path', ''),
        'shell_exec': args.get('command', ''),
    }.get(name, '')


def _dispatch_sync(name: str, args: dict) -> Any:
    _policy.reload()
    if name in WRITE_TOOLS:
        _policy.check(name, _policy_target(name, args), args)

    # ---- граф ----
    if name == 'ros2_list_nodes':
        return {'nodes': _node.get_node_names()}
    if name == 'ros2_list_topics':
        return {'topics': [{'name': n, 'types': t}
                           for n, t in _node.get_topic_names_and_types()]}
    if name == 'ros2_list_services':
        return {'services': [{'name': n, 'types': t}
                             for n, t in _node.get_service_names_and_types()]}
    if name == 'ros2_list_actions':
        return _node.list_actions()
    if name == 'ros2_node_info':
        return _node.node_info(args['node'])
    if name == 'ros2_topic_info':
        return _node.topic_info(args['topic'])
    if name == 'ros2_action_info':
        return _node.action_info(args['action'])
    if name == 'ros2_interface_show':
        return _interface_show(args['type'])
    if name == 'ros2_interface_proto':
        return {'type': args['type'], 'proto': _proto_instance(args['type'])}

    # ---- чтение ----
    if name == 'ros2_topic_echo_once':
        return _node.echo_once(args['topic'], args.get('timeout', 5.0), args.get('qos'),
                               args.get('full', False), args.get('include_image', False))
    if name == 'ros2_topic_hz':
        return _node.topic_hz(args['topic'], args.get('duration', 3.0), args.get('qos'))
    if name == 'ros2_param_list':
        return _node.param_list(args['node'])
    if name == 'ros2_param_get':
        return _node.param_get(args['node'], args['names'])
    if name == 'ros2_param_dump':
        return _node.param_dump(args['node'])

    # ---- запись ----
    if name == 'ros2_topic_publish':
        return _node.publish(args['topic'], args['values'], args.get('type'),
                             args.get('count', 1), args.get('rate_hz', 0.0), args.get('qos'))
    if name == 'ros2_service_call':
        return _node.service_call(args['service'], args.get('values', {}),
                                  args.get('type'), args.get('timeout', 15.0))
    if name == 'ros2_action_send_goal':
        return _node.action_send_goal(args['action'], args.get('values', {}),
                                      args.get('type'), args.get('timeout', 30.0))
    if name == 'ros2_param_set':
        return _node.param_set(args['node'], args['values'])

    # ---- файлы ----
    if name == 'file_list':
        return _file_list(args['path'])
    if name == 'file_read':
        return _file_read(args['path'], args.get('max_bytes', 65536))
    if name == 'file_write':
        return _file_write(args['path'], args['content'])

    # ---- shell ----
    if name == 'shell_exec':
        return _shell_exec(args.get('command', ''), args.get('cwd'),
                           args.get('timeout', 60.0), args.get('source_ros', True))

    # ---- служебное ----
    if name == 'policy_show':
        return _policy.summary()

    # ---- LLM / VLM ----
    if name == 'llm_query':
        return _node_llm(args)
    if name == 'vlm_query':
        return _node_vlm(args)

    raise ValueError(f'Unknown tool: {name!r}')


# ── LLM / VLM (action-based, если interfaces собраны) ───────────────────────

def _node_llm(args: dict) -> dict:
    if not HAS_LLM:
        raise ValueError('llm_interfaces not installed in this image')
    client = _node._ac['llm']
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError('/llm/query action server not available')
    goal = LlmQuery.Goal()
    goal.prompt = args['prompt']
    goal.system_prompt = args.get('system_prompt', '')
    goal.max_tokens = args.get('max_tokens', 256)
    goal.temperature = args.get('temperature', 0.7)
    goal.use_npu = args.get('use_npu', False)
    gh = _node._wait(client.send_goal_async(goal), 10.0)
    if not gh.accepted:
        raise RuntimeError('LLM goal rejected')
    res = _node._wait(gh.get_result_async(), 120.0).result
    if not res.success:
        raise RuntimeError(res.error_msg)
    return {'response': res.response, 'inference_time_ms': res.inference_time_ms,
            'tokens_per_second': res.tokens_per_second}


def _node_vlm(args: dict) -> dict:
    if not HAS_VLM:
        raise ValueError('vlm_interfaces not installed in this image')
    import cv2
    from cv_bridge import CvBridge
    client = _node._ac['vlm']
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError('/vlm/query action server not available')
    cv_img = cv2.imread(args['image_path'])
    if cv_img is None:
        raise ValueError(f'Cannot read image: {args["image_path"]}')
    goal = VlmQuery.Goal()
    goal.image = CvBridge().cv2_to_imgmsg(cv_img, encoding='bgr8')
    goal.query = args['query']
    goal.use_npu_vision = args.get('use_npu', False)
    gh = _node._wait(client.send_goal_async(goal), 10.0)
    if not gh.accepted:
        raise RuntimeError('VLM goal rejected')
    res = _node._wait(gh.get_result_async(), 120.0).result
    if not res.success:
        raise RuntimeError(res.error_msg)
    return {'response': res.response, 'inference_time_ms': res.inference_time_ms}


# ── Файловые операции ───────────────────────────────────────────────────────

def _check_path(path_str: str) -> Path:
    p = Path(path_str).resolve()
    for root in ALLOWED_ROOTS:
        try:
            p.relative_to(root)
            return p
        except ValueError:
            continue
    raise PermissionError(
        f'{path_str!r} outside allowed roots: '
        + ', '.join(str(r) for r in ALLOWED_ROOTS))


def _file_list(path_str: str) -> dict:
    p = _check_path(path_str)
    if not p.exists():
        raise FileNotFoundError(str(p))
    entries = [{'name': e.name, 'type': 'dir' if e.is_dir() else 'file',
                'size': e.stat().st_size if e.is_file() else None}
               for e in sorted(p.iterdir())]
    return {'path': str(p), 'entries': entries}


def _file_read(path_str: str, max_bytes: int) -> dict:
    p = _check_path(path_str)
    size = p.stat().st_size
    with open(p, 'r', errors='replace') as f:
        content = f.read(max_bytes)
    return {'path': str(p), 'content': content, 'truncated': size > max_bytes}


def _file_write(path_str: str, content: str) -> dict:
    p = _check_path(path_str)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content)
    return {'path': str(p), 'bytes_written': len(content.encode())}


# ── shell exec ──────────────────────────────────────────────────────────────

_SHELL_OUT_CAP = 20000  # chars per stream

def _shell_exec(command: str, cwd: str | None, timeout: float,
                source_ros: bool) -> dict:
    if not command or not command.strip():
        raise ValueError('empty command')
    workdir = cwd or '/home/sverk'
    if not os.path.isdir(workdir):
        raise ValueError(f'cwd does not exist: {workdir}')
    prefix = ''
    if source_ros:
        prefix = ('source /opt/ros/humble/setup.bash 2>/dev/null; '
                  'source /home/sverk/sverk_ws/install/setup.bash 2>/dev/null; ')
    timeout = max(1.0, min(float(timeout), 1800.0))
    t0 = time.monotonic()
    try:
        # stdin=DEVNULL: interactive TUIs (nano/htop/vi) see EOF and exit
        # instead of hanging until the timeout.
        proc = subprocess.run(
            ['bash', '-lc', prefix + command], cwd=workdir,
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=timeout, errors='replace')
    except subprocess.TimeoutExpired:
        raise TimeoutError(f'command timed out after {timeout}s: {command[:80]}')
    out, err = proc.stdout or '', proc.stderr or ''
    return {
        'command': command, 'cwd': workdir, 'returncode': proc.returncode,
        'duration_s': round(time.monotonic() - t0, 2),
        'stdout': out[:_SHELL_OUT_CAP], 'stderr': err[:_SHELL_OUT_CAP],
        'truncated': len(out) > _SHELL_OUT_CAP or len(err) > _SHELL_OUT_CAP,
    }


# ── Точка входа ─────────────────────────────────────────────────────────────

def main():
    uvicorn.run(app, host=os.environ.get('MCP_HOST', '0.0.0.0'),
                port=int(os.environ.get('MCP_PORT', '8765')), log_level='info')


if __name__ == '__main__':
    main()
