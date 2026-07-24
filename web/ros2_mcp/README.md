# ros2_mcp

HTTP-мост между [picoclaw](https://github.com/petayyyy/piko_claw) (LLM-мозг) и стеком sverk-ros2. Даёт агенту почти весь `ros2` CLI + shell как MCP-инструменты, а всё «пишущее» режет **файлом-политикой**, границы в котором задаёте вы.

Запускается как rclpy-нода + FastAPI в одном процессе. picoclaw спавнит тонкий stdio-шим (`ros2_mcp_shim.py`), тот проксирует JSON-RPC вызовы сюда по HTTP.

```
picoclaw (LLM brain)
  └── ros2_mcp_shim.py       (stdio JSON-RPC 2.0)
           │ HTTP POST /call
           ▼
      sverk_mcp :8765         ← policy gate (mcp_policy.yaml)
      ├── FastAPI   /tools /call /healthz /policy
      └── rclpy     граф, echo, publish, сервисы, actions, параметры
               │ ROS2 DDS
               ▼
      sverk_sitl (PX4 SITL + Gazebo + aruco/fmu/offboard ...)
```

---

## Содержание

- [Запуск в SITL](#запуск-в-sitl)
- [HTTP API](#http-api)
- [Инструменты (23)](#инструменты-23)
- [Примеры вызовов](#примеры-вызовов)
- [Политика: как выставлять ограничения](#политика-как-выставлять-ограничения) ← главное
- [Подключение picoclaw](#подключение-picoclaw)
- [Развёртывание и обновление](#развёртывание-и-обновление)
- [Связанные файлы](#связанные-файлы)

---

## Запуск в SITL

MCP поднимается как addon к SITL-стеку:

```bash
docker compose --project-directory . \
  -f scripts/sitl/docker-compose.sitl.yml \
  -f scripts/docker/docker-compose.mcp.yml \
  -f scripts/sitl/docker-compose.sitl.mcp.yml \
  up -d
```

Что делает SITL-overlay ([docker-compose.sitl.mcp.yml](../../../../../scripts/sitl/docker-compose.sitl.mcp.yml)):
- **сам запускает сервер** (переопределяет `entrypoint`, т.к. базовый образ его глотает);
- **подключает mcp к сети `sverk_sitl`** — иначе DDS-discovery до графа дрона не проходит;
- **монтирует политику** `scripts/mcp/mcp_policy.yaml` → `/policy/mcp_policy.yaml`.

Проверка, что сервер жив:

```bash
curl -s http://localhost:8765/healthz
# {"ok":true,"node_ready":true,"policy_mode":"guarded"}
```

> Ноды дрона (aruco/fmu/gazebo) запускаются **вручную внутри `sverk_sitl`**, не через compose:
> ```bash
> ros2 launch launch_system full_system_sitl.launch.py headless:=true obrik_config:=base gz_world:=obrik_aruco
> ```
> Пока launch не запущен, MCP покажет почти пустой граф — это нормально.

---

## HTTP API

| Метод | Путь | Назначение |
|---|---|---|
| `GET`  | `/healthz` | жив ли сервер + текущий `policy_mode` |
| `GET`  | `/tools`   | список доступных инструментов (учитывает политику) |
| `GET`  | `/policy`  | активная политика (mode, rules) |
| `POST` | `/call`    | вызвать инструмент |

Формат вызова:

```bash
curl -s -X POST http://localhost:8765/call \
  -H 'Content-Type: application/json' \
  -d '{"name":"<tool>","arguments":{ ... }}'
```

Ответ — MCP-конверт; полезные данные в `content[0].text` (JSON-строка):

```json
{"content":[{"type":"text","text":"{\"nodes\": [\"fmu_control\", ...]}"}]}
```

Ошибки — HTTP-коды: `400` неверные аргументы, `403` запрещено политикой, `504` таймаут, `500` прочее.

---

## Инструменты (23)

Инструменты чтения доступны всегда. **Пишущие ( ) проходят через политику** — по умолчанию (`guarded`) работают только по allow-паттернам.

### Граф и интроспекция (read)

| Инструмент | Аргументы | Что делает |
|---|---|---|
| `ros2_list_nodes` | — | список нод |
| `ros2_list_topics` | — | топики + типы |
| `ros2_list_services` | — | сервисы + типы |
| `ros2_list_actions` | — | actions + типы |
| `ros2_node_info` | `node` | pub/sub/сервисы ноды (работает и для удалённых нод) |
| `ros2_topic_info` | `topic` | тип, число pub/sub, QoS |
| `ros2_action_info` | `action` | тип и доступность action |
| `ros2_interface_show` | `type` | поля msg/srv/action (напр. `geometry_msgs/msg/Twist`) |
| `ros2_interface_proto` | `type` | готовый шаблон для заполнения payload |

### Чтение состояния (read)

| Инструмент | Аргументы | Что делает |
|---|---|---|
| `ros2_topic_echo_once` | `topic`, `timeout=5`, `qos?`, `full=false`, `include_image=false` | одно сообщение (QoS авто-подбор); по умолчанию **компактно** — тяжёлые типы (Image/PointCloud2/LaserScan/OccupancyGrid) сворачиваются в сводку, длинные массивы обрезаются¹ |
| `ros2_topic_hz` | `topic`, `duration=3`, `qos?` | частота публикации за окно |
| `ros2_param_list` | `node` | имена параметров ноды |
| `ros2_param_get` | `node`, `names[]` | значения параметров |
| `ros2_param_dump` | `node` | все параметры ноды |

¹ **Компактный echo** (по мотивам rosboard): `echo /camera_1/image_raw` в full-режиме — это ~18 МБ сырых пикселей в JSON, что забивает контекст LLM. По умолчанию (`full=false`) картинка сворачивается в `{width, height, encoding, step, size_bytes}` (~300 символов); `include_image=true` добавит даунскейл-JPEG (base64, 640px). `full=true` возвращает полный дамп сообщения.

### Актуация (write — gated)

| Инструмент | Аргументы | Категория политики |
|---|---|---|
| `ros2_topic_publish` | `topic`, `values`, `type?`, `count=1`, `rate_hz=0`, `qos?` | `topic_publish` |
| `ros2_service_call` | `service`, `values={}`, `type?`, `timeout=15` | `service_call` |
| `ros2_action_send_goal` | `action`, `values={}`, `type?`, `timeout=30` | `action_send_goal` |
| `ros2_param_set` | `node`, `values{}` | `param_set` |
| `shell_exec` | `command`, `cwd=/home/sverk`, `timeout=60`, `source_ros=true` | `shell_exec` |

### Файлы

| Инструмент | Аргументы | Примечание |
|---|---|---|
| `file_list` / `file_read` | `path` (+`max_bytes`) | read; только разрешённые корни¹ |
| `file_write` | `path`, `content` | gated (`file_write`) |

¹ разрешённые корни: `/home/sverk`, `/tmp`, `/models`, `/shared`, `/policy`. Запрос за их пределы → 400.

### Служебное / LLM

| Инструмент | Примечание |
|---|---|
| `policy_show` | показать активную политику |
| `llm_query` / `vlm_query` | появляются только если собраны `llm_interfaces` / `vlm_interfaces` (в SITL их нет) |

---

## Примеры вызовов

```bash
# Список нод
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"ros2_list_nodes","arguments":{}}'

# Телеметрия — одно сообщение
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"ros2_topic_echo_once","arguments":{"topic":"/fmu/out/vehicle_local_position"}}'

# Узнать схему типа перед публикацией/вызовом
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"ros2_interface_proto","arguments":{"type":"offboard_interfaces/action/NavigateTo"}}'

# Публикация (пройдёт только если topic подходит под rules.topic_publish.allow)
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"ros2_topic_publish","arguments":{"topic":"/offboard/setpoint","type":"std_msgs/msg/String","values":{"data":"go"}}}'

# Вызов сервиса дрона
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"ros2_service_call","arguments":{"service":"/fmu_control/get_param","type":"offboard_interfaces/srv/GetParam","values":{"name":"SYS_AUTOSTART"}}}'

# Shell: сборка workspace
curl -s -X POST http://localhost:8765/call -H 'Content-Type: application/json' \
  -d '{"name":"shell_exec","arguments":{"command":"colcon build --packages-select ros2_mcp","cwd":"/home/sverk/sverk_ws","timeout":300}}'
```

Быстрый smoke-тест изнутри контейнера picoclaw:

```bash
docker exec sverk_picoclaw bash /app/test_mcp.sh
```

---

## Политика: как выставлять ограничения

**Файл:** [`scripts/mcp/mcp_policy.yaml`](../../../../../scripts/mcp/mcp_policy.yaml) на хосте → монтируется как `/policy/mcp_policy.yaml`.
**Hot-reload:** сервер перечитывает файл по mtime — **правьте на хосте, перезапуск контейнера не нужен.**
**Fail-safe:** если файла нет или он битый → сервер уходит в `read_only` (запись запрещена).

Посмотреть, что сервер применяет прямо сейчас:

```bash
curl -s http://localhost:8765/policy
```

### 1. `mode` — глобальный режим записи

| `mode` | Поведение пишущих инструментов |
|---|---|
| `read_only` | запрещены полностью и **скрыты** из `/tools` |
| `guarded` *(дефолт)* | разрешены **только** если цель совпала с `allow` (и не с `deny`) |
| `full` | разрешены; проверяется только `deny` (`allow` игнорируется) |

### 2. `rules` — тонкая настройка по цели

Паттерны — регэкспы (`re.search`). **`deny` всегда сильнее `allow`.** У каждой категории своя «цель» (target):

| Категория | target — это... |
|---|---|
| `topic_publish` | имя топика |
| `service_call` | имя сервиса |
| `action_send_goal` | имя action |
| `param_set` | имя ноды |
| `file_write` | путь файла |
| `shell_exec` | вся командная строка |

### 3. Прочее

- **`tools:`** — жёсткий вкл/выкл отдельного инструмента поверх `mode`: `shell_exec: false` выключит совсем (даже в `full`); `ros2_topic_publish: true` включит даже в `read_only`.
- **`require_confirm: true`** — каждый пишущий вызов должен прийти с `arguments._confirm: true` (двойное подтверждение).

### Рецепты

**Полностью «только чтение» (безопасный просмотр):**
```yaml
mode: read_only
```

**Разрешить полёт, но запретить опасное для железа** (близко к дефолту):
```yaml
mode: guarded
rules:
  topic_publish:
    allow: ['^/fmu/in/.*', '^/offboard/.*']
  service_call:
    allow: ['.*']
    deny:  ['.*stop_all_motors.*', '.*motor_test.*', '.*/reboot.*']
  action_send_goal:
    allow: ['^/navigate_to$', '^/fmu_control/calibrate$']
```

**Разрешить сборку, но запретить удаление:**
```yaml
mode: guarded
rules:
  shell_exec:
    allow: ['^colcon( |$)', '^ros2( |$)', '^(ls|cat|find|touch|mkdir)( |$)']
    deny:  ['\brm\b']          # rm заблокирован целиком
```

**Требовать подтверждение на любую актуацию:**
```yaml
mode: guarded
require_confirm: true          # вызовы без "_confirm": true → 403
```

**Всё разрешить (только для локальной отладки!):**
```yaml
mode: full                     # deny всё ещё действует — оставьте катастрофические паттерны
```

**Выключить именно shell, оставив ROS-актуацию:**
```yaml
mode: guarded
tools:
  shell_exec: false
```

### Что вернётся при запрете

```json
// HTTP 403
{"detail":"service_call on '/fmu_control/motor_test' denied by policy (rules.service_call.deny ~ '.*motor_test.*')"}
```

> `shell_exec` — самый мощный инструмент (произвольные команды в контейнере, `/home/sverk` — общий volume с SITL). Дефолтный `deny` ловит `rm -rf /`, снос `/home/sverk`, `mkfs`, `dd`, `reboot`, fork-bomb. Интерактивные TUI (`nano`, `htop`, `vi`) не в `allow` — они не работают по HTTP; для правки файлов используйте `file_write`, для процессов — `shell_exec "top -b -n1"`.

---

## Подключение picoclaw

В [`scripts/picoclaw/picoclaw.config.json`](../../../../../scripts/picoclaw/picoclaw.config.json):

```json
{
  "tools": {
    "mcp": {
      "servers": {
        "sverk_ros2": {
          "command": "python3",
          "args": ["/app/ros2_mcp_shim.py"],
          "env": { "SVERK_MCP_URL": "http://sverk_mcp:8765" }
        }
      }
    }
  }
}
```

picoclaw спавнит `ros2_mcp_shim.py` как stdio-процесс; шим проксирует `tools/list` и `tools/call` в этот HTTP-сервер.

---

## Развёртывание и обновление

**Переменные окружения сервера:**

| Переменная | Дефолт | Описание |
|---|---|---|
| `MCP_PORT` | `8765` | порт HTTP |
| `MCP_HOST` | `0.0.0.0` | bind-адрес |
| `MCP_POLICY_FILE` | `/policy/mcp_policy.yaml` | путь к файлу-политике |

**Про пересборку образа:** для изменений `server.py` **пересборка не нужна** — все зависимости уже в образе. `/home/sverk` в контейнере — это named volume `shared_sitl_fs`, поэтому рабочий `server.py` лежит **в volume**, а не в образе. Обновить на лету:

```bash
docker cp sverk_ws/src/sverk_drone/web/ros2_mcp/ros2_mcp/server.py \
  sverk_mcp:/home/sverk/sverk_ws/src/sverk_drone/web/ros2_mcp/ros2_mcp/server.py
docker restart sverk_mcp
```

Файл переживает `restart`/`recreate` (volume персистентный); теряется только при `docker compose down -v`. Политику `docker cp`-ить не нужно — она bind-монтируется с хоста и подхватывается автоматически.

---

## Связанные файлы

- [`ros2_mcp/server.py`](ros2_mcp/server.py) — сам сервер (инструменты + policy-движок)
- [`scripts/mcp/mcp_policy.yaml`](../../../../../scripts/mcp/mcp_policy.yaml) — **файл-ограничитель**
- [`scripts/picoclaw/ros2_mcp_shim.py`](../../../../../scripts/picoclaw/ros2_mcp_shim.py) — stdio-шим
- [`scripts/picoclaw/picoclaw.config.json`](../../../../../scripts/picoclaw/picoclaw.config.json) — конфиг picoclaw
- [`scripts/sitl/docker-compose.sitl.mcp.yml`](../../../../../scripts/sitl/docker-compose.sitl.mcp.yml) — SITL-overlay (запуск + сеть + mount политики)
- [`scripts/docker/docker-compose.mcp.yml`](../../../../../scripts/docker/docker-compose.mcp.yml) — базовый compose-override
- [`scripts/picoclaw/test_mcp.sh`](../../../../../scripts/picoclaw/test_mcp.sh) — smoke-тест
