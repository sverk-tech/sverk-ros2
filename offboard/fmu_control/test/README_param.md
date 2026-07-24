# Параметры PX4 через uXRCE-DDS (по имени, без MAVLink)

Чтение/запись/сброс параметров PX4 по **имени** прямо через DDS — без MAVLink и
без QGC. Работает на связке:

- **прошивка**: кастомный модуль `param_dds_bridge` (в форке PX4-Autopilot):
  сообщения `ParamBridgeRequest/Response`, топики в `dds_topics.yaml`, модуль в
  `src/modules/param_dds_bridge/`, включён в board-конфиге, стартует в `rcS`;
- **компаньон**: `px4_msgs` с теми же `ParamBridgeRequest/Response.msg`
  (байт-в-байт как в прошивке), нода `fmu_control` с сервисами `get/set/reset_param`.

> Проверено на железе (matek h743-slim, PX4 v1.15.4): set/get/reset идут по чистому
> DDS, изменения видны в QGC.

> Обслуживание форка: `.msg` в прошивке и в `px4_msgs` должны совпадать
> **байт-в-байт**. При правке одного — правь оба и пересобирай оба.

---

## Штатный способ — сервисы fmu_control

```bash
ros2 service call /fmu_control/get_param offboard_interfaces/srv/GetParam \
  '{name: MPC_XY_CRUISE}'
ros2 service call /fmu_control/set_param offboard_interfaces/srv/SetParam \
  '{name: MPC_XY_CRUISE, value: 8.0}'
ros2 service call /fmu_control/reset_param offboard_interfaces/srv/ResetParam \
  '{name: MPC_XY_CRUISE}'          # пустое имя или "*" — сбросить ВСЕ
```

Из Python:

```python
import sverk_interfaces
drone = sverk_interfaces.init(Nodename="params")
try:
    print(drone.fcu.get_param("MPC_XY_CRUISE").value)   # 5.0
    drone.fcu.set_param("MPC_XY_CRUISE", 8.0)
    drone.fcu.set_param("COM_ARM_WO_GPS", 1)            # int — тип выберет FCU
    drone.fcu.reset_param("MPC_XY_CRUISE")
finally:
    drone.close()
```

`set_param` возвращает `success=False`, если PX4 клипнул значение по диапазону —
в `message` будет фактическое значение (readback mismatch).

---

## Низкоуровневый тест — param_dds_test_node.py

Прямой доступ к топикам моста (в обход fmu_control), удобно для отладки:

```bash
source install/setup.bash
python3 param_dds_test_node.py --get MPC_XY_CRUISE
python3 param_dds_test_node.py --set MPC_XY_CRUISE --value 8
python3 param_dds_test_node.py --reset MPC_XY_CRUISE
python3 param_dds_test_node.py --reset-all
```

Нода публикует `ParamBridgeRequest` в `/fmu/in/param_bridge_request` и ждёт
`ParamBridgeResponse`. Имя `char[17]` сериализуется в коде (в `ros2 topic pub`
руками имя пришлось бы задавать массивом байт-кодов).

> Прерывистые таймауты у этой ноды — норма: она короткоживущая, и BEST_EFFORT
> теряет пакет, пока DDS-discovery не смэтчился. В резидентной `fmu_control`
> подписчик всегда смэтчен + есть ретраи, поэтому там стабильно.

---

## Сборка после изменения .msg

`px4_msgs` собирает список через `file(GLOB ...)` — новые `.msg` подхватываются
только при реконфигурации CMake:

```bash
rm -rf build/px4_msgs install/px4_msgs
colcon build --packages-select px4_msgs
# либо: colcon build --packages-select px4_msgs --cmake-force-configure
source install/setup.bash
ros2 interface show px4_msgs/msg/ParamBridgeRequest
```

---

## Что НЕ покрывает этот путь

- **Интерактивный фидбэк калибровки** (положи на грань / вращай / прогресс) — он
  идёт по MAVLink STATUSTEXT, в DDS его нет. Для этого нужен MAVLink-канал.
