# fmu_control — проверка и ручное управление

Инструменты для ручной работы с нодой `fmu_control` и проверки её команд на
живом PX4 (v1.15.4). Здесь — как вызывать команды и как убедиться, что PX4 их
принял.

- `mav_monitor.py` — пассивный MAVLink-монитор: слушает ответы PX4 и печатает
  их. Сам ничего не отправляет.

API ноды (список сервисов и топиков) описан в [../README.md](../README.md).

---

## 1. Транспорт: что куда подключено

UART и USB разделены и не мешают друг другу:

| Линк | Порт | Назначение |
|------|------|------------|
| uXRCE-DDS | UART (`/dev/ttyAMA0` @ 921600, RPI) | ROS 2 ↔ PX4 (сюда уходят команды `fmu_control`) |
| MAVLink | USB (`/dev/ttyACM0` @ 57600) → `mavlink-routerd` → UDP `0.0.0.0:14550` | QGC и `mav_monitor.py` |

Команды `fmu_control` идут по DDS (UART). Монитор слушает MAVLink (USB/UDP) —
это независимый канал, поэтому монитор и QGC можно держать одновременно, и они
не трогают DDS-линк.

---

## 2. Запуск монитора

`mavlink-routerd` должен быть запущен (в основном лаунче — `ENABLE_MAVLINKROUTER`).
Монитор подключается к его UDP-эндпоинту:

```bash
pip install pymavlink            # один раз
python3 mav_monitor.py --conn udpout:127.0.0.1:14550
```

- С другого компьютера в сети — заменить `127.0.0.1` на IP дрона.
- Если `mavlink-routerd` НЕ запущен и USB свободен — можно напрямую:
  `python3 mav_monitor.py --conn /dev/ttyACM0 --baud 57600`.

Монитор печатает три вещи:
- `ACK <команда> -> <результат>` — ответ PX4 на команду
  (`ACCEPTED` / `TEMPORARILY_REJECTED` / `DENIED` / `UNSUPPORTED` / `FAILED` / `IN_PROGRESS`);
- `TEXT <...>` — статусные сообщения PX4 (в т.ч. прогресс калибровки);
- `STATE armed=<bool>` — смена арм/дизарм.

Альтернатива без MAVLink: смотреть ответы прямо в DDS-топике —
`ros2 topic echo /fmu/out/vehicle_command_ack` и состояние —
`ros2 topic echo /fmu/out/vehicle_status` (поле `arming_state`: 1=дизарм, 2=арм).

---

## 3. Ручные команды fmu_control

Два способа: сервисы ROS 2 напрямую или Python-клиент `sverk_interfaces`.
**Перед любым арм/дизарм/kill — снимите пропеллеры.**

### Через ros2 service call

```bash
ros2 service call /fmu_control/arm          std_srvs/srv/Trigger
ros2 service call /fmu_control/disarm       std_srvs/srv/Trigger
ros2 service call /fmu_control/force_disarm std_srvs/srv/Trigger   # игнорирует "not landed"
ros2 service call /fmu_control/kill_switch  std_srvs/srv/Trigger   # моментальная остановка моторов
ros2 service call /fmu_control/reboot       std_srvs/srv/Trigger   # ребут FC (только в дизарме)
```

Калибровки (топик, значение = тип):

```bash
# 0=gyro 1=mag 2=baro 3=accel 4=level 5=esc
ros2 topic pub --once /fmu_control/request_calibration std_msgs/msg/UInt8 "{data: 0}"
```

### Через sverk_interfaces (Python)

```python
import sverk_interfaces
drone = sverk_interfaces.init(Nodename="manual")
try:
    drone.fcu.arm()
    drone.fcu.disarm()
    drone.fcu.force_disarm()
    drone.fcu.reboot()           # ребут FC

    drone.fcu.calibrate_gyro()
    drone.fcu.calibrate_accel()
    # ...mag / baro / level / esc
finally:
    drone.close()
```

Полный пример — [../../../sverk_interfaces/examples/safety_and_calibration.py](../../../sverk_interfaces/examples/safety_and_calibration.py).

---

## 4. Что считать нормой (PX4 v1.15.4)

При проверке монитором ожидаемое поведение:

| Команда | Cmd | Норма | Примечание |
|---------|-----|-------|-----------|
| arm | 400 | `ACCEPTED`, armed=true | `TEMPORARILY_REJECTED` без батареи / при непройденном pre-arm — это норма PX4, не баг ноды |
| disarm | 400 | `ACCEPTED`, armed=false | только когда «на земле» |
| force_disarm | 400 | `ACCEPTED`, armed=false | работает даже «не на земле» |
| kill_switch | 185 | моторы стоп, `failsafe=true` | требует `CBRK_FLIGHTTERM=0`; ack может прийти `DENIED`, но termination применяется — особенность v1.15 |
| reboot | 246 | топики `/fmu/out/*` пропадают и возвращаются (~15 c) | FC ребутится только в дизарме; ack обычно не приходит — это нормально |
| калибровка | 241 | `ACCEPTED`/`IN_PROGRESS` + текст `[cal] calibration started: ... <sensor>` | в `TEXT` должен значиться **именно запрошенный** датчик |

> Калибровки accel/mag не завершаются одной командой — PX4 ждёт поворотов/
> ориентаций. Для проверки достаточно, что началась нужная процедура.

---

## 5. Сброс локальной позиции

Отдельной команды «обнулить локальную позицию» в PX4 нет. Рабочий способ —
**`reboot`** (`/fmu_control/reboot` / `drone.fcu.reboot()`): после ребута EKF2
переинициализируется и ставит origin в текущую точку. Делать только в дизарме.

> Важно: ребут сбрасывает накопленное смещение origin перед взлётом, но НЕ
> уменьшает дрейф оценки в самом полёте — это вопрос качества EKF/VPE, а не
> сброса позиции.
