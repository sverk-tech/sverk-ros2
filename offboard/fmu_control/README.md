# fmu_control

Узел ROS 2 для **калибровки полётника**, **арм/дизарма**, **перезагрузки**,
**параметров** и **скачивания логов** — всё по uXRCE-DDS
([dds_topics](https://docs.px4.io/main/en/middleware/dds_topics)), без
MAVLink-кабеля.

## Зависимости

- `px4_msgs` (из `sverk_drone/px4/px4_msgs`)
- PX4 с uXRCE-DDS agent и подпиской на `/fmu/in/vehicle_command`

## Сборка

Из `sverk_ws`:

```bash
colcon build --packages-select fmu_control
source install/setup.bash
```

## Запуск

```bash
ros2 launch fmu_control calibration_control.launch.py
```

Или только узел:

```bash
ros2 run fmu_control calibration_control
```

## API

### Арм, дизарм и аварийное отключение

- **`/fmu_control/arm`** — арм моторов (`ARM_DISARM`, param1=1). PX4 выполняет свои pre-arm проверки.
- **`/fmu_control/disarm`** — обычный дизарм. PX4 примет только если дрон «на земле» (иначе: *Disarming denied: not landed*).
- **`/fmu_control/force_disarm`** — принудительный дизарм (param2=21196), обход проверки «not landed». Использовать, когда нужно снять арм в воздухе без остановки моторов по логике flight termination.
- **`/fmu_control/kill_switch`** — **Kill Switch**: `VEHICLE_CMD_DO_FLIGHTTERMINATION` — немедленная остановка моторов (в т.ч. в воздухе). Эквивалент аварийной кнопки.
  - **Требует `CBRK_FLIGHTTERM = 0`** на полётнике. По умолчанию `CBRK_FLIGHTTERM = 121212` (терморазрыв включён) — тогда PX4 проигнорирует команду и моторы не остановятся.
- **`/fmu_control/reboot`** — перезагрузка полётника (`VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN`, param1=1). PX4 перезагрузится только в дизарме. Удобно после калибровки.

```bash
# Арм
ros2 service call /fmu_control/arm std_srvs/srv/Trigger

# Обычный дизарм (только когда приземлился)
ros2 service call /fmu_control/disarm std_srvs/srv/Trigger

# Принудительный дизарм (игнорирует "not landed")
ros2 service call /fmu_control/force_disarm std_srvs/srv/Trigger

# Kill Switch — немедленная остановка моторов (нужен CBRK_FLIGHTTERM=0)
ros2 service call /fmu_control/kill_switch std_srvs/srv/Trigger

# Перезагрузка полётника (только в дизарме)
ros2 service call /fmu_control/reboot std_srvs/srv/Trigger
```

### Калибровка

Топик `/fmu_control/request_calibration` (тип `std_msgs/msg/UInt8`). Значение = тип калибровки. В `MAV_CMD_PREFLIGHT_CALIBRATION` каждый датчик — это **отдельный param-слот** (а не индекс в param1); узел сам подставляет нужный param:

| Значение | Тип   | Описание             | PX4 param      |
|----------|-------|-----------------------|----------------|
| 0        | gyro  | Гироскоп              | param1 = 1     |
| 1        | mag   | Магнитометр (компас)  | param2 = 1     |
| 2        | baro  | Барометр (ground pressure) | param3 = 1 |
| 3        | accel | Акселерометр          | param5 = 1     |
| 4        | level | Уровень горизонта     | param5 = 2     |
| 5        | esc   | Калибровка ESC | param7 = 1     |

> Термокалибровка (temperature) не запускается этой командой в PX4 — она включается параметрами `SYS_CAL_GYRO`/`SYS_CAL_ACCEL`/`SYS_CAL_BARO = 1` с последующей перезагрузкой (`/fmu_control/reboot`).

#### Нужна ли обратная связь

Команда — это только **запуск** (fire-and-forget). PX4 сам решает, как вести калибровку, и шлёт подсказки/прогресс строками `[cal] ...`. По степени интерактивности:

| Калибровка | Взаимодействие | Довести без фидбэка? |
|-----------|----------------|----------------------|
| gyro | держать неподвижно | да |
| baro | мгновенно | да |
| level | поставить ровно, держать | да |
| mag | **вращать** дрон по осям | нужен фидбэк `[cal]` |
| accel | **ставить на 6 сторон** поочерёдно | нужен фидбэк `[cal]` |
| esc | **опасно**: пропеллеры сняты, точная последовательность подачи питания | только осознанно, по методике PX4 |

**ESC** намеренно НЕ выведен в высокоуровневый `sverk_interfaces` — только этот низкоуровневый топик (`data: 5`), чтобы вызывать осознанно.

#### Где брать прогресс калибровки

Строки `[cal] Hold still, measuring <side>`, `[cal] <side> side done, rotate to a different side`, `[cal] progress <N>`, `[cal] calibration done/failed` PX4 публикует в uORB-топик **`log_message`** (тот же источник, что и MAVLink STATUSTEXT в QGC). Чтобы получить их **по DDS** (без MAVLink), достаточно пробросить топик в `dds_topics.yaml` — это конфиг-правка, укладывается в оверлей `fmu/update_for_params`:

```yaml
publications:
  - topic: /fmu/out/log_message
    type: px4_msgs::msg::LogMessage
```

Дальше на компаньоне подписываемся на `/fmu/out/log_message`, фильтруем строки на `[cal]` и парсим (какой датчик, сторона `down/up/left/right/front/back`, `progress <N>`, done/failed) — и отдаём это как feedback ROS 2 Action или как топик прогресса.

Примеры:

```bash
# Калибровка гироскопа
ros2 topic pub --once /fmu_control/request_calibration std_msgs/msg/UInt8 "{data: 0}"

# Калибровка магнитометра
ros2 topic pub --once /fmu_control/request_calibration std_msgs/msg/UInt8 "{data: 1}"

# Калибровка акселерометра
ros2 topic pub --once /fmu_control/request_calibration std_msgs/msg/UInt8 "{data: 3}"

# Быстрый дизарм / Kill Switch при "not landed"
ros2 service call /fmu_control/force_disarm std_srvs/srv/Trigger
ros2 service call /fmu_control/kill_switch std_srvs/srv/Trigger
```

**Важно:** калибровка принимается PX4 только в pre-arm состоянии (полётник не в полёте). Дизарм можно вызывать в любой момент для экстренного снятия арма.

## Калибровка через Action (с прогрессом)

`/fmu_control/calibrate` (`offboard_interfaces/action/Calibrate`) — калибровка с
**живой обратной связью**: сервер триггерит `PREFLIGHT_CALIBRATION`, парсит `[cal]`
из `/fmu/out/log_message` и стримит feedback (`progress`, `instruction`,
`orientation`, `step/total_steps`). Result — `success`/`message`. Отмена
(`cancel`) шлёт калибровку со всеми param=0 (PX4 её прерывает).

`sensor`: `0=gyro 1=mag 2=baro 3=accel 4=level 5=esc`.

 **ESC**: goal принимается только с кодовой фразой
`confirm: "I_UNDERSTAND_WHAT_I_AM_DOING"` (иначе goal отклоняется). Пропеллеры сняты,
строго по методике PX4.

```bash
# accel — крути дрон по подсказкам feedback (orientation)
ros2 action send_goal --feedback /fmu_control/calibrate \
  offboard_interfaces/action/Calibrate '{sensor: 3, confirm: ""}'

# esc — только с фразой
ros2 action send_goal --feedback /fmu_control/calibrate \
  offboard_interfaces/action/Calibrate \
  '{sensor: 5, confirm: "I_UNDERSTAND_WHAT_I_AM_DOING"}'
```

Низкоуровневый триггер без фидбэка (`/fmu_control/request_calibration`, UInt8)
остаётся для простых кейсов.

## Бэкап параметров

`/fmu_control/backup_params` (`offboard_interfaces/srv/BackupParams`) — надёжное
чтение **всех** параметров по индексу (`GET_INDEX`) с дозапросом пропусков.
Возвращает `total`, `Param[] params` и `success` (true только если получены все).
Долго, но гарантированно полно (≈16 c / 1567 параметров).

```bash
ros2 service call /fmu_control/backup_params offboard_interfaces/srv/BackupParams '{}'
```

## Скачивание логов (ulog)

`/fmu_control/download_log` (`offboard_interfaces/action/DownloadLog`) — скачивает
**последний** `.ulg` с SD-карты полностью по uXRCE-DDS, без MAVLink и без
физического доступа к плате. Требует прошивку с модулем `log_download_bridge`
(`fmu/log_download/`, топики `/fmu/in/log_download_request` +
`/fmu/out/log_download_response`).

Внутри — надёжный ack-driven windowed pull (не более `LOG_DL_WINDOW`
запросов "в полёте" одновременно, зависший кусок ретраится по таймауту):
FMU сначала спрашивается `OP_INFO` (имя + размер последнего лога), затем
кусками по 240 байт читается весь файл на диск по `path`. Result —
`success`/`message`/`log_name`/`file_size`/`bytes_received`; Feedback —
`progress`/`chunks_received`/`chunks_total` каждые ~0.5с.

`timeout_s` (goal) — общий дедлайн скачивания: `0` (по умолчанию) —
автовычисление из размера лога (консервативная оценка по замеренному
throughput, потолок 1800с); можно передать явно для заведомо больших логов.

Скорость упирается не в конкуренцию окна, а в реальную пропускную
способность UART/DDS-стека: свип по `window` (16/20/24/32/48, см.
`test/log_download_receiver.py`) показал плато ~26.8 КБ/с уже с 16, без
коллапса на всех значениях — увеличение окна выше 16 не ускоряет
скачивание, только немного грузит CPU агента.

 Тем не менее дефолт **`LOG_DL_WINDOW=12`** (ниже плато) — после инцидента,
когда на `window=16` при повторных скачиваниях подряд с одновременно активным
полным стеком (камера/aruco) весь входящий DDS-канал у FMU намертво отвалился
(`uxrce_dds_client`: `rx=0 B/s`, `tx` жив), потребовался ребут. Похожая
деградация уже фиксировалась в soak-тесте (повторные скачивания без паузы), так
что дело, вероятно, не только в `window` — но понижение окна на 4 (throughput
~20 КБ/с вместо ~26.8) — дешёвая подстраховка, пока настоящий
watchdog-восстановления моста не сделан на стороне прошивки.

```bash
ros2 action send_goal --feedback /fmu_control/download_log \
  offboard_interfaces/action/DownloadLog '{path: "/home/sverk/last.ulg", timeout_s: 0.0}'
```

 Известное ограничение: если бридж на стороне FMU перегружен (например,
слишком агрессивным флудом запросов извне), он может перестать отвечать до
перезагрузки полётника — сам `download_log` при этом просто уйдёт в таймаут
(`success: false`, `bytes_received` < `file_size`). Из компаньона это не
лечится, нужен ребут FMU.

## Motor test (пропеллеры сняты!)

- `/fmu_control/motor_test` (`offboard_interfaces/srv/MotorTest`): `motor_id`
  (1-based), `throttle` 0..1, `timeout_s` → `VEHICLE_CMD_DO_MOTOR_TEST`.
  Отклоняется, если аппарат **в арме**.
- `/fmu_control/stop_all_motors` (`Trigger`): нулевой throttle на моторы 1..8.

```bash
ros2 service call /fmu_control/motor_test offboard_interfaces/srv/MotorTest \
  '{motor_id: 1, throttle: 0.1, timeout_s: 1.0}'
ros2 service call /fmu_control/stop_all_motors std_srvs/srv/Trigger
```

## Проверка и ручное управление

Инструмент для ручной работы и проверки команд на живом PX4 (монитор `mav_monitor.py` + инструкция) — см. [test/README.md](test/README.md).

## Связь с DDS-топиками PX4

По [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics):

- **Subscription:** `/fmu/in/vehicle_command` — тип `px4_msgs/msg/VehicleCommand`.
- Команды: `VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241`, `VEHICLE_CMD_COMPONENT_ARM_DISARM = 400`, `VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246`, `VEHICLE_CMD_DO_FLIGHTTERMINATION = 185`.

Ответы и подтверждения можно смотреть в `/fmu/out/vehicle_command_ack` и `/fmu/out/vehicle_status`.
