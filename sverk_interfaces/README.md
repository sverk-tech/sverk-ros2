# sverk_interfaces — Python-библиотека управления дроном

Высокоуровневая Python-библиотека для управления дроном через ROS 2:

- **`offboard_control`** — полёт, позиции, скорости, флип, телеметрия.
- **`fmu_control`** — дизарм, force_disarm, kill switch, калибровки.
- **`image_control`** — захват кадров, стриминг, публикация изображений.
- **`gpio_control`** — GPIO-пины (вкл/выкл/читать), сервоприводы, электромагнит.
- **`topic_control`** — подписка на топики, публикация сообщений, одноразовое чтение, spin-циклы.
- **`led_control`** (опционально) — управление светодиодной лентой: эффекты и цвет отдельных светодиодов.

Работает поверх `rclpy` и уже запущенных нод PX4/Offboard/Calib/led_control.

Проверка:

```bash
python3 -c "import sverk_interfaces; print(sverk_interfaces)"
```

---

## 1. Быстрый старт

### 1.1. Простейший пример

```python
import time
import sverk_interfaces

# Создаём клиентский объект
drone = sverk_interfaces.init(Nodename="drone_control")

# Взлёт на 1.5 м относительно корпуса (frame_id='body'), с auto_arm
resp = drone.control.navigate(
    x=0.0,
    y=0.0,
    z=1.5,
    yaw=0.0,
    speed=1.0,
    frame_id="body",
    auto_arm=True,
)
print("navigate:", resp.success, resp.message)

time.sleep(5.0)

# Посадка
land_resp = drone.control.land()
print("land:", land_resp.success, land_resp.message)

# Явно освобождаем ресурсы (опционально, но хорошо)
drone.close()

```

### 1.2. Основные атрибуты

- **`drone.control`** — доступ к сервисам/действиям `offboard_control`:
  - `navigate`, `navigate_wait`, `navigate_to`, `land`, `get_telemetry`
    - `navigate_wait` реализован через action `/navigate_to`: прибытие
      отслеживает сама нода (без опроса телеметрии из Python); недостижимая
      цель завершается `TimeoutError`, дрон паркуется в hold. Параметр
      `check_interval` оставлен для совместимости и игнорируется.
    - `navigate_to(..., timeout=, tolerance=, feedback=)` — то же, но с
      живым feedback (`distance_remaining`/`eta`/`phase`) и результатом
      (`success`/`message`/`final_distance_error`) без исключений.
  - `emergency_stop(land=False)` — мгновенно остановить ЛЮБОЙ манёвр
    (hold на месте или посадка при `land=True`)
  - `return_to_launch()` — вернуться на точку взлёта и сесть (PX4 AUTO.RTL)
  - `stream_telemetry(callback, rate_hz=, duration=)` — поток телеметрии с
    частотой ноды (action `/receive_telemetry`)
  - `set_telemetry_rate(hz)` — включить топик `/offboard_control/telemetry`
    (0 = выключить); `subscribe_telemetry(callback)` — подписка на него с
    правильным QoS (best-effort)
  - `get_status()` — диагностика ноды из `/offboard_control/status`
    (тип сетпоинта, активный манёвр, причина последнего аварийного отката)
  - `set_altitude`, `set_yaw`, `clear_yaw_override`, `set_yaw_rate`
  - `set_position`, `set_velocity`
  - `set_attitude`, `set_rates`
  - `flip`
  - `run_excitation(..., confirm=...)` / `stop_excitation(emergency=)` —
    возбудитель для идентификации (PID-тюнинг); НЕ read-only, нужна
    кодовая фраза `I_UNDERSTAND_WHAT_I_AM_DOING`, сначала SITL
- **`drone.fcu`** — доступ к `fmu_control`:
  - `arm`, `disarm`, `force_disarm`, `kill_switch`, `reboot`
  - `get_param(name)`, `set_param(name, value)`, `reset_param(name="")` — параметры PX4 по имени через DDS (param_dds_bridge, без MAVLink)
  - `backup_params()` — надёжный полный дамп параметров (по индексу, с дозапросом); возвращает `.success/.total/.params`
  - `backup_to_file(path)` — тот же дамп, но сразу в файл формата QGC `.params` (грузится в QGC, восстанавливается через `set_param`)
  - `restore_from_file(path, confirm="I_UNDERSTAND_WHAT_I_AM_DOING")` — залить параметры из `.params` ( опасно, нужна кодовая фраза; ребут делать отдельно — `drone.fcu.reboot()`, обязателен для части параметров); возвращает `(ok, failed)`
  - `download_log(path, timeout_s=0.0, client_timeout=None)` — скачать последний `.ulg` с SD-карты по DDS (`log_download_bridge`, без MAVLink и физического доступа к плате); `timeout_s=0` — сервер сам считает дедлайн по размеру лога. Блокирует до результата (`.success/.message/.log_name/.file_size/.bytes_received`); измеренная скорость на живом линке ~27 КБ/с — многомегабайтный лог может качаться несколько минут.
  - Калибровки (только «без движения»): `calibrate_gyro`, `calibrate_level`
    - Остальные (`mag`, `baro`, `accel`, `esc`) в этом API намеренно НЕ выведены — им нужен интерактивный фидбэк (или, для ESC, соблюдение методики). Делай их напрямую через пакет **`fmu_control`**: action `/fmu_control/calibrate` (стримит прогресс `[cal]`) или топик `/fmu_control/request_calibration`. Подробности и примеры — в документации `fmu_control` ([offboard/fmu_control/README.md](../offboard/fmu_control/README.md), раздел «Калибровка через Action»).
- **`drone.image`** — захват, публикация и конвертация изображений (по умолчанию отдаёт OpenCV-кадры — numpy BGR):
  - `take_picture(topic, timeout, raw=False)` — получить один кадр (по умолчанию `/camera_1/image_raw`); с `raw=True` вернёт исходный `sensor_msgs/Image`
  - `stream(callback, topic, duration, raw=False)` — стриминг кадров N секунд или до `stop_stream()`; `callback(frame)` получает OpenCV-кадр (или ROS Image при `raw=True`)
  - `stop_stream()` — остановить текущий стрим
  - `publish(image, topic)` — опубликовать кадр (numpy **или** `sensor_msgs/Image`) в топик (по умолчанию `/out_detection`)
  - `to_cv2(image_msg)` — `sensor_msgs/Image` → numpy BGR (OpenCV)
  - `to_ros(img, header, encoding)` — numpy → `sensor_msgs/Image`
  - `detect_qr(image=None)` — найти и раскодировать QR/штрих-коды (через `pyzbar`); вернёт список `QRCode`
  - `calibration` — калибровка камеры (`CameraCalibrationAPI`): `start_session`, `set_config`, `capture`, `set_auto_capture`, `calibrate`, `save`, `get_state`… (или `None`, если пакет `camera_calibration` недоступен)
- **`drone.gpio`** — GPIO, сервоприводы, электромагнит:
  - `pin_on(pin)` / `pin_off(pin)` — включить/выключить пин (3.3 В / 0 В)
  - `pin_read(pin)` — прочитать состояние пина (0 или 1)
  - `pin_release(pin)` — освободить пин
  - `servo_enable()` / `servo_disable()` — включить/выключить PWM сервопривода
  - `servo_set_angle(degrees)` — повернуть сервопривод на заданный угол
  - `servo_center()` — вернуть сервопривод в центр (90°)
  - `magnet_on(pin=23)` / `magnet_off(pin=23)` — включить/выключить электромагнит
- **`drone.topic`** — работа с топиками ROS 2:
  - `create_publisher(msg_type, topic, qos)` — создать publisher (кэшируется)
  - `subscribe(msg_type, topic, callback, qos)` — подписаться на топик
  - `unsubscribe(sub)` — отписаться
  - `wait_for_message(msg_type, topic, timeout)` — дождаться одного сообщения
  - `spin()` — блокирующий цикл обработки (аналог `rclpy.spin`)
  - `spin_once(timeout_sec)` — обработать одну пачку событий
- **`drone.led`** (если установлен пакет `led_interfaces`) — доступ к ноде `led_control`:
  - `set_effect(effect, r, g, b)` — эффект и цвет ленты (fill, blink, rainbow и др.)
  - `set_leds(leds)` — цвет отдельных светодиодов по индексу
  - `get_state(timeout)` — текущее состояние ленты из топика `/led_control/state`

Все методы синхронные: ждут ответа сервиса и возвращают `Response` соответствующего ROS-сервиса.

---

## 2. Примеры использования

Помимо сниппетов ниже, в пакете есть готовые **примерные программы** в каталоге:

`src/sverk_drone/sverk_interfaces/examples`

Кратко:

- `simple_takeoff_land.py` — взлёт, пролёт 1 м вперёд по `body`, посадка.
- `square_mission.py` — квадрат в плоскости `map` с возвратом в стартовую точку.
- `cube_mission.py` — объёмная фигура «куб» в системе `body` (два квадрата на разных высотах).
- `circle_trajectory.py` — движение по окружности через `set_position`.
- `telemetry_monitor.py` — монитор телеметрии: стрим через
  `stream_telemetry` + классический опрос `get_telemetry`.
- `offboard_hardening_demo.py` — новые API одним полётом: `navigate_wait`
  (onboard-прибытие), `navigate_to` с feedback и таймаутом недостижимой цели,
  `stream_telemetry`, `get_status`, `emergency_stop`, `return_to_launch`.
- `safety_and_calibration.py` — арм/дизарм/kill/reboot и калибровки без движения (`gyro`, `level`) через `drone.fcu`.
- `params_backup.py` — бэкап/восстановление всех параметров в `.params` через `drone.fcu` (по DDS, без MAVLink).
- `log_download.py` — скачивание последнего `.ulg` с SD-карты через `drone.fcu.download_log` (по DDS, без MAVLink и физического доступа к плате).
- `led_effects.py` — эффекты светодиодной ленты (заливка, мигание, радуга и т.д.).
- `led_set_leds.py` — низкоуровневое управление отдельными светодиодами и чтение состояния ленты.
- `image_capture.py` — один кадр с камеры (numpy BGR) и сохранение в файл.
- `camera_stream.py` — стрим кадров и слежение за красным объектом в полёте.
- `image_publish.py` — обработка кадров и публикация в `/out_detection`.
- `qr_detect.py` — поиск QR-кодов через `drone.image.detect_qr` и публикация размеченного кадра.
- `camera_calibration_example.py` — калибровка камеры через `drone.image.calibration`.

```bash
cd ~/sverk_ws
source install/setup.bash
python3 src/sverk_drone/sverk_interfaces/examples/simple_takeoff_land.py
```

---

### 2.1. Телеметрия

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="drone_telemetry")

t = drone.control.get_telemetry(frame_id="map", timeout=1.0)
print(
    "connected:", t.connected,
    "armed:", t.armed,
    "mode:", t.mode,
    "pos:", (t.x, t.y, t.z),
    "yaw:", t.yaw,
)

drone.close()
```

### 2.2. Полёт по карте и посадка

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="simple_mission")

# Взлёт и полёт к точке в map
drone.control.navigate(
    x=1.0, y=0.0, z=1.0,
    yaw=0.0,
    speed=0.5,
    frame_id="map",
    auto_arm=True,
)

time.sleep(10.0)

drone.control.land()
drone.close()
```

### 2.3. Force disarm, калибровка и бэкап параметров

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="safety_tools")

# Принудительный дизарм (в т.ч. в воздухе)
drone.fcu.force_disarm()

# Калибровки без движения (в pre-arm)
drone.fcu.calibrate_gyro()      # держать неподвижно
drone.fcu.calibrate_level()     # поставить ровно
# mag/accel/esc — с фидбэком, через fmu_control (action /fmu_control/calibrate)

# Бэкап всех параметров в QGC .params (по чистому DDS, без MAVLink)
drone.fcu.backup_to_file("/home/sverk/backup.params")

# Восстановление ( опасно, нужна фраза; ребут делать отдельно)
ok, failed = drone.fcu.restore_from_file(
    "/home/sverk/backup.params", confirm="I_UNDERSTAND_WHAT_I_AM_DOING")
print("restored", ok, "failed", len(failed))
drone.fcu.reboot()              # обязателен для части параметров, в дизарме

# Скачать последний .ulg с SD-карты (по чистому DDS, без MAVLink)
result = drone.fcu.download_log("/home/sverk/last.ulg")
print(result.success, result.message)

drone.close()
```

### 2.4. Светодиодная лента (LED)

Нода `led_control` поднимается автозапуском (главный лаунч `full_system_real`), а пакет `led_interfaces` уже есть в образе — поэтому `drone.led` обычно доступен сразу. Если ноды или пакета нет, `drone.led` — `None`.

**Эффекты и цвет (сервис `/led_control/set_effect`):**

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="led_demo")
if drone.led:
    drone.led.set_effect("fill", r=255, g=0, b=0)      # заливка красным
    time.sleep(2.0)
    drone.led.set_effect("fade", r=0, g=255, b=0)       # плавный переход к зелёному
    time.sleep(2.0)
    drone.led.set_effect("rainbow")                      # радуга по ленте
else:
    print("LED API недоступен (нет led_interfaces или нода led_control не запущена)")
drone.close()
```

**Отдельные светодиоды (сервис `/led_control/set_leds`) и состояние ленты:**

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="led_demo")
if drone.led:
    # Список (index, r, g, b) или словари {"index", "r", "g", "b"}
    drone.led.set_leds([
        (0, 255, 0, 0),
        (1, 0, 255, 0),
        (2, 0, 0, 255),
    ])
    state = drone.led.get_state(timeout=2.0)
    if state:
        print("Светодиодов в ленте:", len(state.leds))
drone.close()
```

Доступные эффекты: `fill`, `blink`, `blink_fast`, `fade`, `wipe`, `flash`, `rainbow`, `rainbow_fill`. Константа `sverk_interfaces.LED_EFFECTS` содержит все имена.

### 2.5. Работа с изображениями (камера)

`drone.image` берёт на себя подписки и конвертацию. По умолчанию кадр приходит уже как **OpenCV-картинка** (numpy BGR) — `cv_bridge` не нужен. Если нужен исходный `sensor_msgs/Image`, передай `raw=True`.

**Сделать один снимок (OpenCV-кадр):**

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="photographer")
img = drone.image.take_picture(timeout=2.0)   # numpy BGR или None
if img is not None:
    print(f"Frame: {img.shape[1]}x{img.shape[0]}")   # ширина x высота
else:
    print("No frame — is camera running?")
drone.close()
```

**Стриминг 10 секунд — обработать каждый кадр:**

```python
import cv2
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="streamer")

def detect_red(frame):              # frame — numpy BGR
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    print(f"Red pixels: {cv2.countNonZero(mask)}")

drone.image.stream(detect_red, duration=10.0)
drone.close()
```

**Стриминг до ручной остановки (например, по нажатию Ctrl+C):**

```python
import cv2
import signal
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="infinite_streamer")

def draw_crosshair(frame):          # numpy BGR — можно рисовать прямо на кадре
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
    cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 0), 1)

signal.signal(signal.SIGINT, lambda *_: drone.image.stop_stream())
try:
    drone.image.stream(draw_crosshair)
finally:
    drone.close()
```

**Опубликовать обработанный кадр:**

```python
import cv2
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="detector")
img = drone.image.take_picture()       # numpy BGR
if img is not None:
    cv2.rectangle(img, (50, 50), (200, 200), (0, 255, 0), 2)  # рисуем bbox
    drone.image.publish(img)                      # numpy → /out_detection
    drone.image.publish(img, topic="/my/topic")    # или свой топик
drone.close()
```

**Нужен исходный кадр ROS (raw) и ручная конвертация:**

```python
drone = sverk_interfaces.init(Nodename="raw_cam")
msg = drone.image.take_picture(raw=True)   # sensor_msgs/Image
if msg is not None:
    print(f"{msg.width}x{msg.height}, {msg.encoding}")
    img = drone.image.to_cv2(msg)           # ROS Image → numpy BGR при необходимости
drone.close()
```

**Свой топик камеры (не `/camera_1/image_raw`):**

```python
drone = sverk_interfaces.init(
    Nodename="alt_cam",
    image_topic="/camera_2/image_raw",
    image_out_topic="/camera_2/detections",
)
img = drone.image.take_picture(timeout=3.0)
```

**Чтение QR-кодов (`pyzbar`):**

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="qr_reader")

# image=None — detect_qr сам возьмёт кадр с камеры
for code in drone.image.detect_qr():
    print(code.data, "в центре", code.center)   # data — строка, center — (x, y)

drone.close()
```

`detect_qr()` принимает numpy-кадр, `sensor_msgs/Image` или `None` (взять кадр сам) и
возвращает список `QRCode(data, type, rect, polygon, center)`. Нужен пакет `pyzbar`
(`pip install pyzbar`, системная `libzbar0`).

### 2.5.1. Калибровка камеры (`drone.image.calibration`)

Управление калибровкой через сервисы ноды `camera_calibration` — без ручного `ros2 service call`.

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="calibrator")
cal = drone.image.calibration
if cal is None:
    print("Нода camera_calibration недоступна")
else:
    cal.start_session()
    cal.set_config(board_type="charuco", board_width=11, board_height=8,
                   square_size=0.022, marker_size=0.016, aruco_dict="DICT_4X4_50")
    cal.set_auto_capture(True, period_sec=1.0)   # медленно води доской перед камерой
    time.sleep(20.0)
    cal.set_auto_capture(False)
    print("Состояние:", cal.get_state())          # число кадров, RMS error и т.д.
    cal.calibrate()                                # рассчитать параметры
    cal.save()                                     # сохранить latest_calibration.yaml
drone.close()
```

Превью с распознанной доской нода публикует в `/camera_calibration/processed_image` (смотри в веб-интерфейсе).

### 2.6. Работа с GPIO (пины, сервоприводы, электромагнит)

`drone.gpio` даёт управление GPIO-пинами, сервоприводами и электромагнитом без ручной работы с `gpiod` и без запуска отдельных ROS 2 нод. Для GPIO нужен пакет `python3-libgpiod` (`sudo apt install python3-libgpiod`).

**Мигаем светодиодом на GPIO17:**

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="blinker")

for _ in range(5):
    drone.gpio.pin_on(17)    # зажгли
    time.sleep(0.3)
    drone.gpio.pin_off(17)   # погасили
    time.sleep(0.3)

drone.close()
```

**Ждём нажатия кнопки на GPIO24:**

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="button_waiter")
print("Press the button...")
while drone.gpio.pin_read(24) == 0:
    time.sleep(0.05)
print("Button pressed!")
drone.close()
```

**Управляем сервоприводом** (`servo_control` не в автозапуске — сначала запусти `ros2 launch servo_control servo.launch.py`):

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_demo")

drone.gpio.servo_enable()
for angle in [0.0, 45.0, 90.0, 135.0, 180.0, 90.0]:
    drone.gpio.servo_set_angle(angle)
    time.sleep(0.5)
drone.gpio.servo_disable()

drone.close()
```

**Управляем электромагнитом на GPIO23 (нужен транзисторный ключ!):**

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="gripper")

drone.gpio.magnet_on(pin=23)   # захватили груз
time.sleep(3.0)
drone.gpio.magnet_off(pin=23)  # отпустили груз

drone.close()
```

**Свой GPIO чип (не `/dev/gpiochip0`):**

```python
drone = sverk_interfaces.init(gpio_chip="/dev/gpiochip1")
```

### 2.7. Работа с топиками (подписка, публикация, spin)

`drone.topic` даёт прямой доступ к топикам ROS 2 без ручного `rclpy.init()` и `create_node()`.

**Подписка — слушаем батарею:**

```python
import sverk_interfaces
from sensor_msgs.msg import BatteryState

drone = sverk_interfaces.init(Nodename="bat_monitor")

def battery_cb(msg):
    pct = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
    print(f"Voltage: {msg.voltage:.2f} V, {pct:.0f}%")

sub = drone.topic.subscribe(BatteryState, "/fmu/out/battery_status", battery_cb)
drone.topic.spin()  # блокируется, вызывая battery_cb на каждом сообщении
```

**Публикация — отправляем свои данные:**

```python
import sverk_interfaces
from std_msgs.msg import String

drone = sverk_interfaces.init(Nodename="talker")
pub = drone.topic.create_publisher(String, "/chatter")

for i in range(5):
    msg = String()
    msg.data = f"Hello {i}"
    pub.publish(msg)
    drone.topic.spin_once(timeout_sec=0.5)

drone.close()
```

**Одноразовое чтение — ждём один кадр камеры:**

```python
import sverk_interfaces
from sensor_msgs.msg import Image

drone = sverk_interfaces.init(Nodename="frame_grabber")
frame = drone.topic.wait_for_message(Image, "/camera_1/image_raw", timeout=2.0)
if frame is not None:
    print(f"Got frame: {frame.width}x{frame.height}")
else:
    print("No frame received within timeout")
drone.close()
```

---

## 3. Управление жизненным циклом

- `sverk_interfaces.init(...)`:
  - если `rclpy` ещё не инициализирован, вызывает `rclpy.init()`;
  - создаёт `rclpy.Node` с указанным именем (`Nodename` / `node_name`);
  - возвращает объект `DroneInterfaces`.
- `drone.close()`:
  - уничтожает созданный нод;
  - при отсутствии других инстансов библиотеки инициализированный библиотекой `rclpy` будет корректно остановлен (`rclpy.shutdown()`).
- При завершении процесса:
  - срабатывает `atexit`‑обработчик, который вызывает `close()` для всех активных инстансов.

Обычному пользователю достаточно вызывать `drone.close()` в конце программы, остальное библиотека сделает сама.

---

## 4. Настройка значений по умолчанию (`configure_defaults`)

Для упрощения использования библиотека позволяет задать **общие дефолты** для всех
основных методов `drone.control` (navigate, navigate_wait, set_position, set_velocity,
set_attitude, set_rates, flip, land, get_telemetry и т.д.).

Пример:

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="drone_control")

# Один раз настраиваем "режим по умолчанию":
drone.control.configure_defaults(
    frame_id="body",   # все команды без явного frame_id будут относиться к body
    yaw=0.0,           # yaw по умолчанию
    speed=0.5,         # скорость по умолчанию для navigate/navigate_wait
    auto_arm=False,    # auto_arm по умолчанию (например, выключен)
    timeout=60.0,      # общий таймаут по умолчанию
    tolerance=0.25,    # допуск по расстоянию для navigate_wait
    check_interval=0.2 # период опроса телеметрии в navigate_wait
)

# Теперь можно опускать большинство аргументов:
drone.control.navigate_wait(x=0.0, y=0.0, z=1.5)   # frame_id="body", speed=0.5, yaw=0.0
drone.control.set_position(z=1.0)                  # полёт на 1 м вверх в body с yaw=0.0
drone.control.set_velocity(vx=1.0, vy=0.0, vz=0.0) # вперёд по body с yaw=0.0
```

Внутри `OffboardControlAPI` есть поля с текущими дефолтами:

- `drone.control.default_frame_id`
- `drone.control.default_yaw`
- `drone.control.default_speed`
- `drone.control.default_auto_arm`
- `drone.control.default_timeout`
- `drone.control.default_tolerance`
- `drone.control.default_check_interval`

Их можно менять как напрямую, так и через `configure_defaults(...)`.

**Важно:** если пользователь не передал параметр (`frame_id`, `yaw`, `speed`,
`auto_arm`, `timeout` и т.п.), он **не останется неопределённым** — всегда
подставится одно из этих дефолтных значений. Это позволяет задать безопасный
«профиль» поведения дрона по умолчанию и не бояться неполных вызовов
со стороны начинающих пользователей.

---

## 5. DEV‑секция (для разработчиков)

Этот раздел для тех, кто расширяет библиотеку или интегрирует дополнительные ROS‑сервисы.

### 5.1. Добавление нового семейства сервисов

Внутри `DroneInterfaces` есть метод `add_family`, который использует класс `GenericServiceFamily`:

```python
from my_pkg.srv import DoThing
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="dev_client")

my_family = drone.add_family(
    name="my",                 # drone.my
    namespace="/my_node_ns",   # префикс для сервисов
    services={
        "do_thing": DoThing,   # имя сервиса -> тип srv
    },
)

req = DoThing.Request()
req.param = 123
resp = my_family.call("do_thing", req, timeout=1.0)
print(resp)

drone.close()
```

Внутри `GenericServiceFamily`:

- создаются клиенты `rclpy` для каждого сервиса (`node.create_client`),
- метод `call(name, request, timeout)` оборачивает общий `_call_service(...)`.

Это удобный шаблон для быстрого подключения новых ROS‑сервисов без написания отдельного API‑класса.

### 5.3. Расширение `OffboardControlAPI` / `FmuCalibrationAPI` / `ImageAPI` / `GpioAPI` / `LedAPI` / `TopicAPI`

Для добавления нового метода:

1. Убедитесь, что соответствующий сервис уже объявлен и запущен в ноде (`offboard_control` или `fmu_control`).
1. В `OffboardControlAPI.__init__` или `FmuCalibrationAPI.__init__`:
   - добавьте `self._имя = node.create_client(SrvType, _service_name(..., "имя"))`.
2. Реализуйте метод‑обёртку:
   - создайте `request = SrvType.Request()`,
   - заполните поля,
   - верните `_call_service(self._node, self._имя, request, timeout)`.

### 5.2. Неймспейсы

Параметры `init`:

```python
drone = sverk_interfaces.init(
    Nodename="drone_control",
    offboard_namespace="",                      # по умолчанию сервисы /navigate, /land, ...
    fcu_namespace="/fmu_control",    # по умолчанию
    led_namespace="led_control",                        # по умолчанию /led_control/set_effect, /led_control/set_leds, /led_control/state
    topic_namespace="",                         # префикс для drone.topic (пустой = абсолютные имена)
    image_topic="/camera_1/image_raw",          # топик входа для take_picture / stream
    image_out_topic="/out_detection",           # топик выхода для publish
    gpio_chip="/dev/gpiochip0",                # путь к GPIO-чипу
    servo_enable="/servo_control/enable",          # сервис включения сервопривода
    servo_angle_topic="/servo_control/target_angle_deg",   # топик угла сервопривода
    servo_center="/servo_control/center",          # сервис центрирования сервопривода
)
```

Можно переназначить, если offboard_control, fmu_control, нода LED, топики, камера или сервопривод запущены в другом namespace.

### 5.4. Поведение rclpy

- `_ensure_rclpy_init()`:
  - вызывает `rclpy.init()` только если он ещё не был вызван;
  - помечает, что `rclpy` принадлежит библиотеке.
- `_maybe_rclpy_shutdown()`:
  - вызывает `rclpy.shutdown()` только когда:
    - нет живых `DroneInterfaces`,
    - инициализация `rclpy` была сделана библиотекой.

Если приложение само управляет жизненным циклом `rclpy` (например, создаёт свои ноды и свой executor), библиотека не будет мешать, пока не будет создана/закрыта последняя её инстанция.

### 5.5. Пересборка

Типичный цикл:

```bash
cd ~/sverk_ws
colcon build --packages-select sverk_interfaces
source install/setup.bash
```
