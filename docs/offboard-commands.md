# Программирование автономных полётов

Здесь собраны все команды, которые вы можете отправить дрону из Python-программы. Одна библиотека — `sverk_interfaces` — берёт на себя всю сложную работу: запуск ROS 2 ноды, подключение к сервисам, пересчёт координат. Вам достаточно двух строк, чтобы поднять дрон в воздух.

---

## Быстрый старт

### Взлёт и посадка за 10 строк

**Что мы делаем:** дрон взлетает на 1.5 метра и через 5 секунд садится.

```python
import time                            # стандартная библиотека для паузы
import sverk_interfaces                # библиотека управления Обриком

# создаём объект drone — он запускает ROS 2 ноду внутри
drone = sverk_interfaces.init(Nodename="my_first_flight")

# navigate отправляет дрон в точку (x=0, y=0, z=1.5) — это 1.5 м вверх
# frame_id='body' означает «относительно корпуса», auto_arm=True — взлетайте сам
resp = drone.control.navigate(
    x=0.0, y=0.0, z=1.5,
    yaw=0.0, speed=1.0,
    frame_id="body",
    auto_arm=True
)
print("Взлёт:", resp.success, resp.message)  # True или False + сообщение

time.sleep(5.0)                        # ждём 5 секунд в воздухе

land_resp = drone.control.land()      # команда посадки
print("Посадка:", land_resp.success)

drone.close()                          # освобождаем ресурсы
```

Команда `navigate` отправляется мгновенно и программа продолжает работу. Дрон летит в фоне, пока вы делаете что-то ещё. Если хотите дождаться прилёта — используйте `navigate_wait`.

---

## Все команды drone.control

### navigate — полёт в точку

Базовая команда. Отправляет дрон к нужным координатам и сразу возвращает управление программе.

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `x`, `y`, `z` | float | обязательно | Координаты цели в метрах |
| `yaw` | float | 0.0 | Целевой курс в радианах |
| `speed` | float | 0.5 | Скорость полёта в м/с |
| `frame_id` | str | `'map'` | Система координат: `'body'`, `'map'`, `'aruco_map'` |
| `auto_arm` | bool | False | Если True, сам включит моторы и поднимется |

!!! Формулировка для auto_arm не совсем правильная без True моторы просто не запустятся и если до этого они не были включены дрон будет стоять на месте в дальнейшем нет необходмости выставлять True !!!

```python
# полёт на 1 метр вперёд относительно корпуса, сохраняя курс
drone.control.navigate(x=1.0, y=0.0, z=0.0, yaw=0.0, speed=0.5, frame_id="body")
```

---

### navigate_wait — полёт в точку с ожиданием прилёта

Как `navigate`, но программа ждёт, пока дрон не окажется в нужной точке с
заданной точностью. Реализован через action `/navigate_to`: прибытие
отслеживает сама нода `offboard_control` (без опроса телеметрии из Python).
Если дрон физически не долетел за `timeout` секунд — нода паркует его в hold,
а вызов бросает `TimeoutError`.

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `tolerance` | float | 0.2 | Допустимое расстояние до цели (м) |
| `timeout` | float | 60.0 | Максимальное время полёта (с); по истечении — hold + TimeoutError |
| `check_interval` | float | — | Устарел, игнорируется (прибытие отслеживает нода) |

```python
# взлёт и ожидание, пока дрон не окажется в 0.25 м от цели
drone.control.navigate_wait(
    x=0.0, y=0.0, z=1.5,
    frame_id="body",
    auto_arm=True,
    tolerance=0.25,
    timeout=30.0
)
print("Дрон прилетел!")
```

---

### land — посадка

Переводит дрон в режим автоматической посадки (PX4 AUTO.LAND).

```python
drone.control.land()
```

---

### get_telemetry — узнать, где дрон и что с ним

Возвращает текущее состояние дрона: координаты, скорость, заряд батареи, режим.

```python
t = drone.control.get_telemetry(frame_id="map")

print("Подключён:", t.connected)       # True/False
print("Заармирован:", t.armed)         # True = моторы включены
print("Режим:", t.mode)                # OFFBOARD, POSITION, LAND...
print("Позиция:", t.x, t.y, t.z)      # метры в системе map
print("Скорость:", t.vx, t.vy, t.vz)  # м/с
print("Батарея:", t.voltage, "В")      # напряжение
print("GPS:", t.lat, t.lon, t.alt)     # широта, долгота, высота
```

!!! Необходимо пояснение по входному frame_id, что отнисельно выбранного будут выводится данные и тд !!!

---

### set_altitude — сменить высоту, не меняя курс и положение по X, Y

Удобно, когда нужно просто подняться или спуститься во время полёта.

```python
# подняться до 2 м над землёй (terrain = относительно поверхности)
drone.control.set_altitude(z=2.0, frame_id="terrain")

# опуститься до 1 м в мировой системе координат
drone.control.set_altitude(z=1.0, frame_id="map")
```

---

### set_yaw — повернуться на угол

Поворот вокруг вертикальной оси без смены позиции.

```python
import math

# повернуться на 90 градусов по часовой (отрицательное = по часовой)
drone.control.set_yaw(yaw=-math.pi / 2, frame_id="body")

# сбросить ручное управление курсом
drone.control.clear_yaw_override()
```

---

### set_yaw_rate — вращаться с постоянной скоростью

Дрон вращается вокруг вертикальной оси до следующей команды.

```python
# вращение 0.5 рад/с против часовой стрелки (вид сверху)
drone.control.set_yaw_rate(yaw_rate=0.5)

# остановить вращение — сбросить override
drone.control.clear_yaw_override()
```

---

### set_position — задать точку для удержания (низкоуровневое)

Используется для частого обновления цели, например при полёте по кругу. Нода `offboard_control` сразу начинает лететь к этой точке.

```python
import math, time

# полёт по кругу: каждые 0.1 с обновляем цель
start = drone.control.get_telemetry(frame_id="map")
radius = 0.6
for i in range(200):
    angle = i * 0.1 * 0.3          # угол меняется 0.3 рад/с
    x = start.x + math.sin(angle) * radius
    y = start.y + math.cos(angle) * radius
    drone.control.set_position(x=x, y=y, z=start.z, yaw=start.yaw, frame_id="map")
    time.sleep(0.1)
```

---

### set_velocity — управление по скорости

Дрон летит с заданной скоростью по осям, пока не получит новую команду.

```python
# лететь вперёд 1 м/с по оси корпуса
drone.control.set_velocity(vx=1.0, vy=0.0, vz=0.0, yaw=0.0, frame_id="body")
```

!!! Описание входных vx vy и vz на всякий в чем минимальные максимальные и рекомендуемые значения !!!

---

### set_trajectory — единый setpoint (точка + скорость + ускорение)

Единый «траекторный» setpoint для упрощённого обхода препятствий: одним вызовом можно задать целевую **точку** И/ИЛИ вектор **скорости** И/ИЛИ **ускорения** (плюс `yaw`/`yaw_rate`). Смысл — держать целевую точку и одновременно подмешивать вектор скорости/ускорения для ухода от препятствия.

Правило групп: каждая тройка (позиция `x y z`, скорость `vx vy vz`, ускорение `ax ay az`) — либо **все** поля заданы, либо **все** не заданы (в Python — просто не передавай их, тогда группа выключена). Хотя бы одна группа должна быть задана.

```python
# держать точку и подмешать боковое ускорение (уход вбок)
drone.control.set_trajectory(x=2.0, y=0.0, z=1.5, ax=0.0, ay=1.0, az=0.0,
                             yaw=0.0, frame_id="map", auto_arm=True)
```

Для **динамического** обхода препятствий есть высокочастотный поток-коррекция `publish_trajectory_correction(...)`: базу (точку) задаёшь один раз через `set_trajectory`, а быстро меняющийся feed-forward (скорость/ускорение/yaw_rate) шлёшь в цикле по топику — это быстрее и не блокирует, как повторные вызовы сервиса:

```python
# базовая цель — один раз
drone.control.set_trajectory(x=0.0, y=0.0, z=1.5, yaw=0.0, auto_arm=True)

# коррекция — в цикле 50 Гц (например, из логики avoidance)
import math, time
t0 = time.time()
while time.time() - t0 < 4.0:
    ay = 1.5 * math.sin(math.pi * (time.time() - t0) / 4.0)
    drone.control.publish_trajectory_correction(ax=0.0, ay=ay, az=0.0,
                                                frame_id="base_link")  # в СК дрона
    time.sleep(0.02)
```

Если поток коррекций замолчит дольше `correction_timeout` (по умолчанию 0.5 с), нода сама сбросит коррекцию и удержит базовую точку.

---

### set_attitude — управление по углам (продвинутое)

Аналог режима STABILIZED: вы задаёте углы наклона и газ.

```python
import math
# небольшой наклон вперёд + газ 0.5
drone.control.set_attitude(roll=0.0, pitch=0.1, yaw=0.0, thrust=0.5, frame_id="map")
```

!!! Описание входных углов на всякий в чем минимальные максимальные и рекомендуемые значения !!!

---

### set_rates — управление угловыми скоростями (самое низкоуровневое)

Аналог режима ACRO. Задаёте скорости вращения по каждой оси и газ.

```python
# вращение по крену 1 рад/с, газ 0.4
drone.control.set_rates(roll_rate=1.0, pitch_rate=0.0, yaw_rate=0.0, thrust=0.4)
```

!!! Описание входных углов на всякий в чем минимальные максимальные и рекомендуемые значения !!!

Направления: `yaw_rate > 0` — против часовой (вид сверху), `pitch_rate > 0` — нос вверх, `roll_rate > 0` — влево.

---

### flip — автоматический кувырок

Дрон делает почти полный оборот по выбранной оси в три автоматических фазы:
1. Набор высоты с заданной скоростью `vz` в течение `climb_duration` секунд.
2. Вращение по оси с угловой скоростью `rate` до достижения `target_angle`.
3. Возврат к начальной позиции.

> **Внимание** Перед кувырком дрон должен находиться минимум на 2 метра над землёй.

```python
# кувырок вперёд (pitch) — дрон должен висеть на высоте >= 2 м
drone.control.flip(
    axis="pitch",          # ось: 'roll', 'pitch' или 'yaw'
    vz=2.0,                # скорость подъёма перед флипом (м/с)
    climb_duration=0.5,    # время подъёма (с)
    rate=16.0,             # скорость вращения (рад/с)
    target_angle=6.0,      # 6.0 рад ≈ почти полный оборот (2*pi = 6.28)
    thrust=0.1,            # тяга во время вращения
    auto_arm=False         # дрон уже в воздухе
)
```

---

## Новые команды безопасности и стриминга

Все новые утилиты ноды `offboard_control` доступны прямо из `drone.control`:

```python
import sverk_interfaces
drone = sverk_interfaces.init(Nodename="hardening_demo")

# аварийный стоп: зависнуть на месте (или land=True — сесть)
drone.control.emergency_stop()

# вернуться на точку взлёта и сесть (PX4 AUTO.RTL)
drone.control.return_to_launch()

# полёт с ожиданием прилёта и живым feedback; действие всегда завершается:
# если дрон не долетел за timeout — нода паркует его, success=False
def on_progress(fb):
    print(f"[{fb.phase}] осталось {fb.distance_remaining:.2f} м, ETA {fb.eta:.1f} с")

res = drone.control.navigate_to(x=1.0, y=0.0, z=1.5, speed=0.5,
                                frame_id="map", timeout=30.0, tolerance=0.2,
                                feedback=on_progress)
print(res.success, res.message, res.final_distance_error)

# поток телеметрии 10 Гц на 5 секунд (частоту держит нода)
drone.control.stream_telemetry(lambda t: print(t.z), rate_hz=10.0, duration=5.0)

# телеметрия постоянным топиком (подписка с правильным QoS — best-effort)
drone.control.set_telemetry_rate(50.0)
drone.control.subscribe_telemetry(lambda t: print(t.z))

# диагностика ноды: что летит, busy, причина последнего аварийного отката
s = drone.control.get_status()
print(s.setpoint_type, s.active_maneuver, s.last_abort_reason)
```

Те же команды доступны и из терминала (`ros2 service call /emergency_stop ...`,
`ros2 action send_goal /navigate_to ...`, `ros2 topic echo /offboard_control/status`) —
см. [ноду offboard_control](offboard-node.md).

---

## Команды drone.fcu — безопасность и калибровка

### Выключение моторов

```python
drone.fcu.disarm()         # штатное выключение (только на земле)
drone.fcu.force_disarm()   # принудительное (можно в воздухе, аварийно!)
drone.fcu.kill_switch()    # мгновенная остановка всех моторов
```

### Калибровки (всегда на земле, до полёта)

```python
drone.fcu.calibrate_gyro()        # гироскоп (дрон должен стоять неподвижно)
drone.fcu.calibrate_accel()       # акселерометр (потребует 6 положений)
drone.fcu.calibrate_level()       # горизонт (поставьте дрон ровно)
drone.fcu.calibrate_mag()         # компас (нужно вращать дрон вокруг всех осей)
drone.fcu.calibrate_baro()        # барометр (высотомер)
drone.fcu.calibrate_temperature() # температурная калибровка датчиков
```

---

## Команды drone.led — светодиодная лента

Доступна, если установлен пакет `led_interfaces` и запущена нода `led_control`.

!!! Это и так по стандарту лучше что то другое написать !!!

```python
if drone.led:
    # заливка всей ленты красным цветом
    drone.led.set_effect("fill", r=255, g=0, b=0)

    # мигание зелёным
    drone.led.set_effect("blink", r=0, g=255, b=0)

    # радуга по всей ленте
    drone.led.set_effect("rainbow")

    # управление отдельными светодиодами: (индекс, r, g, b)
    drone.led.set_leds([(0, 255, 0, 0), (1, 0, 255, 0), (2, 0, 0, 255)])
```

Доступные эффекты: `fill`, `blink`, `blink_fast`, `fade`, `wipe`, `flash`, `rainbow`, `rainbow_fill`.

---

## Настройка значений по умолчанию

Если все команды используют одинаковые параметры, задайте их один раз:

```python
drone.control.configure_defaults(
    frame_id="body",    # все команды без явного frame_id — в body
    speed=0.5,          # скорость по умолчанию
    auto_arm=False,     # не армировать автоматически
    tolerance=0.25,     # точность navigate_wait
    timeout=60.0        # максимальное ожидание
)

# теперь можно коротко
drone.control.navigate_wait(x=0.0, y=0.0, z=1.5)  # frame_id='body', speed=0.5
```

---

## Жизненный цикл программы

`sverk_interfaces.init(...)` сам запускает ROS 2, если он ещё не запущен. `drone.close()` корректно завершает работу. При любом выходе из программы (даже при ошибке) библиотека вызовет `close()` автоматически через `atexit`.

Рекомендуемая структура любой программы:

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="my_program")
try:
    # весь код полёта здесь
    pass
finally:
    drone.close()  # выполнится даже при ошибке
```

---

## Готовые примеры в репозитории

```bash
cd ~/sverk_ws && source install/setup.bash
```

> **Примечание** Команды запускаются **внутри контейнера** `sverk_ros2` — по SSH вы попадаете туда сразу: `ssh sverk@<IP_дрона>` (порт 22, пароль: sverk).

```bash
# взлёт на 1 м вперёд и посадка
python3 src/sverk_drone/sverk_interfaces/examples/simple_takeoff_land.py

# квадрат в плоскости map
python3 src/sverk_drone/sverk_interfaces/examples/square_mission.py

# объёмная фигура «куб» в системе body
python3 src/sverk_drone/sverk_interfaces/examples/cube_mission.py

# полёт по кругу через set_position
python3 src/sverk_drone/sverk_interfaces/examples/circle_trajectory.py

# монитор телеметрии в консоль
python3 src/sverk_drone/sverk_interfaces/examples/telemetry_monitor.py
```

!!! Это не все нужно свериться с sverk_ws\src\sverk_drone\sverk_interfaces !!!