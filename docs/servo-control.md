# Управление сервоприводами

Сервоприводы на Обрике управляются через API `drone.gpio` из библиотеки `sverk_interfaces`. Для одного или двух сервоприводов — один и тот же способ, только с разными каналами.

> **Примечание** Сначала подключите сервопривод: [Подключение сервоприводов](servo-hardware.md)

---

## Пакет servo_control

За управление PWM отвечает ROS 2 пакет `servo_control`, нода `servo_node` (библиотека `rpi_hardware_pwm`).

> **Внимание** В отличие от ядра (камера, `offboard_control`, лента), `servo_node` **не входит в [главный launch-файл](main-launch.md)** и сама не запускается. Перед работой с сервоприводом запустите ноду (см. ниже) — или добавьте её в главный launch-файл, если хотите, чтобы серво поднималось автоматически.

Конфигурационный файл ноды:

```bash
servo_control/config/servo_params.yaml
```

---

## Запуск ноды

Запустите `servo_node` внутри контейнера `sverk_ros2`:

```bash
# Запуск через launch-файл (рекомендуется)
ros2 launch servo_control servo.launch.py

# Запуск напрямую
ros2 run servo_control servo_node

# Запуск с пользовательскым конфигом
ros2 run servo_control servo_node --ros-args --params-file /path/to/servo_params.yaml

# Запуск сервопривода на GPIO13 (второй канал)
ros2 run servo_control servo_node --ros-args -p pwm_chip:=0 -p pwm_channel:=1
```

---

## Управление из Python — основные операции

**Что мы делаем:** включаем сервопривод, поворачиваем его в разные положения, возвращаем в центр, выключаем.

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_example")

# Включаем сервопривод — подаём питание и начинаем удерживать позицию
drone.gpio.servo_enable()
time.sleep(0.3)                        # небольшая пауза для инициализации

# Поворачиваем в разные положения (от 0 до 180 градусов)
drone.gpio.servo_set_angle(0)          # крайнее левое положение
time.sleep(1.0)

drone.gpio.servo_set_angle(90)         # центральное положение
time.sleep(1.0)

drone.gpio.servo_set_angle(180)        # крайнее правое положение
time.sleep(1.0)

# Возвращаем в центр — удобный способ без указания угла
drone.gpio.servo_center()
time.sleep(0.5)

# Выключаем сервопривод — PWM-сигнал прекращается, нагрузка на мотор снимается
drone.gpio.servo_disable()

drone.close()
```

---

## Управление из Python — сканирование углов

**Что мы делаем:** пишем программу, которая плавно поворачивает сервопривод через все углы.

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_example")

drone.gpio.servo_enable()
time.sleep(0.3)

try:
    # Плавный проход от 0 до 180 градусов шагами по 10°
    for angle in range(0, 181, 10):
        drone.gpio.servo_set_angle(angle)
        print(f"Угол: {angle}°")
        time.sleep(0.3)                 # даём время механике дойти до позиции

    # Плавный проход обратно
    for angle in range(180, -1, -10):
        drone.gpio.servo_set_angle(angle)
        print(f"Угол: {angle}°")
        time.sleep(0.3)

finally:
    drone.gpio.servo_center()           # возвращаем в центр перед выключением
    drone.gpio.servo_disable()
    drone.close()
```

---

## Управление из Python — сервопривод как захват

**Что мы делаем:** управляем захватом на основе сервопривода — открываем и закрываем.

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_example")

# Константы для захвата — подберите под свой сервопривод
OPEN_ANGLE  = 0    # открыто
CLOSE_ANGLE = 90   # захвачено

def gripper_open():
    """Открыть захват."""
    drone.gpio.servo_set_angle(OPEN_ANGLE)
    print("Захват открыт")

def gripper_close():
    """Закрыть захват (взять предмет)."""
    drone.gpio.servo_set_angle(CLOSE_ANGLE)
    print("Захват закрыт")

drone.gpio.servo_enable()
time.sleep(0.3)

try:
    gripper_open()
    time.sleep(1.0)

    gripper_close()   # взяли предмет
    time.sleep(2.0)

    gripper_open()    # отпустили предмет
    time.sleep(1.0)

finally:
    drone.gpio.servo_disable()
    drone.close()
```

---

## Проверка состояния

```bash
# Посмотреть текущий угол (если нода запущена)
ros2 topic echo /servo_node/current_angle_deg

# Проверить, включён ли сервопривод
ros2 topic echo /servo_node/enabled

# Убедиться, что нода живёт
ros2 topic hz /servo_node/current_angle_deg
```

---

## Топики и сервисы ноды (справочно)

| Топик / Сервис | Направление | Тип | Описание |
|----------------|-------------|-----|----------|
| `~/target_angle_deg` | входящий | `Float32` | Задать угол в градусах |
| `~/target_pulse_width_us` | входящий | `UInt16` | Задать ширину импульса напрямую (мкс) |
| `~/current_angle_deg` | исходящий | `Float32` | Текущий угол |
| `~/enabled` | исходящий | `Bool` | Включён ли сервопривод |
| `~/enable` | сервис | `SetBool` | Включить/выключить PWM |
| `~/center` | сервис | `Trigger` | Вернуть в центр |

---

## Два сервопривода одновременно

Для управления двумя сервоприводами запускаем две ноды с разными именами и каналами:

```bash
# Первый сервопривод — GPIO12 (канал 0)
ros2 run servo_control servo_node --ros-args -r __node:=servo_node_1 -p pwm_chip:=0 -p pwm_channel:=0

# Второй сервопривод — GPIO13 (канал 1)
ros2 run servo_control servo_node --ros-args -r __node:=servo_node_2 -p pwm_chip:=0 -p pwm_channel:=1
```

**Управление двумя сервоприводами из Python:**

```python
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_example")

# первый сервопривод — канал 0 (активен по умолчанию)
drone.gpio.servo_select_channel(0)
drone.gpio.servo_enable()
time.sleep(0.3)
drone.gpio.servo_set_angle(45)    # 45°
time.sleep(1.0)

# второй сервопривод — канал 1 (отдельный servo_node, включаем заново)
drone.gpio.servo_select_channel(1)
drone.gpio.servo_enable()
time.sleep(0.3)
drone.gpio.servo_set_angle(90)    # 90°
time.sleep(1.0)

drone.gpio.servo_disable()
drone.close()
```

---

## Практические советы

- Если сервопривод дёргается или не доходит до крайних позиций — подберите диапазон `min_pulse_width_us` и `max_pulse_width_us` в конфигурационном файле `servo_params.yaml`.
- Убедитесь, что питание сервопривода достаточное — при нагрузке слабое питание может вызывать сбои.
- После `servo_disable()` сервопривод перестаёт удерживать позицию — вал можно будет провернуть руками.

---

> **Подсказка** Подробнее о реализации серво и магнита → [servo-magnet-raw.md](knowledge_base/servo-magnet-raw.md).
