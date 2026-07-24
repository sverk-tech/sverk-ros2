"""
Что мы делаем: качаем сервопривод от 0° до 180° и обратно через drone.gpio.servo_*.

Подключение сервопривода:
  - Жёлтый провод (сигнал) → GPIO12 (пин 32, PWM0)
  - Красный провод (питание) → 5В (внешний источник или плата расширения)
  - Коричневый провод (земля) → GND (общая с Обриком)

Перед запуском убедись, что нода servo_control запущена:
  ros2 launch servo_control servo.launch.py
"""
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="servo_sweep")
try:
    print("Включаем сервопривод...")
    drone.gpio.servo_enable()       # разрешаем PWM-выход servo_control

    time.sleep(0.5)                 # небольшая пауза после включения

    print("Движение 0° → 180°")
    for angle in range(0, 181, 10):  # шаг 10°
        drone.gpio.servo_set_angle(float(angle))
        time.sleep(0.1)              # ждём, чтобы серва успела повернуться

    print("Движение 180° → 0°")
    for angle in range(180, -1, -10):
        drone.gpio.servo_set_angle(float(angle))
        time.sleep(0.1)

    print("Возврат в центр (90°)")
    drone.gpio.servo_center()        # вернуть в 90°

    time.sleep(0.5)

    print("Выключаем сервопривод.")
    drone.gpio.servo_disable()       # выключаем PWM — серва "обмякнет"
finally:
    drone.close()
