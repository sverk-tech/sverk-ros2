"""
Что мы делаем: ждём нажатия кнопки на GPIO 24, затем мигаем светодиодом на GPIO 17.

Подключение:
  - Кнопка: GPIO24 (пин 18) → GND (при нажатии — LOW, поэтому ждём LOW).
    Включи внутренний pull-up: кнопка нажата = LOW.
  - Светодиод: GPIO17 (пин 11) → резистор 330 Ом → GND.
"""
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="gpio_button")
try:
    print("Нажми кнопку на GPIO 24...")

    # ждём нажатия: pin_read вернёт 0 (LOW), когда кнопка нажата
    while drone.gpio.pin_read(24) == 1:  # 1 = не нажата (pull-up HIGH)
        time.sleep(0.05)                 # проверяем каждые 50 мс

    print("Кнопка нажата! Мигаем светодиодом 3 раза.")

    for _ in range(3):
        drone.gpio.pin_on(17)    # LED on
        time.sleep(0.2)
        drone.gpio.pin_off(17)   # LED off
        time.sleep(0.2)

    print("Готово!")
finally:
    drone.gpio.pin_release(24)   # освобождаем кнопку
    drone.gpio.pin_release(17)   # освобождаем светодиод
    drone.close()
