"""
Что мы делаем: мигаем светодиодом на GPIO 17 через drone.gpio.pin_on/pin_off.

Подключение: светодиод + резистор 330 Ом → GPIO17 (пин 11) → GND.
"""
import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="gpio_blink")
try:
    print("Мигаем светодиодом на GPIO 17 (5 раз)...")
    for i in range(5):
        drone.gpio.pin_on(17)     # светодиод включается — HIGH 3.3 В
        time.sleep(0.3)
        drone.gpio.pin_off(17)    # светодиод выключается — LOW 0 В
        time.sleep(0.3)
        print(f"  Мигание {i + 1}/5")

    print("Готово! Освобождаем пин.")
finally:
    drone.gpio.pin_release(17)    # всегда освобождай пин после использования
    drone.close()
