"""
Что мы делаем: дрон взлетает, включает электромагнит для захвата груза,
летит к точке доставки, выключает электромагнит, садится.

Подключение электромагнита:
  - GPIO 23 (пин 16) → транзисторный ключ (база)
  - Коллектор транзистора → электромагнит → 5–12В
  - Эмиттер → GND (общая с Обриком)
  - Защитный диод: параллельно катушке электромагнита

ВАЖНО: дрон должен стоять на точке захвата груза перед запуском!
       Груз должен находиться прямо под электромагнитом.
"""
import time
import sverk_interfaces

MAGNET_PIN = 23    # GPIO-пин управления транзисторным ключом
FLIGHT_HEIGHT = 0.5  # высота полёта в метрах

drone = sverk_interfaces.init(Nodename="magnet_grab")
try:
    print("Шаг 1: включаем электромагнит — захватываем груз")
    drone.gpio.magnet_on(MAGNET_PIN)   # HIGH → транзистор открыт → ток через катушку
    time.sleep(1.0)                    # ждём, чтобы магнит намагнитился

    print("Шаг 2: взлетаем с грузом")
    drone.control.navigate(
        x=0.0, y=0.0, z=FLIGHT_HEIGHT,
        frame_id="body",
        auto_arm=True,
        speed=0.3            # медленно — несём груз
    )
    time.sleep(8.0)          # ждём набора высоты

    print("Шаг 3: летим к точке доставки (1 м вперёд)")
    drone.control.navigate(
        x=1.0, y=0.0, z=0.0,
        frame_id="body",
        speed=0.3
    )
    time.sleep(6.0)

    print("Шаг 4: опускаемся")
    drone.control.navigate(
        x=0.0, y=0.0, z=-FLIGHT_HEIGHT + 0.1,  # чуть выше земли
        frame_id="body",
        speed=0.2
    )
    time.sleep(5.0)

    print("Шаг 5: выключаем электромагнит — отпускаем груз")
    drone.gpio.magnet_off(MAGNET_PIN)   # LOW → транзистор закрыт → груз падает

    time.sleep(0.5)

    print("Шаг 6: садимся")
    drone.control.land()

finally:
    # В любом случае — выключаем магнит и отпускаем пин
    drone.gpio.magnet_off(MAGNET_PIN)
    drone.gpio.pin_release(MAGNET_PIN)
    drone.close()
