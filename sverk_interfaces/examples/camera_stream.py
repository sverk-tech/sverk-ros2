"""
Что мы делаем: стримим кадры с камеры, ищем красные объекты через HSV-маску,
и если находим — летим в сторону объекта (следование за красным предметом).

Это пример как соединить drone.image.stream() с drone.control.set_velocity().

Остановка: Ctrl+C — дрон садится.
"""
import time
import threading
import cv2
import sverk_interfaces

# Параметры камеры
FRAME_W = 320     # ожидаемая ширина кадра (для вычисления центра)
FRAME_H = 240     # ожидаемая высота кадра

# Параметры слежения
DEAD_ZONE_PX = 30   # пикселей от центра — не двигаемся (мёртвая зона)
FOLLOW_SPEED = 0.2  # скорость следования в м/с

# Разделяемое состояние между потоками
state = {
    "object_x": None,   # пиксельная X-координата найденного объекта
    "object_y": None,
    "running": True,
}

drone = sverk_interfaces.init(Nodename="camera_stream")

def detect_red(img) -> None:
    """Обрабатываем каждый кадр (numpy BGR): ищем красный объект."""
    # Конвертируем в HSV для выделения цвета
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Маска красного цвета (два диапазона, потому что красный — на обоих концах HSV)
    mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))     # красный ниже 10°
    mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))  # красный выше 160°
    mask = cv2.bitwise_or(mask1, mask2)                          # объединяем

    # Находим контуры красных областей
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)  # берём самый большой контур
        if cv2.contourArea(largest) > 500:            # игнорируем шум (меньше 500 пкс²)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])   # центр масс по X
                cy = int(M["m01"] / M["m00"])   # центр масс по Y
                state["object_x"] = cx
                state["object_y"] = cy
                return

    state["object_x"] = None  # объект не найден


def follow_thread() -> None:
    """В отдельном потоке следим за объектом и управляем дроном."""
    # Ждём взлёта
    drone.control.navigate(x=0.0, y=0.0, z=1.0, frame_id="body", auto_arm=True, speed=0.5)
    time.sleep(8.0)

    while state["running"]:
        ox = state["object_x"]
        if ox is not None:
            error_x = ox - FRAME_W // 2     # насколько объект смещён от центра

            vy = 0.0
            if abs(error_x) > DEAD_ZONE_PX:
                # летим в сторону объекта (ось Y в body — влево/вправо)
                vy = FOLLOW_SPEED * (error_x / (FRAME_W // 2))

            drone.control.set_velocity(vx=0.0, vy=vy, vz=0.0, frame_id="body")
        else:
            # объект не виден — зависаем
            drone.control.set_velocity(vx=0.0, vy=0.0, vz=0.0, frame_id="body")

        time.sleep(0.1)   # 10 Гц


print("Запускаем слежение за красным объектом... Ctrl+C для остановки")

# Запускаем следование в отдельном потоке
t = threading.Thread(target=follow_thread, daemon=True)
t.start()

try:
    drone.image.stream(detect_red)    # блокируем главный поток стримингом
except KeyboardInterrupt:
    print("\nОстановка...")
finally:
    state["running"] = False
    drone.image.stop_stream()
    drone.control.land()
    drone.close()
