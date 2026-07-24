"""
Что мы делаем: получаем поток кадров с камеры (уже в формате OpenCV), обрабатываем
(рисуем сетку и информацию) и публикуем результат в топик /out_detection через
drone.image.publish() — он сам конвертирует numpy в ROS Image.

Посмотреть результат: ros2 run rqt_image_view rqt_image_view → /out_detection
Или в веб-интерфейсе Обрика в разделе "View image topics".

Остановка: Ctrl+C.
"""
import cv2
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="image_publish")

frame_count = {"n": 0}

def process_and_publish(img) -> None:
    """Обрабатываем каждый кадр (numpy BGR) и публикуем результат."""
    frame_count["n"] += 1

    # Рисуем вспомогательную сетку и перекрестие
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    cv2.line(img, (cx, 0), (cx, h), (0, 255, 0), 1)      # вертикальная линия
    cv2.line(img, (0, cy), (w, cy), (0, 255, 0), 1)       # горизонтальная линия
    cv2.circle(img, (cx, cy), 20, (0, 255, 0), 2)         # центральный круг

    # Счётчик кадров
    cv2.putText(
        img, f"Frame #{frame_count['n']}", (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
    )

    # Публикуем обработанный кадр в /out_detection.
    # publish() сам упакует numpy BGR в sensor_msgs/Image (метка времени —
    # от последнего кадра, его запомнил take_picture/stream).
    drone.image.publish(img)     # → /out_detection

    if frame_count["n"] % 30 == 0:  # каждые 30 кадров пишем в консоль
        print(f"Опубликовано кадров: {frame_count['n']}")

print("Обрабатываем видео и публикуем в /out_detection... Ctrl+C для выхода")

try:
    drone.image.stream(process_and_publish)   # блокируем до Ctrl+C
except KeyboardInterrupt:
    print(f"\nОстановлено. Всего кадров: {frame_count['n']}")
finally:
    drone.close()
