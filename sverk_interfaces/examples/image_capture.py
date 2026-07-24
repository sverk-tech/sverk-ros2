"""
Что мы делаем: получаем один кадр с камеры Обрика через drone.image.take_picture()
(кадр приходит уже в формате OpenCV — numpy BGR) и сохраняем в файл.

Запуск: python3 image_capture.py [выходной_файл.png]
"""
import sys
import cv2
import sverk_interfaces

OUTPUT_FILE = sys.argv[1] if len(sys.argv) > 1 else "/tmp/obrik_frame.png"

drone = sverk_interfaces.init(Nodename="image_capture")
try:
    print("Захватываем кадр с /camera_1/image_raw...")

    # ждём один кадр (таймаут 5 секунд); возвращается numpy BGR или None
    img = drone.image.take_picture(timeout=5.0)

    if img is None:
        print("Ошибка: кадр не получен за 5 секунд.")
        print("Проверь: ros2 topic hz /camera_1/image_raw")
    else:
        print(f"Получен кадр: {img.shape[1]}x{img.shape[0]}")

        cv2.imwrite(OUTPUT_FILE, img)
        print(f"Сохранено в {OUTPUT_FILE}")

        # покажем минимальную статистику
        print(f"Яркость: среднее={img.mean():.1f}, мин={img.min()}, макс={img.max()}")
finally:
    drone.close()
