"""
Что мы делаем: калибруем камеру через drone.image.calibration — задаём параметры
доски, набираем кадры автозахватом, считаем параметры камеры и сохраняем результат.
Никаких ручных ros2 service call — всё через обёртку.

Перед запуском подними ноду калибровки (она НЕ в автозапуске):
  ros2 launch camera_calibration calibration.launch.py image_topic:=/camera_1/image_raw

Запуск: python3 camera_calibration_example.py
Во время набора кадров медленно води калибровочной доской перед камерой —
в разных частях кадра, под разными углами, на разном расстоянии.
Превью с распознанной доской — топик /camera_calibration/processed_image (веб-интерфейс).
"""
import time
import sverk_interfaces

CAPTURE_SECONDS = 20.0   # сколько секунд набирать кадры автозахватом

drone = sverk_interfaces.init(Nodename="camera_calibration_example")
cal = drone.image.calibration   # None, если нода camera_calibration не запущена

try:
    if cal is None:
        print("drone.image.calibration недоступен — нода camera_calibration не запущена.")
        print("Подними её и запусти пример снова:")
        print("  ros2 launch camera_calibration calibration.launch.py "
              "image_topic:=/camera_1/image_raw")
    else:
        # 1. Параметры доски (здесь — ChArUco 11x8, клетка 22 мм, маркер 16 мм)
        cal.set_config(board_type="charuco", camera_model="pinhole",
                       board_width=11, board_height=8,
                       square_size=0.022, marker_size=0.016,
                       aruco_dict="DICT_4X4_50", min_frames=10)

        # 2. Чистая сессия + автозахват кадров
        cal.start_session()
        print(f"Набираем кадры {CAPTURE_SECONDS:.0f} c — медленно води доской перед камерой...")
        cal.set_auto_capture(True, period_sec=1.0)
        time.sleep(CAPTURE_SECONDS)
        cal.set_auto_capture(False)

        # 3. Сколько набрали
        print("Состояние:", cal.get_state())
        print("Кадров в наборе:", len(cal.list_captures()))

        # 4. Рассчитать и сохранить
        print("Считаем калибровку (может занять время)...")
        cal.calibrate()
        cal.save()
        print("Готово. latest_calibration.yaml сохранён в output_dir (~/camera_calibrations).")
finally:
    drone.close()
