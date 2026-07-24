"""
Что мы делаем: стримим кадры с камеры, ищем QR-коды через drone.image.detect_qr(),
рисуем рамку и текст на кадре, публикуем результат в /out_detection и печатаем
каждый новый найденный код.

Посмотреть результат: веб-интерфейс Обрика (View image topics) → /out_detection
или ros2 run rqt_image_view rqt_image_view → /out_detection

Нужен пакет pyzbar: pip install pyzbar (+ системная библиотека: sudo apt install libzbar0)

Остановка: Ctrl+C.
"""
import cv2
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="qr_detect")

seen = set()  # коды, которые уже печатали (чтобы не спамить консоль)

def on_frame(img) -> None:
    """Обрабатываем каждый кадр (numpy BGR): ищем QR и рисуем их."""
    for code in drone.image.detect_qr(img):   # передаём готовый кадр
        x, y, w, h = code.rect
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, code.data, (x, max(15, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if code.data not in seen:
            seen.add(code.data)
            print(f"Новый QR: {code.data!r}  (центр {code.center}, тип {code.type})")

    drone.image.publish(img)   # numpy → /out_detection

print("Ищем QR-коды... покажи код камере. Ctrl+C для выхода")

try:
    drone.image.stream(on_frame)   # блокируем до Ctrl+C
except KeyboardInterrupt:
    print(f"\nВсего разных кодов: {len(seen)}")
finally:
    drone.close()
