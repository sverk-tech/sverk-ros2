"""
Что мы делаем: публикуем свои данные в кастомный ROS 2 топик через
drone.topic.create_publisher — например, результаты анализа изображения
(координата найденного объекта).

Другие ноды (или другие программы) могут подписаться на этот топик
через drone.topic.subscribe().

Запусти в одном терминале этот скрипт, а в другом посмотри данные:
  ros2 topic echo /detection_x
"""
import time
import math
import sverk_interfaces
from std_msgs.msg import Float32  # тип сообщения: одно число

drone = sverk_interfaces.init(Nodename="topic_publisher")

# создаём публикатор — один раз, потом вызываем .publish() много раз
pub = drone.topic.create_publisher(Float32, "/detection_x")

print("Публикуем синусоиду в /detection_x... Ctrl+C для выхода")

try:
    t = 0.0
    while True:
        msg = Float32()
        msg.data = math.sin(t) * 100.0   # имитируем позицию X объекта в пикселях
        pub.publish(msg)
        print(f"  Опубликовано: detection_x = {msg.data:.1f}")
        t += 0.1
        time.sleep(0.1)                   # 10 Гц
except KeyboardInterrupt:
    print("\nОстановлено.")
finally:
    drone.close()
