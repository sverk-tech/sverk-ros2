"""
Что мы делаем: подписываемся на произвольный ROS 2 топик через
drone.topic.subscribe и выводим данные в реальном времени.

Пример показывает подписку на локальную позицию от PX4.
Запусти вместе с topic_publish.py или самостоятельно когда Обрик включён.
"""
import sverk_interfaces
from px4_msgs.msg import VehicleLocalPosition  # сообщение PX4 с позицией

drone = sverk_interfaces.init(Nodename="topic_subscriber")

# счётчик полученных сообщений
counter = {"n": 0}

def on_position(msg: VehicleLocalPosition) -> None:
    """Вызывается при каждом новом сообщении о позиции дрона."""
    counter["n"] += 1
    # x, y, z — позиция в метрах (ENU: x=восток, y=север, z=вверх)
    # vx, vy, vz — скорость в м/с
    print(
        f"[{counter['n']:4d}] "
        f"pos=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}) м  "
        f"vel=({msg.vx:.2f}, {msg.vy:.2f}, {msg.vz:.2f}) м/с"
    )

print("Получаем позицию PX4... Ctrl+C для выхода")
print("-" * 60)

# подписываемся — callback будет вызываться при каждом новом сообщении
sub = drone.topic.subscribe(
    VehicleLocalPosition,
    "/fmu/out/vehicle_local_position",
    on_position
)

try:
    drone.topic.spin()    # блокируем до Ctrl+C
except KeyboardInterrupt:
    print(f"\nПолучено сообщений: {counter['n']}")
finally:
    drone.topic.unsubscribe(sub)  # отписываемся (хорошая практика)
    drone.close()
