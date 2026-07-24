#!/usr/bin/env python3
"""SITL-адаптер QoS для camera_info.

Зачем: ноды aruco_det_loc подписываются на camera_info с
reliable+transient_local — так публикует драйвер реальной камеры (latched,
поздно стартовавшая нода всё равно получает K и D). В SITL же camera_info
приходит из ros_gz parameter_bridge, который публикует VOLATILE — QoS
несовместим (DURABILITY), подписка aruco не матчится, калибровка не доходит
и детекция молча не работает.

Эта нода слушает тот же топик volatile-подпиской (совместима и с volatile,
и с transient_local паблишерами) и перепубликовывает СВЕЖЕЕ сообщение на том
же топике latched-паблишером (KeepLast(1) + transient_local) — ровно так, как
это делает реальный драйвер. Собственные сообщения (нода получает их из-за
совпадающего топика) отфильтровываются по содержимому, поэтому петли нет;
camera_info статичен, так что фактически публикация происходит один раз.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoLatch(Node):
    def __init__(self):
        super().__init__('camera_info_latch')
        self.declare_parameter('topic', '/camera_1/camera_info')
        topic = self.get_parameter('topic').get_parameter_value().string_value

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._last_key = None
        self._pub = self.create_publisher(CameraInfo, topic, latched)
        self._sub = self.create_subscription(CameraInfo, topic, self._cb, volatile)
        self.get_logger().info(f'Latching camera_info on {topic} (volatile -> transient_local)')

    def _cb(self, msg: CameraInfo):
        key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d))
        if key == self._last_key:
            return
        self._last_key = key
        self._pub.publish(msg)
        self.get_logger().info(
            f'camera_info latched: {msg.width}x{msg.height}, fx={msg.k[0]:.1f}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoLatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
