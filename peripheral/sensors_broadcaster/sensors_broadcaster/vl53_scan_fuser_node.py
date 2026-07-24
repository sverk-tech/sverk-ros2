import math
from functools import partial

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class VL53ScanFuserNode(Node):
    def __init__(self):
        super().__init__('vl53_scan_fuser_node')

        dynamic_desc = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('input_topics', [], dynamic_desc)
        self.declare_parameter('sensor_angles_deg', [], dynamic_desc)
        self.declare_parameter('sensor_fov_deg', [], dynamic_desc)

        self.declare_parameter('projection_mode', 'point')
        self.declare_parameter('overlap_mode', 'average')
        self.declare_parameter('output_topic', '/sensors_broadcaster/scan')
        self.declare_parameter('frame_id', 'vl53_scan')
        self.declare_parameter('angle_min_deg', -90.0)
        self.declare_parameter('angle_max_deg', 90.0)
        self.declare_parameter('angle_increment_deg', 1.0)
        self.declare_parameter('range_min', 0.04)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('measurement_timeout_sec', 0.25)

        self.input_topics = [str(topic) for topic in list(self.get_parameter('input_topics').value or [])]
        self.sensor_angles_deg = self._parse_float_list('sensor_angles_deg')
        self.sensor_fov_deg = self._parse_float_list('sensor_fov_deg')

        self.projection_mode = str(self.get_parameter('projection_mode').value).strip().lower()
        self.overlap_mode = str(self.get_parameter('overlap_mode').value).strip().lower()
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.angle_min_deg = float(self.get_parameter('angle_min_deg').value)
        self.angle_max_deg = float(self.get_parameter('angle_max_deg').value)
        self.angle_increment_deg = float(self.get_parameter('angle_increment_deg').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.measurement_timeout_sec = float(self.get_parameter('measurement_timeout_sec').value)

        if not self.input_topics:
            self.get_logger().error('Parameter input_topics is empty. Nothing to fuse.')
            return

        if len(self.input_topics) != len(self.sensor_angles_deg):
            self.get_logger().error(
                'input_topics and sensor_angles_deg must have the same length. '
                f'Got {len(self.input_topics)} and {len(self.sensor_angles_deg)}.'
            )
            return

        if self.projection_mode not in ('point', 'fov'):
            self.get_logger().error(
                f'Unsupported projection_mode={self.projection_mode!r}. Use "point" or "fov".'
            )
            return

        if self.overlap_mode not in ('average', 'min'):
            self.get_logger().error(
                f'Unsupported overlap_mode={self.overlap_mode!r}. Use "average" or "min".'
            )
            return

        if self.projection_mode == 'fov':
            self.sensor_fov_deg = self._normalize_fov(self.sensor_fov_deg, len(self.input_topics))
            if self.sensor_fov_deg is None:
                self.get_logger().error(
                    'sensor_fov_deg must be empty, a single value, or have the same length as input_topics.'
                )
                return
        else:
            self.sensor_fov_deg = [0.0] * len(self.input_topics)

        if self.angle_increment_deg <= 0.0:
            self.get_logger().error('angle_increment_deg must be > 0.')
            return

        if self.angle_max_deg < self.angle_min_deg:
            self.get_logger().error('angle_max_deg must be >= angle_min_deg.')
            return

        if self.publish_rate_hz <= 0.0:
            self.get_logger().error('publish_rate_hz must be > 0.')
            return

        self.angle_min_rad = math.radians(self.angle_min_deg)
        self.angle_max_rad = math.radians(self.angle_max_deg)
        self.angle_increment_rad = math.radians(self.angle_increment_deg)
        self.sensor_angles_rad = [math.radians(angle) for angle in self.sensor_angles_deg]
        self.sensor_fov_rad = [math.radians(fov) for fov in self.sensor_fov_deg]
        self.scan_size = int(round((self.angle_max_rad - self.angle_min_rad) / self.angle_increment_rad)) + 1

        self.latest_ranges = [None] * len(self.input_topics)
        self.latest_timestamps = [None] * len(self.input_topics)

        for index, topic in enumerate(self.input_topics):
            self.create_subscription(Float32, topic, partial(self._range_callback, index=index), 10)

        self.scan_publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_scan)

        self.get_logger().info(
            'VL53 scan fuser started: '
            f'mode={self.projection_mode}, sensors={len(self.input_topics)}, '
            f'overlap_mode={self.overlap_mode}, output_topic={self.output_topic}, frame_id={self.frame_id}'
        )

    def _parse_float_list(self, name):
        raw_value = self.get_parameter(name).value
        if raw_value is None:
            return []
        return [float(value) for value in list(raw_value)]

    def _normalize_fov(self, values, target_len):
        if not values:
            return [0.0] * target_len
        if len(values) == 1:
            return [float(values[0])] * target_len
        if len(values) == target_len:
            return [float(value) for value in values]
        return None

    def _range_callback(self, msg, index):
        self.latest_ranges[index] = float(msg.data)
        self.latest_timestamps[index] = self.get_clock().now()

    def _publish_scan(self):
        now = self.get_clock().now()
        range_samples = [[] for _ in range(self.scan_size)]

        for index, distance in enumerate(self.latest_ranges):
            if distance is None:
                continue

            last_stamp = self.latest_timestamps[index]
            if last_stamp is None:
                continue

            age_sec = (now - last_stamp).nanoseconds / 1e9
            if age_sec > self.measurement_timeout_sec:
                continue

            if not (self.range_min <= distance <= self.range_max):
                continue

            if self.projection_mode == 'point':
                self._write_point_sample(range_samples, self.sensor_angles_rad[index], distance)
            else:
                self._write_fov_sample(
                    range_samples,
                    self.sensor_angles_rad[index],
                    self.sensor_fov_rad[index],
                    distance,
                )

        ranges = [self._reduce_bin(samples) for samples in range_samples]

        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min_rad
        scan.angle_max = self.angle_max_rad
        scan.angle_increment = self.angle_increment_rad
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate_hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_publisher.publish(scan)

    def _reduce_bin(self, samples):
        if not samples:
            return math.inf
        if self.overlap_mode == 'min':
            return min(samples)
        return sum(samples) / len(samples)

    def _write_point_sample(self, range_samples, angle_rad, distance):
        index = self._angle_to_index(angle_rad)
        if index is None:
            return
        range_samples[index].append(distance)

    def _write_fov_sample(self, range_samples, center_angle_rad, fov_rad, distance):
        if fov_rad <= 0.0:
            self._write_point_sample(range_samples, center_angle_rad, distance)
            return

        half_fov = fov_rad / 2.0
        start_index = self._angle_to_index(center_angle_rad - half_fov)
        end_index = self._angle_to_index(center_angle_rad + half_fov)

        if start_index is None and end_index is None:
            return

        if start_index is None:
            start_index = 0
        if end_index is None:
            end_index = self.scan_size - 1

        start_index, end_index = sorted((start_index, end_index))
        for index in range(start_index, end_index + 1):
            range_samples[index].append(distance)

    def _angle_to_index(self, angle_rad):
        if angle_rad < self.angle_min_rad or angle_rad > self.angle_max_rad:
            return None

        index = int(round((angle_rad - self.angle_min_rad) / self.angle_increment_rad))
        if index < 0 or index >= self.scan_size:
            return None
        return index


def main(args=None):
    rclpy.init(args=args)
    node = VL53ScanFuserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down scan fuser...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
