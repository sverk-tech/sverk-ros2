from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
from serial import SerialException

from .parser import LD19Packet, LD19Point, LD19Parser


class LD19LidarNode(Node):
    def __init__(self) -> None:
        super().__init__("ld19_lidar_node")

        self.declare_parameter("connection_mode", "usb")
        self.declare_parameter("port_name", "")
        self.declare_parameter("usb_port_name", "/dev/ttyUSB0")
        self.declare_parameter(
            "uart_port_candidates",
            ["/dev/ttyAMA3", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyAMA10", "/dev/ttyS0"],
        )
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_id", "ld19_frame")
        self.declare_parameter("topic_name", "/ld19_lidar/scan")
        self.declare_parameter("scan_bins", 720)
        self.declare_parameter("range_min_m", 0.02)
        self.declare_parameter("range_max_m", 12.0)
        self.declare_parameter("clockwise_data", True)
        self.declare_parameter("read_chunk_size", 256)
        self.declare_parameter("read_timer_period_sec", 0.01)

        self.connection_mode = str(self.get_parameter("connection_mode").value)
        self.port_name = self._select_port()
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.topic_name = str(self.get_parameter("topic_name").value)
        self.scan_bins = int(self.get_parameter("scan_bins").value)
        self.range_min_m = float(self.get_parameter("range_min_m").value)
        self.range_max_m = float(self.get_parameter("range_max_m").value)
        self.clockwise_data = bool(self.get_parameter("clockwise_data").value)
        self.read_chunk_size = int(self.get_parameter("read_chunk_size").value)
        self.read_timer_period_sec = float(
            self.get_parameter("read_timer_period_sec").value
        )

        self.publisher = self.create_publisher(LaserScan, self.topic_name, 10)
        self.parser = LD19Parser()
        self.serial_port: Optional[serial.Serial] = None
        self.current_scan_points: list[LD19Point] = []
        self.last_raw_angle_deg: Optional[float] = None
        self.last_scan_time = None
        self.last_speed_deg_per_sec = 0.0

        self.get_logger().info(
            f"LD19 mode={self.connection_mode}, port={self.port_name}, "
            f"baudrate={self.baudrate}"
        )
        self._connect_serial()
        self.read_timer = self.create_timer(
            self.read_timer_period_sec,
            self._read_serial,
        )

    def _select_port(self) -> str:
        explicit_port = str(self.get_parameter("port_name").value).strip()
        if explicit_port:
            return explicit_port

        mode = str(self.get_parameter("connection_mode").value).strip().lower()
        if mode == "usb":
            return str(self.get_parameter("usb_port_name").value)

        if mode == "uart":
            candidates = [
                str(port)
                for port in self.get_parameter("uart_port_candidates").value
            ]
            for candidate in candidates:
                if Path(candidate).exists():
                    return candidate
            return candidates[0]

        raise ValueError("connection_mode must be 'usb' or 'uart'")

    def _connect_serial(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            return

        try:
            self.serial_port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=0.0,
                write_timeout=1.0,
            )
            self.get_logger().info(
                f"Connected to LD19 on {self.port_name} @ {self.baudrate} baud."
            )
        except SerialException as exc:
            self.serial_port = None
            self.get_logger().warning(
                f"Failed to open {self.port_name}: {exc}. Will retry automatically."
            )

    def _read_serial(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self._connect_serial()
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return

            chunk = self.serial_port.read(min(waiting, self.read_chunk_size))
            if not chunk:
                return

            for packet in self.parser.push(chunk):
                self.last_speed_deg_per_sec = float(packet.speed_deg_per_sec)
                self._handle_packet(packet)
        except SerialException as exc:
            self.get_logger().warning(
                f"Serial error on {self.port_name}: {exc}. Reconnecting."
            )
            self._close_serial()

    def _handle_packet(self, packet: LD19Packet) -> None:
        for point in packet.points:
            if (
                self.last_raw_angle_deg is not None
                and point.angle_deg < self.last_raw_angle_deg
            ):
                self._publish_current_scan()
                self.current_scan_points = []
            self.current_scan_points.append(point)
            self.last_raw_angle_deg = point.angle_deg

    def _publish_current_scan(self) -> None:
        if len(self.current_scan_points) < 30:
            return

        scan = LaserScan()
        now = self.get_clock().now()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_increment = (2.0 * math.pi) / float(self.scan_bins)
        scan.angle_max = (2.0 * math.pi) - scan.angle_increment
        scan.range_min = float(self.range_min_m)
        scan.range_max = float(self.range_max_m)

        if self.last_scan_time is None:
            scan.scan_time = 0.0
            scan.time_increment = 0.0
        else:
            elapsed = now - self.last_scan_time
            scan.scan_time = elapsed.nanoseconds / 1e9
            scan.time_increment = scan.scan_time / float(max(1, self.scan_bins - 1))

        ranges = [math.inf] * self.scan_bins
        intensities = [0.0] * self.scan_bins

        for point in self.current_scan_points:
            distance_m = point.distance_mm / 1000.0
            if distance_m < self.range_min_m or distance_m > self.range_max_m:
                continue

            if self.clockwise_data:
                angle_deg = (360.0 - point.angle_deg) % 360.0
            else:
                angle_deg = point.angle_deg

            index = int(round((angle_deg / 360.0) * self.scan_bins)) % self.scan_bins
            if distance_m < ranges[index]:
                ranges[index] = distance_m
                intensities[index] = float(point.intensity)

        scan.ranges = ranges
        scan.intensities = intensities

        self.publisher.publish(scan)
        self.last_scan_time = now

    def _close_serial(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LD19LidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
