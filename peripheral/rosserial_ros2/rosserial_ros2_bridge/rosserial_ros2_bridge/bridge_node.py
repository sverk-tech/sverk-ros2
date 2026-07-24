#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import struct
import sys
import time

import serial

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Range
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from rosserial_ros2_bridge.protocol_codec import (
    ID_LOG,
    ID_PARAMETER_REQUEST,
    ID_PUBLISHER,
    ID_SERVICE_CLIENT,
    ID_SERVICE_SERVER,
    ID_SUBSCRIBER,
    ID_TIME,
    build_packet,
    decode_geometry_msgs_quaternion,
    decode_geometry_msgs_quaternion_stamped,
    decode_geometry_msgs_point,
    decode_geometry_msgs_pose,
    decode_geometry_msgs_pose_stamped,
    decode_geometry_msgs_twist,
    decode_geometry_msgs_twist_stamped,
    decode_geometry_msgs_vector3,
    decode_geometry_msgs_vector3_stamped,
    decode_request_param_request,
    decode_rosserial_log,
    decode_sensor_msgs_battery_state,
    decode_sensor_msgs_range,
    decode_sensor_msgs_fluid_pressure,
    decode_sensor_msgs_imu,
    decode_sensor_msgs_illuminance,
    decode_sensor_msgs_laser_scan,
    decode_sensor_msgs_magnetic_field,
    decode_sensor_msgs_relative_humidity,
    decode_sensor_msgs_temperature,
    decode_std_msgs_bool,
    decode_std_msgs_color_rgba,
    decode_std_msgs_empty,
    decode_std_msgs_float32,
    decode_std_msgs_float64,
    decode_std_msgs_int8,
    decode_std_msgs_int16,
    decode_std_msgs_int32,
    decode_std_msgs_string,
    decode_std_msgs_uint8,
    decode_std_msgs_uint32,
    decode_std_msgs_uint16,
    decode_std_srvs_set_bool_request,
    decode_std_srvs_set_bool_response,
    decode_std_srvs_trigger_request,
    decode_std_srvs_trigger_response,
    decode_topic_info,
    encode_geometry_msgs_quaternion,
    encode_geometry_msgs_quaternion_stamped,
    encode_geometry_msgs_point,
    encode_geometry_msgs_pose,
    encode_geometry_msgs_pose_stamped,
    encode_geometry_msgs_twist,
    encode_geometry_msgs_twist_stamped,
    encode_geometry_msgs_vector3,
    encode_geometry_msgs_vector3_stamped,
    encode_request_param_response,
    encode_std_msgs_bool,
    encode_std_msgs_color_rgba,
    encode_std_msgs_empty,
    encode_std_msgs_float32,
    encode_std_msgs_float64,
    encode_std_msgs_int8,
    encode_std_msgs_int16,
    encode_std_msgs_int32,
    encode_std_msgs_string,
    encode_std_msgs_time,
    encode_std_msgs_uint8,
    encode_std_msgs_uint32,
    encode_std_msgs_uint16,
    encode_std_srvs_set_bool_request,
    encode_std_srvs_set_bool_response,
    encode_std_srvs_trigger_request,
    encode_std_srvs_trigger_response,
    read_packet,
)
from rosserial_ros2_bridge.generic_codec import (
    load_generated_schemas,
    make_message_decoder,
    make_message_encoder,
    make_service_request_decoder,
    make_service_request_encoder,
    make_service_response_decoder,
    make_service_response_encoder,
    ros2_message_class,
    ros2_service_class,
)


@dataclass(frozen=True)
class TypeSupport:
    ros_msg_type: type
    decode: object
    encode: object | None = None


@dataclass(frozen=True)
class ServiceTypeSupport:
    ros_srv_type: type
    decode_request: object
    encode_request: object
    decode_response: object
    encode_response: object


def rosserial_type_aliases(message_type):
    if "/msg/" in message_type:
        package, name = message_type.split("/msg/", 1)
        return {message_type, f"{package}/{name}"}
    package, name = message_type.split("/", 1)
    return {message_type, f"{package}/msg/{name}"}


def rosserial_schema_key(interface_type):
    if "/msg/" in interface_type:
        package, name = interface_type.split("/msg/", 1)
        return f"{package}/{name}"
    if "/srv/" in interface_type:
        package, name = interface_type.split("/srv/", 1)
        return f"{package}/{name}"
    return interface_type


class RosserialBridge(Node):
    def __init__(self, port=None, baud=None):
        super().__init__(
            "rosserial_bridge",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        if not self.has_parameter("port"):
            self.declare_parameter("port", port or "/dev/ttyS4")
        if not self.has_parameter("baud"):
            self.declare_parameter("baud", baud if baud is not None else 57600)
        if not self.has_parameter("reconnect_delay"):
            self.declare_parameter("reconnect_delay", 2.0)
        if not self.has_parameter("max_payload_size"):
            self.declare_parameter("max_payload_size", 4096)
        if not self.has_parameter("topic_request_interval"):
            self.declare_parameter("topic_request_interval", 1.0)
        if not self.has_parameter("stats_log_interval"):
            self.declare_parameter("stats_log_interval", 10.0)
        if not self.has_parameter("diagnostics_topic"):
            self.declare_parameter("diagnostics_topic", "/rosserial_ros2_bridge/diagnostics")
        if not self.has_parameter("diagnostics_period"):
            self.declare_parameter("diagnostics_period", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_delay = float(self.get_parameter("reconnect_delay").value)
        self.max_payload_size = int(self.get_parameter("max_payload_size").value)
        self.topic_request_interval = float(self.get_parameter("topic_request_interval").value)
        self.stats_log_interval = float(self.get_parameter("stats_log_interval").value)
        self.diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self.diagnostics_period = float(self.get_parameter("diagnostics_period").value)
        self.known_topics = {}
        self.topic_publishers = {}
        self.topic_subscriptions = {}
        self.service_servers = {}
        self.service_server_endpoints = {}
        self.service_clients = {}
        self.service_client_endpoints = {}
        self.next_topic_request = 0.0
        self.next_reconnect = 0.0
        self.next_stats_log = time.monotonic() + self.stats_log_interval
        self.packet_stats = {}
        self.last_logged_packet_error_stats = {}
        self.serial = None
        self.last_disconnect_reason = ""
        self.generated_schemas = load_generated_schemas()
        self.setup_type_registry()
        self.setup_service_registry()
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, self.diagnostics_topic, 10)

        self.timer = self.create_timer(0.02, self.poll_serial)
        self.diagnostics_timer = self.create_timer(self.diagnostics_period, self.publish_diagnostics)

    def payload_fits_arduino_buffer(self, payload, buffer_size, endpoint_name):
        if buffer_size and len(payload) > buffer_size:
            self.packet_stats["oversized_to_arduino"] = self.packet_stats.get("oversized_to_arduino", 0) + 1
            self.get_logger().warn(
                f"Dropping outbound rosserial payload for {endpoint_name}: "
                f"payload={len(payload)} bytes exceeds Arduino buffer={buffer_size} bytes"
            )
            return False
        return True

    def destroy_node(self):
        try:
            if self.serial:
                self.serial.close()
        finally:
            super().destroy_node()

    def open_transport(self):
        try:
            self.serial = serial.serial_for_url(self.port, self.baud, timeout=0.05)
            try:
                self.serial.dtr = False
                time.sleep(0.1)
                self.serial.reset_input_buffer()
                self.serial.dtr = True
            except (AttributeError, OSError, serial.SerialException):
                pass
            time.sleep(2.0)
        except (OSError, serial.SerialException) as exc:
            self.serial = None
            self.next_reconnect = time.monotonic() + self.reconnect_delay
            self.last_disconnect_reason = str(exc)
            self.get_logger().warn(
                f"Failed to open {self.port}: {exc}. Retrying in {self.reconnect_delay:.1f}s."
            )
            return False

        self.reset_rosserial_state()
        self.last_disconnect_reason = ""
        self.get_logger().info(f"Opened {self.port} @ {self.baud}")
        self.request_topics()
        return True

    def close_transport(self, reason):
        if self.serial:
            try:
                self.serial.close()
            except (OSError, serial.SerialException):
                pass
        self.serial = None
        self.next_reconnect = time.monotonic() + self.reconnect_delay
        self.last_disconnect_reason = str(reason)
        self.get_logger().warn(f"rosserial transport disconnected: {reason}. Reconnecting automatically.")

    def reset_rosserial_state(self):
        for state in self.topic_publishers.values():
            publisher = state.get("publisher")
            if publisher:
                self.destroy_publisher(publisher)
        for subscription in self.topic_subscriptions.values():
            self.destroy_subscription(subscription)
        for state in self.service_server_endpoints.values():
            service = state.get("service")
            if service:
                self.destroy_service(service)
        for state in self.service_client_endpoints.values():
            client = state.get("client")
            if client:
                self.destroy_client(client)
        self.known_topics.clear()
        self.topic_publishers.clear()
        self.topic_subscriptions.clear()
        self.service_server_endpoints.clear()
        self.service_clients.clear()
        self.service_client_endpoints.clear()
        self.next_topic_request = 0.0

    def write_packet(self, topic_id, payload=b""):
        if not self.serial:
            return False
        try:
            self.serial.write(build_packet(topic_id, payload))
            self.serial.flush()
            return True
        except (OSError, serial.SerialException, ValueError) as exc:
            self.close_transport(exc)
            return False

    def request_topics(self):
        if self.write_packet(ID_PUBLISHER):
            self.next_topic_request = time.monotonic() + self.topic_request_interval

    def send_time(self):
        now = self.get_clock().now().nanoseconds
        sec = now // 1_000_000_000
        nanosec = now % 1_000_000_000
        payload = encode_std_msgs_time(sec, nanosec)
        self.write_packet(ID_TIME, payload)

    def handle_log(self, payload):
        decoded = decode_rosserial_log(payload)
        message = f"Arduino: {decoded['msg']}"
        level = decoded["level"]

        if level == 0:
            self.get_logger().debug(message)
        elif level == 1:
            self.get_logger().info(message)
        elif level == 2:
            self.get_logger().warn(message)
        elif level == 3:
            self.get_logger().error(message)
        elif level == 4:
            self.get_logger().fatal(message)
        else:
            self.get_logger().warn(f"Arduino log with unknown level {level}: {decoded['msg']}")

    def handle_parameter_request(self, payload):
        request = decode_request_param_request(payload)
        name = request["name"]
        if name.startswith("~"):
            lookup_name = name[1:]
        else:
            lookup_name = name.lstrip("/")

        ints = []
        floats = []
        strings = []

        if not lookup_name:
            self.get_logger().warn("Arduino requested an empty parameter name")
        elif not self.has_parameter(lookup_name):
            self.get_logger().warn(f"Arduino requested missing parameter: {name}")
        else:
            value = self.get_parameter(lookup_name).value
            try:
                ints, floats, strings = self.parameter_value_to_rosserial_arrays(name, value)
            except ValueError as exc:
                self.get_logger().warn(str(exc))

        self.write_packet(ID_PARAMETER_REQUEST, encode_request_param_response(ints, floats, strings))

    def parameter_value_to_rosserial_arrays(self, name, value):
        if value is None:
            raise ValueError(f"Arduino requested unset parameter: {name}")

        if isinstance(value, (list, tuple)):
            values = list(value)
        else:
            values = [value]

        if not values:
            raise ValueError(f"Arduino requested empty parameter array: {name}")

        if all(isinstance(item, bool) for item in values):
            return [1 if item else 0 for item in values], [], []
        if all(isinstance(item, int) and not isinstance(item, bool) for item in values):
            return [int(item) for item in values], [], []
        if all(isinstance(item, float) for item in values):
            return [], [float(item) for item in values], []
        if all(isinstance(item, str) for item in values):
            return [], [], [str(item) for item in values]

        raise ValueError(
            f"Cannot send parameter {name} to Arduino: expected bool, int, float, string, "
            "or a homogeneous array of one of those types."
        )

    def setup_type_registry(self):
        def decode_string(node, payload):
            msg = String()
            msg.data = decode_std_msgs_string(payload)
            return msg

        def encode_string(_node, msg):
            return encode_std_msgs_string(msg.data)

        def decode_bool(node, payload):
            msg = Bool()
            msg.data = decode_std_msgs_bool(payload)
            return msg

        def encode_bool(_node, msg):
            return encode_std_msgs_bool(msg.data)

        def decode_int8(node, payload):
            msg = Int8()
            msg.data = decode_std_msgs_int8(payload)
            return msg

        def encode_int8(_node, msg):
            return encode_std_msgs_int8(msg.data)

        def decode_uint8(node, payload):
            msg = UInt8()
            msg.data = decode_std_msgs_uint8(payload)
            return msg

        def encode_uint8(_node, msg):
            return encode_std_msgs_uint8(msg.data)

        def decode_uint16(node, payload):
            msg = UInt16()
            msg.data = decode_std_msgs_uint16(payload)
            return msg

        def encode_uint16(_node, msg):
            return encode_std_msgs_uint16(msg.data)

        def decode_int16(node, payload):
            msg = Int16()
            msg.data = decode_std_msgs_int16(payload)
            return msg

        def encode_int16(_node, msg):
            return encode_std_msgs_int16(msg.data)

        def decode_int32(node, payload):
            msg = Int32()
            msg.data = decode_std_msgs_int32(payload)
            return msg

        def encode_int32(_node, msg):
            return encode_std_msgs_int32(msg.data)

        def decode_uint32(node, payload):
            msg = UInt32()
            msg.data = decode_std_msgs_uint32(payload)
            return msg

        def encode_uint32(_node, msg):
            return encode_std_msgs_uint32(msg.data)

        def decode_float32(node, payload):
            msg = Float32()
            msg.data = decode_std_msgs_float32(payload)
            return msg

        def encode_float32(_node, msg):
            return encode_std_msgs_float32(msg.data)

        def decode_float64(node, payload):
            msg = Float64()
            msg.data = decode_std_msgs_float64(payload)
            return msg

        def encode_float64(_node, msg):
            return encode_std_msgs_float64(msg.data)

        def decode_empty(node, payload):
            decode_std_msgs_empty(payload)
            return Empty()

        def encode_empty(_node, msg):
            return encode_std_msgs_empty(None)

        def decode_color_rgba(node, payload):
            decoded = decode_std_msgs_color_rgba(payload)
            msg = ColorRGBA()
            msg.r = decoded["r"]
            msg.g = decoded["g"]
            msg.b = decoded["b"]
            msg.a = decoded["a"]
            return msg

        def encode_color_rgba(_node, msg):
            return encode_std_msgs_color_rgba(msg)

        def apply_header(node, msg, decoded_header):
            if decoded_header["stamp_sec"] or decoded_header["stamp_nanosec"]:
                msg.header.stamp.sec = int(decoded_header["stamp_sec"])
                msg.header.stamp.nanosec = int(decoded_header["stamp_nanosec"])
            else:
                msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = decoded_header["frame_id"]

        def decode_range(node, payload):
            decoded = decode_sensor_msgs_range(payload)
            msg = Range()
            apply_header(node, msg, decoded["header"])
            msg.radiation_type = decoded["radiation_type"]
            msg.field_of_view = decoded["field_of_view"]
            msg.min_range = decoded["min_range"]
            msg.max_range = decoded["max_range"]
            msg.range = decoded["range"]
            return msg

        def decode_point(node, payload):
            decoded, _ = decode_geometry_msgs_point(payload)
            msg = Point()
            msg.x = decoded["x"]
            msg.y = decoded["y"]
            msg.z = decoded["z"]
            return msg

        def encode_point(_node, msg):
            return encode_geometry_msgs_point(msg)

        def decode_vector3(node, payload):
            decoded, _ = decode_geometry_msgs_vector3(payload)
            msg = Vector3()
            msg.x = decoded["x"]
            msg.y = decoded["y"]
            msg.z = decoded["z"]
            return msg

        def encode_vector3(_node, msg):
            return encode_geometry_msgs_vector3(msg)

        def decode_quaternion(node, payload):
            decoded, _ = decode_geometry_msgs_quaternion(payload)
            msg = Quaternion()
            msg.x = decoded["x"]
            msg.y = decoded["y"]
            msg.z = decoded["z"]
            msg.w = decoded["w"]
            return msg

        def encode_quaternion(_node, msg):
            return encode_geometry_msgs_quaternion(msg)

        def decode_twist(node, payload):
            decoded = decode_geometry_msgs_twist(payload)
            msg = Twist()
            msg.linear.x = decoded["linear"]["x"]
            msg.linear.y = decoded["linear"]["y"]
            msg.linear.z = decoded["linear"]["z"]
            msg.angular.x = decoded["angular"]["x"]
            msg.angular.y = decoded["angular"]["y"]
            msg.angular.z = decoded["angular"]["z"]
            return msg

        def encode_twist(_node, msg):
            return encode_geometry_msgs_twist(msg)

        def fill_pose(msg, decoded):
            msg.position.x = decoded["position"]["x"]
            msg.position.y = decoded["position"]["y"]
            msg.position.z = decoded["position"]["z"]
            msg.orientation.x = decoded["orientation"]["x"]
            msg.orientation.y = decoded["orientation"]["y"]
            msg.orientation.z = decoded["orientation"]["z"]
            msg.orientation.w = decoded["orientation"]["w"]

        def fill_twist(msg, decoded):
            msg.linear.x = decoded["linear"]["x"]
            msg.linear.y = decoded["linear"]["y"]
            msg.linear.z = decoded["linear"]["z"]
            msg.angular.x = decoded["angular"]["x"]
            msg.angular.y = decoded["angular"]["y"]
            msg.angular.z = decoded["angular"]["z"]

        def decode_pose(node, payload):
            decoded, _ = decode_geometry_msgs_pose(payload)
            msg = Pose()
            fill_pose(msg, decoded)
            return msg

        def encode_pose(_node, msg):
            return encode_geometry_msgs_pose(msg)

        def decode_pose_stamped(node, payload):
            decoded = decode_geometry_msgs_pose_stamped(payload)
            msg = PoseStamped()
            apply_header(node, msg, decoded["header"])
            fill_pose(msg.pose, decoded["pose"])
            return msg

        def encode_pose_stamped(_node, msg):
            return encode_geometry_msgs_pose_stamped(msg)

        def decode_twist_stamped(node, payload):
            decoded = decode_geometry_msgs_twist_stamped(payload)
            msg = TwistStamped()
            apply_header(node, msg, decoded["header"])
            fill_twist(msg.twist, decoded["twist"])
            return msg

        def encode_twist_stamped(_node, msg):
            return encode_geometry_msgs_twist_stamped(msg)

        def decode_vector3_stamped(node, payload):
            decoded = decode_geometry_msgs_vector3_stamped(payload)
            msg = Vector3Stamped()
            apply_header(node, msg, decoded["header"])
            msg.vector.x = decoded["vector"]["x"]
            msg.vector.y = decoded["vector"]["y"]
            msg.vector.z = decoded["vector"]["z"]
            return msg

        def encode_vector3_stamped(_node, msg):
            return encode_geometry_msgs_vector3_stamped(msg)

        def decode_quaternion_stamped(node, payload):
            decoded = decode_geometry_msgs_quaternion_stamped(payload)
            msg = QuaternionStamped()
            apply_header(node, msg, decoded["header"])
            msg.quaternion.x = decoded["quaternion"]["x"]
            msg.quaternion.y = decoded["quaternion"]["y"]
            msg.quaternion.z = decoded["quaternion"]["z"]
            msg.quaternion.w = decoded["quaternion"]["w"]
            return msg

        def encode_quaternion_stamped(_node, msg):
            return encode_geometry_msgs_quaternion_stamped(msg)

        def decode_temperature(node, payload):
            decoded = decode_sensor_msgs_temperature(payload)
            msg = Temperature()
            apply_header(node, msg, decoded["header"])
            msg.temperature = decoded["temperature"]
            msg.variance = decoded["variance"]
            return msg

        def decode_relative_humidity(node, payload):
            decoded = decode_sensor_msgs_relative_humidity(payload)
            msg = RelativeHumidity()
            apply_header(node, msg, decoded["header"])
            msg.relative_humidity = decoded["relative_humidity"]
            msg.variance = decoded["variance"]
            return msg

        def decode_fluid_pressure(node, payload):
            decoded = decode_sensor_msgs_fluid_pressure(payload)
            msg = FluidPressure()
            apply_header(node, msg, decoded["header"])
            msg.fluid_pressure = decoded["fluid_pressure"]
            msg.variance = decoded["variance"]
            return msg

        def decode_illuminance(node, payload):
            decoded = decode_sensor_msgs_illuminance(payload)
            msg = Illuminance()
            apply_header(node, msg, decoded["header"])
            msg.illuminance = decoded["illuminance"]
            msg.variance = decoded["variance"]
            return msg

        def decode_imu(node, payload):
            decoded = decode_sensor_msgs_imu(payload)
            msg = Imu()
            apply_header(node, msg, decoded["header"])
            msg.orientation.x = decoded["orientation"]["x"]
            msg.orientation.y = decoded["orientation"]["y"]
            msg.orientation.z = decoded["orientation"]["z"]
            msg.orientation.w = decoded["orientation"]["w"]
            msg.orientation_covariance = decoded["orientation_covariance"]
            msg.angular_velocity.x = decoded["angular_velocity"]["x"]
            msg.angular_velocity.y = decoded["angular_velocity"]["y"]
            msg.angular_velocity.z = decoded["angular_velocity"]["z"]
            msg.angular_velocity_covariance = decoded["angular_velocity_covariance"]
            msg.linear_acceleration.x = decoded["linear_acceleration"]["x"]
            msg.linear_acceleration.y = decoded["linear_acceleration"]["y"]
            msg.linear_acceleration.z = decoded["linear_acceleration"]["z"]
            msg.linear_acceleration_covariance = decoded["linear_acceleration_covariance"]
            return msg

        def decode_magnetic_field(node, payload):
            decoded = decode_sensor_msgs_magnetic_field(payload)
            msg = MagneticField()
            apply_header(node, msg, decoded["header"])
            msg.magnetic_field.x = decoded["magnetic_field"]["x"]
            msg.magnetic_field.y = decoded["magnetic_field"]["y"]
            msg.magnetic_field.z = decoded["magnetic_field"]["z"]
            msg.magnetic_field_covariance = decoded["magnetic_field_covariance"]
            return msg

        def decode_battery_state(node, payload):
            decoded = decode_sensor_msgs_battery_state(payload)
            msg = BatteryState()
            apply_header(node, msg, decoded["header"])
            msg.voltage = decoded["voltage"]
            msg.temperature = decoded["temperature"]
            msg.current = decoded["current"]
            msg.charge = decoded["charge"]
            msg.capacity = decoded["capacity"]
            msg.design_capacity = decoded["design_capacity"]
            msg.percentage = decoded["percentage"]
            msg.power_supply_status = decoded["power_supply_status"]
            msg.power_supply_health = decoded["power_supply_health"]
            msg.power_supply_technology = decoded["power_supply_technology"]
            msg.present = decoded["present"]
            msg.cell_voltage = decoded["cell_voltage"]
            msg.cell_temperature = decoded["cell_temperature"]
            msg.location = decoded["location"]
            msg.serial_number = decoded["serial_number"]
            return msg

        def decode_laser_scan(node, payload):
            decoded = decode_sensor_msgs_laser_scan(payload)
            msg = LaserScan()
            apply_header(node, msg, decoded["header"])
            msg.angle_min = decoded["angle_min"]
            msg.angle_max = decoded["angle_max"]
            msg.angle_increment = decoded["angle_increment"]
            msg.time_increment = decoded["time_increment"]
            msg.scan_time = decoded["scan_time"]
            msg.range_min = decoded["range_min"]
            msg.range_max = decoded["range_max"]
            msg.ranges = decoded["ranges"]
            msg.intensities = decoded["intensities"]
            return msg

        registry = {
            "std_msgs/String": TypeSupport(String, decode_string, encode_string),
            "std_msgs/Bool": TypeSupport(Bool, decode_bool, encode_bool),
            "std_msgs/Empty": TypeSupport(Empty, decode_empty, encode_empty),
            "std_msgs/Int8": TypeSupport(Int8, decode_int8, encode_int8),
            "std_msgs/UInt8": TypeSupport(UInt8, decode_uint8, encode_uint8),
            "std_msgs/Int16": TypeSupport(Int16, decode_int16, encode_int16),
            "std_msgs/UInt16": TypeSupport(UInt16, decode_uint16, encode_uint16),
            "std_msgs/Int32": TypeSupport(Int32, decode_int32, encode_int32),
            "std_msgs/UInt32": TypeSupport(UInt32, decode_uint32, encode_uint32),
            "std_msgs/Float32": TypeSupport(Float32, decode_float32, encode_float32),
            "std_msgs/Float64": TypeSupport(Float64, decode_float64, encode_float64),
            "std_msgs/ColorRGBA": TypeSupport(ColorRGBA, decode_color_rgba, encode_color_rgba),
            "sensor_msgs/Range": TypeSupport(Range, decode_range, None),
            "geometry_msgs/Point": TypeSupport(Point, decode_point, encode_point),
            "geometry_msgs/Vector3": TypeSupport(Vector3, decode_vector3, encode_vector3),
            "geometry_msgs/Quaternion": TypeSupport(Quaternion, decode_quaternion, encode_quaternion),
            "geometry_msgs/Twist": TypeSupport(Twist, decode_twist, encode_twist),
            "geometry_msgs/Pose": TypeSupport(Pose, decode_pose, encode_pose),
            "geometry_msgs/PoseStamped": TypeSupport(PoseStamped, decode_pose_stamped, encode_pose_stamped),
            "geometry_msgs/TwistStamped": TypeSupport(TwistStamped, decode_twist_stamped, encode_twist_stamped),
            "geometry_msgs/Vector3Stamped": TypeSupport(Vector3Stamped, decode_vector3_stamped, encode_vector3_stamped),
            "geometry_msgs/QuaternionStamped": TypeSupport(QuaternionStamped, decode_quaternion_stamped, encode_quaternion_stamped),
            "sensor_msgs/Temperature": TypeSupport(Temperature, decode_temperature, None),
            "sensor_msgs/RelativeHumidity": TypeSupport(RelativeHumidity, decode_relative_humidity, None),
            "sensor_msgs/FluidPressure": TypeSupport(FluidPressure, decode_fluid_pressure, None),
            "sensor_msgs/Illuminance": TypeSupport(Illuminance, decode_illuminance, None),
            "sensor_msgs/Imu": TypeSupport(Imu, decode_imu, None),
            "sensor_msgs/MagneticField": TypeSupport(MagneticField, decode_magnetic_field, None),
            "sensor_msgs/BatteryState": TypeSupport(BatteryState, decode_battery_state, None),
            "sensor_msgs/LaserScan": TypeSupport(LaserScan, decode_laser_scan, None),
        }
        self.type_registry = {}
        for message_type, support in registry.items():
            for alias in rosserial_type_aliases(message_type):
                self.type_registry[alias] = support

        for message_type, schema in self.generated_schemas.get("messages", {}).items():
            if message_type in self.type_registry:
                continue
            try:
                ros_msg_type = ros2_message_class(message_type)
            except (AttributeError, ImportError, ValueError) as exc:
                self.get_logger().debug(f"Skipping generated message {message_type}: {exc}")
                continue
            support = TypeSupport(
                ros_msg_type,
                make_message_decoder(ros_msg_type, schema),
                make_message_encoder(schema),
            )
            for alias in rosserial_type_aliases(message_type):
                self.type_registry.setdefault(alias, support)

    def setup_service_registry(self):
        def decode_set_bool_request(_node, payload):
            decoded = decode_std_srvs_set_bool_request(payload)
            request = SetBool.Request()
            request.data = decoded["data"]
            return request

        def encode_set_bool_request(_node, request):
            return encode_std_srvs_set_bool_request(request)

        def decode_set_bool_response(_node, payload):
            decoded = decode_std_srvs_set_bool_response(payload)
            response = SetBool.Response()
            response.success = decoded["success"]
            response.message = decoded["message"]
            return response

        def encode_set_bool_response(_node, response):
            return encode_std_srvs_set_bool_response(response)

        def decode_trigger_request(_node, payload):
            decode_std_srvs_trigger_request(payload)
            return Trigger.Request()

        def encode_trigger_request(_node, request):
            return encode_std_srvs_trigger_request(request)

        def decode_trigger_response(_node, payload):
            decoded = decode_std_srvs_trigger_response(payload)
            response = Trigger.Response()
            response.success = decoded["success"]
            response.message = decoded["message"]
            return response

        def encode_trigger_response(_node, response):
            return encode_std_srvs_trigger_response(response)

        registry = {
            "std_srvs/SetBool": ServiceTypeSupport(
                SetBool,
                decode_set_bool_request,
                encode_set_bool_request,
                decode_set_bool_response,
                encode_set_bool_response,
            ),
            "std_srvs/Trigger": ServiceTypeSupport(
                Trigger,
                decode_trigger_request,
                encode_trigger_request,
                decode_trigger_response,
                encode_trigger_response,
            ),
        }
        self.service_registry = {}
        for service_type, support in registry.items():
            for alias in rosserial_type_aliases(service_type):
                self.service_registry[alias] = support

        for service_type, schema in self.generated_schemas.get("services", {}).items():
            if service_type in self.service_registry:
                continue
            try:
                ros_srv_type = ros2_service_class(service_type)
            except (AttributeError, ImportError, ValueError) as exc:
                self.get_logger().debug(f"Skipping generated service {service_type}: {exc}")
                continue
            support = ServiceTypeSupport(
                ros_srv_type,
                make_service_request_decoder(ros_srv_type, schema),
                make_service_request_encoder(schema),
                make_service_response_decoder(ros_srv_type, schema),
                make_service_response_encoder(schema),
            )
            for alias in rosserial_type_aliases(service_type):
                self.service_registry.setdefault(alias, support)

    def get_type_support(self, message_type):
        support = self.type_registry.get(message_type)
        if not support:
            self.get_logger().warn(f"Unsupported message type: {message_type}")
        return support

    def get_service_support(self, service_type):
        support = self.service_registry.get(service_type)
        if not support:
            self.get_logger().warn(f"Unsupported service type: {service_type}")
        return support

    def validate_md5(self, info, interface_kind):
        key = rosserial_schema_key(info["message_type"])
        schemas = self.generated_schemas.get(interface_kind, {})
        expected = schemas.get(key, {}).get("md5")
        actual = info.get("md5sum")
        if not expected or not actual:
            return
        if actual != expected:
            self.get_logger().warn(
                f"MD5 mismatch for {info['topic_name']} [{info['message_type']}]: "
                f"Arduino={actual}, bridge={expected}. "
                "The topic/service may decode incorrectly; regenerate ros_lib for this interface."
            )

    def setup_publisher(self, info):
        topic_id = info["topic_id"]
        message_type = info["message_type"]
        topic_name = info["topic_name"]
        self.validate_md5(info, "messages")
        support = self.get_type_support(message_type)
        if not support:
            return

        self.topic_publishers[topic_id] = {
            "publisher": self.create_publisher(support.ros_msg_type, topic_name, 10),
            "decode": support.decode,
        }
        self.get_logger().info(
            f"Created ROS 2 publisher for {topic_name} [{message_type}] "
            f"from rosserial topic_id={topic_id}"
        )

    def setup_subscription(self, info):
        topic_id = info["topic_id"]
        message_type = info["message_type"]
        topic_name = info["topic_name"]
        self.validate_md5(info, "messages")
        support = self.get_type_support(message_type)
        if not support:
            return
        if not support.encode:
            self.get_logger().warn(f"Message type cannot be sent to Arduino yet: {message_type}")
            return

        buffer_size = int(info.get("buffer_size") or 0)

        def callback(
            msg,
            rosserial_topic_id=topic_id,
            type_support=support,
            ros_topic=topic_name,
            arduino_buffer_size=buffer_size,
        ):
            payload = type_support.encode(self, msg)
            if not self.payload_fits_arduino_buffer(payload, arduino_buffer_size, ros_topic):
                return
            if self.write_packet(rosserial_topic_id, payload):
                self.get_logger().info(f"Forwarded {ros_topic} to rosserial topic_id={rosserial_topic_id}: {msg}")

        self.topic_subscriptions[topic_id] = self.create_subscription(
            support.ros_msg_type, topic_name, callback, 10
        )
        self.get_logger().info(
            f"Created ROS 2 subscription for {topic_name} [{message_type}] "
            f"to rosserial topic_id={topic_id}"
        )

    def setup_service_server_endpoint(self, info, is_response_publisher):
        topic_name = info["topic_name"]
        self.validate_md5(info, "services")
        support = self.get_service_support(info["message_type"])
        if not support:
            return

        state = self.service_server_endpoints.setdefault(topic_name, {"support": support})
        state["support"] = support
        if is_response_publisher:
            state["response_id"] = info["topic_id"]
        else:
            state["request_id"] = info["topic_id"]
            state["request_buffer_size"] = int(info.get("buffer_size") or 0)

        if "service" in state or "request_id" not in state or "response_id" not in state:
            return

        def callback(request, response, service_state=state, service_name=topic_name):
            payload = service_state["support"].encode_request(self, request)
            if not self.payload_fits_arduino_buffer(
                payload,
                service_state.get("request_buffer_size", 0),
                service_name,
            ):
                if hasattr(response, "success"):
                    response.success = False
                if hasattr(response, "message"):
                    response.message = "rosserial request exceeds Arduino buffer"
                return response
            if not self.write_packet(service_state["request_id"], payload):
                if hasattr(response, "success"):
                    response.success = False
                if hasattr(response, "message"):
                    response.message = "rosserial transport is disconnected"
                return response
            self.get_logger().info(
                f"Forwarded ROS 2 service request {service_name} "
                f"to rosserial topic_id={service_state['request_id']}: {request}"
            )

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    packet = read_packet(self.serial, 0.05, self.max_payload_size, self.packet_stats)
                except (OSError, serial.SerialException) as exc:
                    self.close_transport(exc)
                    break
                if packet is None:
                    continue
                response_topic_id, response_payload = packet
                if response_topic_id == service_state["response_id"]:
                    rosserial_response = service_state["support"].decode_response(self, response_payload)
                    for field_name in rosserial_response.get_fields_and_field_types().keys():
                        setattr(response, field_name, getattr(rosserial_response, field_name))
                    self.get_logger().info(f"Returned ROS 2 service response {service_name}: {response}")
                    return response
                self.handle_packet(response_topic_id, response_payload)

            self.get_logger().warn(f"Timed out waiting for rosserial service response from {service_name}")
            if hasattr(response, "success"):
                response.success = False
            if hasattr(response, "message"):
                response.message = "rosserial service response timeout"
            return response

        state["service"] = self.create_service(support.ros_srv_type, topic_name, callback)
        self.get_logger().info(
            f"Created ROS 2 service server for Arduino service {topic_name} "
            f"[{info['message_type']}] request_id={state['request_id']} response_id={state['response_id']}"
        )

    def setup_service_client_endpoint(self, info, is_request_publisher):
        topic_name = info["topic_name"]
        self.validate_md5(info, "services")
        support = self.get_service_support(info["message_type"])
        if not support:
            return

        state = self.service_client_endpoints.setdefault(topic_name, {"support": support})
        state["support"] = support
        if is_request_publisher:
            state["request_id"] = info["topic_id"]
        else:
            state["response_id"] = info["topic_id"]
            state["response_buffer_size"] = int(info.get("buffer_size") or 0)

        if "request_id" not in state or "response_id" not in state:
            return
        if "client" not in state:
            state["client"] = self.create_client(support.ros_srv_type, topic_name)
            self.get_logger().info(
                f"Created ROS 2 service client for Arduino client {topic_name} "
                f"[{info['message_type']}] request_id={state['request_id']} response_id={state['response_id']}"
            )

        self.service_clients[state["request_id"]] = {
            "client": state["client"],
            "support": support,
            "response_id": state["response_id"],
            "response_buffer_size": state.get("response_buffer_size", 0),
            "service_name": topic_name,
        }

    def make_failed_service_response(self, support, message):
        response = support.ros_srv_type.Response()
        if hasattr(response, "success"):
            response.success = False
        if hasattr(response, "message"):
            response.message = message
        return response

    def handle_service_client_request(self, topic_id, payload):
        state = self.service_clients[topic_id]
        support = state["support"]
        client = state["client"]
        service_name = state["service_name"]
        request = support.decode_request(self, payload)

        if not client.service_is_ready():
            self.get_logger().warn(f"ROS 2 service {service_name} is not ready")
            response = self.make_failed_service_response(support, "ROS 2 service is not ready")
            response_payload = support.encode_response(self, response)
            if self.payload_fits_arduino_buffer(
                response_payload,
                state.get("response_buffer_size", 0),
                service_name,
            ):
                self.write_packet(state["response_id"], response_payload)
            return

        future = client.call_async(request)

        def done_callback(done_future, client_state=state):
            try:
                response = done_future.result()
            except Exception as exc:
                self.get_logger().warn(f"ROS 2 service call {service_name} failed: {exc}")
                response = self.make_failed_service_response(support, str(exc))
            response_payload = support.encode_response(self, response)
            if not self.payload_fits_arduino_buffer(
                response_payload,
                client_state.get("response_buffer_size", 0),
                service_name,
            ):
                return
            if self.write_packet(client_state["response_id"], response_payload):
                self.get_logger().info(
                    f"Forwarded ROS 2 service response {service_name} "
                    f"to rosserial topic_id={client_state['response_id']}: {response}"
                )

        future.add_done_callback(done_callback)
        self.get_logger().info(f"Forwarded rosserial service request {service_name} to ROS 2: {request}")

    def poll_serial(self):
        now = time.monotonic()
        if not self.serial:
            if now >= self.next_reconnect:
                self.open_transport()
            return

        if now >= self.next_stats_log:
            self.log_packet_stats()
            self.next_stats_log = now + self.stats_log_interval

        if not self.known_topics and now >= self.next_topic_request:
            self.request_topics()

        try:
            packet = read_packet(self.serial, 0.1, self.max_payload_size, self.packet_stats)
        except (OSError, serial.SerialException) as exc:
            self.close_transport(exc)
            return
        if packet is None:
            return

        try:
            self.handle_packet(*packet)
        except (struct.error, UnicodeDecodeError, ValueError, IndexError) as exc:
            self.packet_stats["decode_errors"] = self.packet_stats.get("decode_errors", 0) + 1
            self.get_logger().warn(f"Dropped malformed rosserial payload: {exc}")

    def log_packet_stats(self):
        error_keys = [
            "bytes_discarded",
            "bad_protocol",
            "bad_size_checksum",
            "bad_message_checksum",
            "oversized_payload",
            "oversized_to_arduino",
            "incomplete_packets",
            "decode_errors",
        ]
        error_snapshot = {key: self.packet_stats.get(key, 0) for key in error_keys}
        if not any(error_snapshot.values()):
            return
        if error_snapshot == self.last_logged_packet_error_stats:
            return
        self.last_logged_packet_error_stats = error_snapshot
        summary = ", ".join(f"{key}={self.packet_stats.get(key, 0)}" for key in sorted(self.packet_stats))
        self.get_logger().warn(f"rosserial packet stats: {summary}")

    def publish_diagnostics(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(self.make_bridge_diagnostic_status())
        self.diagnostics_publisher.publish(array)

    def make_bridge_diagnostic_status(self):
        status = DiagnosticStatus()
        status.name = "rosserial_bridge"
        status.hardware_id = str(self.port)

        error_keys = [
            "bytes_discarded",
            "bad_protocol",
            "bad_size_checksum",
            "bad_message_checksum",
            "oversized_payload",
            "oversized_to_arduino",
            "incomplete_packets",
            "decode_errors",
        ]
        error_count = sum(int(self.packet_stats.get(key, 0)) for key in error_keys)

        if not self.serial:
            status.level = DiagnosticStatus.WARN
            status.message = "transport disconnected"
        elif error_count:
            status.level = DiagnosticStatus.WARN
            status.message = "connected with packet errors"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "connected"

        values = {
            "port": self.port,
            "baud": self.baud,
            "connected": bool(self.serial),
            "known_topics": len(self.known_topics),
            "publishers_from_arduino": len(self.topic_publishers),
            "subscriptions_to_arduino": len(self.topic_subscriptions),
            "service_servers": len([state for state in self.service_server_endpoints.values() if "service" in state]),
            "service_clients": len(self.service_clients),
            "last_disconnect_reason": self.last_disconnect_reason,
        }
        values.update({key: self.packet_stats.get(key, 0) for key in sorted(self.packet_stats)})

        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        return status

    def handle_packet(self, topic_id, payload):
        if topic_id == ID_TIME:
            self.send_time()
            return
        if topic_id == ID_LOG:
            self.handle_log(payload)
            return
        if topic_id == ID_PARAMETER_REQUEST:
            self.handle_parameter_request(payload)
            return

        if topic_id == ID_PUBLISHER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_publisher(info)
            self.get_logger().info(
                "TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return
        if topic_id == ID_SUBSCRIBER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_subscription(info)
            self.get_logger().info(
                "Subscriber TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return
        if topic_id == ID_SERVICE_SERVER + ID_PUBLISHER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_service_server_endpoint(info, is_response_publisher=True)
            self.get_logger().info(
                "ServiceServer response TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return
        if topic_id == ID_SERVICE_SERVER + ID_SUBSCRIBER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_service_server_endpoint(info, is_response_publisher=False)
            self.get_logger().info(
                "ServiceServer request TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return
        if topic_id == ID_SERVICE_CLIENT + ID_PUBLISHER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_service_client_endpoint(info, is_request_publisher=True)
            self.get_logger().info(
                "ServiceClient request TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return
        if topic_id == ID_SERVICE_CLIENT + ID_SUBSCRIBER:
            info = decode_topic_info(payload)
            self.known_topics[info["topic_id"]] = info
            self.setup_service_client_endpoint(info, is_request_publisher=False)
            self.get_logger().info(
                "ServiceClient response TopicInfo: "
                f"id={info['topic_id']} "
                f"name={info['topic_name']} "
                f"type={info['message_type']} "
                f"buffer={info['buffer_size']}"
            )
            return

        if topic_id in self.service_clients:
            self.handle_service_client_request(topic_id, payload)
            return

        info = self.known_topics.get(topic_id)
        if not info:
            self.get_logger().debug(f"Ignoring unknown topic_id={topic_id}")
            return

        publisher_state = self.topic_publishers.get(topic_id)
        if not publisher_state:
            return

        msg = publisher_state["decode"](self, payload)
        publisher_state["publisher"].publish(msg)
        self.get_logger().debug(f"Published {info['topic_name']}: {msg}")


def main():
    argv = sys.argv
    non_ros_args = remove_ros_args(args=argv)
    parser = argparse.ArgumentParser(
        description="ROS 2 bridge for Arduino sketches that use the rosserial protocol."
    )
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=None)
    args = parser.parse_args(non_ros_args[1:])

    rclpy.init(args=argv)
    node = RosserialBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
