#!/usr/bin/env python3
import argparse
import struct
import time
import sys
from pymavlink import mavutil


INTEGER_PARAM_FORMATS = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: "<B",
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: "<b",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: "<H",
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: "<h",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: "<I",
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: "<i",
}


def encode_param_value(value, param_type):
    value_format = INTEGER_PARAM_FORMATS.get(param_type)
    if value_format is None:
        return value
    encoded = struct.pack(value_format, int(value)).ljust(4, b"\0")
    return struct.unpack("<f", encoded)[0]


def decode_param_value(value, param_type):
    value_format = INTEGER_PARAM_FORMATS.get(param_type)
    if value_format is None:
        return value
    encoded = struct.pack("<f", value)
    return struct.unpack(value_format, encoded[:struct.calcsize(value_format)])[0]


def set_px4_parameter(mav, param_name, param_value):
    current = None
    for _ in range(3):
        while mav.recv_match(blocking=False) is not None:
            pass
        mav.mav.param_request_read_send(
            mav.target_system,
            mav.target_component,
            param_name.encode(),
            -1,
        )

        start = time.time()
        while time.time() - start < 5.0:
            msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is not None and msg.param_id == param_name:
                current = msg
                break
        if current is not None:
            break

    if current is None:
        print(f"Warning: could not read {param_name}", file=sys.stderr)
        return False

    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        param_name.encode(),
        encode_param_value(param_value, current.param_type),
        current.param_type,
    )

    start = time.time()
    while time.time() - start < 10.0:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is not None and msg.param_id == param_name:
            decoded_value = decode_param_value(msg.param_value, msg.param_type)
            print(f"{param_name} = {decoded_value} (set OK)")
            return True

    print(f"Warning: timeout waiting for {param_name} acknowledgement", file=sys.stderr)
    return False


def handoff_gps_to_vio(mav):
    import rclpy
    from px4_msgs.msg import EstimatorStatusFlags
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("px4_sitl_vio_handoff")
    state = {"ready_count": 0, "complete_count": 0}

    def status_callback(msg):
        vio_ready = msg.cs_ev_pos and msg.cs_ev_yaw and msg.cs_rng_hgt
        state["ready_count"] = state["ready_count"] + 1 if vio_ready else 0
        handoff_complete = vio_ready and not msg.cs_gps
        state["complete_count"] = state["complete_count"] + 1 if handoff_complete else 0

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    node.create_subscription(
        EstimatorStatusFlags,
        "/fmu/out/estimator_status_flags",
        status_callback,
        qos,
    )

    try:
        deadline = time.time() + 120.0
        while time.time() < deadline and state["ready_count"] < 10:
            rclpy.spin_once(node, timeout_sec=0.2)
        if state["ready_count"] < 10:
            print("Warning: timed out waiting for stable ArUco/range fusion", file=sys.stderr)
            return False

        print("Stable ArUco XY/yaw and range-height fusion detected; disabling GPS")
        if not set_px4_parameter(mav, "EKF2_GPS_CTRL", 0):
            return False

        deadline = time.time() + 30.0
        while time.time() < deadline and state["complete_count"] < 10:
            rclpy.spin_once(node, timeout_sec=0.2)
        if state["complete_count"] < 10:
            print("Warning: GPS-to-ArUco handoff did not stabilize", file=sys.stderr)
            return False

        print("GPS-to-ArUco handoff complete")
        return True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Set PX4 parameter via MAVLink in SITL")
    parser.add_argument("--param", required=True, action="append", help="PX4 parameter name")
    parser.add_argument("--value", required=True, action="append", type=float, help="PX4 parameter value")
    parser.add_argument(
        "--handoff-gps-after-vio",
        action="store_true",
        help="disable GPS after stable ArUco XY/yaw and range-height fusion",
    )
    args = parser.parse_args()

    if len(args.param) != len(args.value):
        parser.error("each --param must have a matching --value")

    mav = mavutil.mavlink_connection("udp:127.0.0.1:14540")
    mav.wait_heartbeat(timeout=30)
    print(f"Connected to PX4 (system {mav.target_system}, component {mav.target_component})")

    success = True
    for param_name, param_value in zip(args.param, args.value):
        if not set_px4_parameter(mav, param_name, param_value):
            success = False

    if success and args.handoff_gps_after_vio:
        success = handoff_gps_to_vio(mav)

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
