#!/usr/bin/env python3
"""
Passive MAVLink monitor for verifying fmu_control commands on PX4 v1.15.4.

It does NOT send anything — it only listens and prints:
  - COMMAND_ACK    (result of arm/disarm/reboot/kill/calibration commands)
  - STATUSTEXT     (calibration progress / errors, e.g. "[cal] calibration started: ... gyro")
  - HEARTBEAT      (armed/disarmed state via base_mode flag)

Use it as a second window while another process (sverk_interfaces drone.fcu.*
or `ros2 service call`) issues the commands.

This drone keeps UART and USB strictly separated:
  - uXRCE-DDS  -> UART  (/dev/ttyAMA0 @ 921600), used by ROS 2 / PX4.
  - MAVLink    -> USB   (/dev/ttyACM0 @ 57600), fanned out by mavlink-routerd
                  to UDP 0.0.0.0:14550 (for QGC and any client).
So this monitor uses the MAVLink UDP endpoint and NEVER touches the DDS UART.

Connection:
  # Default: connect to the mavlink-router UDP endpoint (sends a packet so the
  # router registers us and starts forwarding):
  python3 mav_monitor.py --conn udpout:127.0.0.1:14550
  # From another host on the network, replace 127.0.0.1 with the Pi's IP.

  # Direct USB serial (only if mavlink-router is NOT running and you own ACM0):
  python3 mav_monitor.py --conn /dev/ttyACM0 --baud 57600

Requires: pip install pymavlink
"""
import argparse
import time

from pymavlink import mavutil

CMD_NAMES = {
    241: "PREFLIGHT_CALIBRATION",
    400: "COMPONENT_ARM_DISARM",
    246: "PREFLIGHT_REBOOT_SHUTDOWN",
    185: "DO_FLIGHTTERMINATION",
}
ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
}
MAV_MODE_FLAG_SAFETY_ARMED = 128


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpout:127.0.0.1:14550",
                    help="pymavlink connection string (mavlink-router UDP endpoint)")
    ap.add_argument("--baud", type=int, default=57600,
                    help="baud rate (serial connections only, e.g. /dev/ttyACM0)")
    args = ap.parse_args()

    print(f"Connecting to {args.conn} ...")
    master = mavutil.mavlink_connection(args.conn, baud=args.baud)
    master.wait_heartbeat(timeout=15)
    print(f"Heartbeat from system {master.target_system} "
          f"component {master.target_component}. Listening...\n")

    last_armed = None
    while True:
        msg = master.recv_match(
            type=["COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"],
            blocking=True, timeout=1.0,
        )
        if msg is None:
            continue
        t = time.strftime("%H:%M:%S")
        mtype = msg.get_type()

        if mtype == "COMMAND_ACK":
            cmd = CMD_NAMES.get(msg.command, str(msg.command))
            res = ACK_RESULTS.get(msg.result, str(msg.result))
            print(f"[{t}] ACK   {cmd:<24} -> {res}")

        elif mtype == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            print(f"[{t}] TEXT  {text}")

        elif mtype == "HEARTBEAT":
            armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != last_armed:
                print(f"[{t}] STATE armed={armed}")
                last_armed = armed


if __name__ == "__main__":
    main()
