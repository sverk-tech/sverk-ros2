#!/usr/bin/env python3
"""
SITL smoke test for the offboard hardening block (prompts 91-104):
  arm -> navigate -> get_telemetry -> NavigateTo action (feedback) ->
  ReceiveTelemetry stream -> emergency_stop (hold) -> land.

Checks that the new interfaces work and the existing services show no
regressions. Run against a live SITL with the offboard_control node up:

  python3 src/sverk_drone/offboard/offboard_control/examples/test_offboard_hardening.py
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from offboard_interfaces.action import NavigateTo, ReceiveTelemetry
from offboard_interfaces.srv import EmergencyStop, GetTelemetry, Navigate
from std_srvs.srv import Trigger

TAKEOFF_HEIGHT = 2.5
NAVIGATE_SPEED = 0.6
SERVICE_TIMEOUT = 10.0
TAKEOFF_WAIT_S = 40
NAV_ACTION_TIMEOUT_S = 60


class HardeningTester(Node):
    def __init__(self):
        super().__init__("offboard_hardening_tester")
        self.navigate_cli = self.create_client(Navigate, "/navigate")
        self.telemetry_cli = self.create_client(GetTelemetry, "/get_telemetry")
        self.emergency_cli = self.create_client(EmergencyStop, "/emergency_stop")
        self.land_cli = self.create_client(Trigger, "/land")
        self.navigate_to_ac = ActionClient(self, NavigateTo, "/navigate_to")
        self.receive_telemetry_ac = ActionClient(self, ReceiveTelemetry, "/receive_telemetry")
        self.failures = []

    def wait_ready(self):
        for name, cli in [("navigate", self.navigate_cli), ("get_telemetry", self.telemetry_cli),
                          ("emergency_stop", self.emergency_cli), ("land", self.land_cli)]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"waiting for /{name} ...")
                if not rclpy.ok():
                    sys.exit(1)
        for name, ac in [("navigate_to", self.navigate_to_ac),
                         ("receive_telemetry", self.receive_telemetry_ac)]:
            while not ac.wait_for_server(timeout_sec=2.0):
                self.get_logger().info(f"waiting for action /{name} ...")
                if not rclpy.ok():
                    sys.exit(1)
        self.get_logger().info("all interfaces ready")

    def call(self, cli, request, tag):
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)
        if not future.done():
            self.failures.append(f"{tag}: service call timed out")
            return None
        return future.result()

    def check(self, ok, tag, detail=""):
        if ok:
            self.get_logger().info(f"[PASS] {tag} {detail}")
        else:
            self.get_logger().error(f"[FAIL] {tag} {detail}")
            self.failures.append(f"{tag} {detail}")

    # --- steps ---

    def step_takeoff(self):
        req = Navigate.Request()
        req.x, req.y, req.z = 0.0, 0.0, TAKEOFF_HEIGHT
        req.yaw = 0.0
        req.speed = NAVIGATE_SPEED
        req.frame_id = "body"
        req.auto_arm = True
        res = self.call(self.navigate_cli, req, "navigate(auto_arm)")
        self.check(res is not None and res.success, "navigate(auto_arm)",
                   res.message if res else "")
        # Wait until we are near the takeoff altitude.
        deadline = time.time() + TAKEOFF_WAIT_S
        while time.time() < deadline:
            t = self.call(self.telemetry_cli, GetTelemetry.Request(), "get_telemetry")
            if t and t.connected and t.armed and not math.isnan(t.z) and t.z > TAKEOFF_HEIGHT * 0.8:
                self.check(True, "takeoff altitude", f"z={t.z:.2f}")
                return True
            time.sleep(1.0)
        self.check(False, "takeoff altitude", "timeout")
        return False

    def step_get_telemetry(self):
        t = self.call(self.telemetry_cli, GetTelemetry.Request(), "get_telemetry")
        ok = t is not None and t.connected and t.frame_id == "map" and not math.isnan(t.x)
        self.check(ok, "get_telemetry", f"x={getattr(t, 'x', float('nan')):.2f}" if t else "")

    def step_navigate_to(self):
        goal = NavigateTo.Goal()
        goal.x, goal.y, goal.z = 1.5, 1.0, TAKEOFF_HEIGHT
        goal.yaw = 0.0
        goal.speed = NAVIGATE_SPEED
        goal.frame_id = "map"
        goal.auto_arm = False
        feedback_counter = {"n": 0, "last": float("inf")}

        def on_feedback(msg):
            feedback_counter["n"] += 1
            feedback_counter["last"] = msg.feedback.distance_remaining

        send_future = self.navigate_to_ac.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SERVICE_TIMEOUT)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.check(False, "navigate_to goal accepted")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=NAV_ACTION_TIMEOUT_S)
        if not result_future.done():
            self.check(False, "navigate_to result", "timeout")
            return
        result = result_future.result().result
        self.check(result.success, "navigate_to result",
                   f"err={result.final_distance_error:.2f} m, feedback={feedback_counter['n']}")
        self.check(feedback_counter["n"] > 0, "navigate_to feedback flowed")

    def step_navigate_to_timeout(self):
        # Unreachable in time: far target, tiny timeout -> the action must abort
        # with success=false and the drone must park (hold), not fly on forever.
        goal = NavigateTo.Goal()
        goal.x, goal.y, goal.z = 30.0, 0.0, TAKEOFF_HEIGHT
        goal.yaw = 0.0
        goal.speed = NAVIGATE_SPEED
        goal.frame_id = "map"
        goal.auto_arm = False
        goal.timeout = 4.0

        send_future = self.navigate_to_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SERVICE_TIMEOUT)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.check(False, "navigate_to(timeout) goal accepted")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        if not result_future.done():
            self.check(False, "navigate_to(timeout) result", "no result — action hung")
            return
        result = result_future.result().result
        ok = (not result.success) and ("Timeout" in result.message)
        self.check(ok, "navigate_to timeout aborts",
                   f"success={result.success}, msg='{result.message}', err={result.final_distance_error:.1f} m")
        time.sleep(2.0)  # let the hold settle before the next step

    def step_receive_telemetry(self):
        goal = ReceiveTelemetry.Goal()
        goal.rate_hz = 10.0
        goal.frame_id = "map"
        goal.duration = 2.0
        samples = {"n": 0}

        def on_feedback(msg):
            samples["n"] += 1

        send_future = self.receive_telemetry_ac.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SERVICE_TIMEOUT)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.check(False, "receive_telemetry goal accepted")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        if not result_future.done():
            self.check(False, "receive_telemetry result", "timeout")
            return
        result = result_future.result().result
        # ~10 Hz x 2 s: expect a stream, allow generous slack for SITL jitter.
        self.check(result.success and result.samples_sent >= 10, "receive_telemetry stream",
                   f"samples_sent={result.samples_sent}, feedback={samples['n']}")

    def step_emergency_stop_hold(self):
        # Start a maneuver, then chop it with emergency_stop(land=false).
        req = Navigate.Request()
        req.x, req.y, req.z = -1.5, -1.0, TAKEOFF_HEIGHT
        req.yaw = 0.0
        req.speed = NAVIGATE_SPEED
        req.frame_id = "map"
        req.auto_arm = False
        res = self.call(self.navigate_cli, req, "navigate(for e-stop)")
        self.check(res is not None and res.success, "navigate(for e-stop)",
                   res.message if res else "")
        time.sleep(1.0)
        estop = EmergencyStop.Request()
        estop.land = False
        res = self.call(self.emergency_cli, estop, "emergency_stop(hold)")
        self.check(res is not None and res.success, "emergency_stop(hold)",
                   res.message if res else "")
        # The drone should hold: altitude stays near the takeoff height.
        time.sleep(3.0)
        t = self.call(self.telemetry_cli, GetTelemetry.Request(), "get_telemetry(post-stop)")
        ok = t is not None and t.armed and not math.isnan(t.z) and abs(t.z - TAKEOFF_HEIGHT) < 1.0
        self.check(ok, "hold after emergency_stop",
                   f"z={getattr(t, 'z', float('nan')):.2f}" if t else "")

    def step_land(self):
        res = self.call(self.land_cli, Trigger.Request(), "land")
        self.check(res is not None and res.success, "land", res.message if res else "")


def main():
    rclpy.init()
    tester = HardeningTester()
    tester.wait_ready()
    try:
        if tester.step_takeoff():
            tester.step_get_telemetry()
            tester.step_navigate_to()
            tester.step_navigate_to_timeout()
            tester.step_receive_telemetry()
            tester.step_emergency_stop_hold()
        tester.step_land()
    finally:
        if tester.failures:
            tester.get_logger().error(f"FAILURES ({len(tester.failures)}):")
            for f in tester.failures:
                tester.get_logger().error(f"  - {f}")
        else:
            tester.get_logger().info("ALL CHECKS PASSED")
        code = 1 if tester.failures else 0
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(code)


if __name__ == "__main__":
    main()
