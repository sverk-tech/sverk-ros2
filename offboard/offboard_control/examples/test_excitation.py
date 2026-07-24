#!/usr/bin/env python3
"""
SITL smoke-test for the autonomous excitation executor.

Takes off, sends a small ROLL DOUBLET excitation in ATTITUDE mode (Pattern A),
prints live feedback, checks that the run succeeded and the drone returned close
to its start point, then lands.

WARNING Run in SITL ONLY first (then a tethered stand, then indoors with a geofence).
   The excitation actively perturbs a flying vehicle.

Run (from the workspace, after sourcing install/setup.bash, with the
offboard_control node and SITL running):
  python3 src/sverk_drone/offboard/offboard_control/examples/test_excitation.py
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from offboard_interfaces.action import RunExcitation
from offboard_interfaces.srv import Navigate, StopExcitation
from std_srvs.srv import Trigger

SAFETY_PHRASE = "I_UNDERSTAND_WHAT_I_AM_DOING"
TAKEOFF_HEIGHT = 1.8      # m; must clear excite_agl_floor (default 1.0)
WAIT_TAKEOFF = 40         # s
DRIFT_LIMIT = 1.0         # m; result.max_drift_m must stay below this


class ExcitationTester(Node):
    def __init__(self):
        super().__init__("excitation_tester")
        self._navigate = self.create_client(Navigate, "/navigate")
        self._land = self.create_client(Trigger, "/land")
        self._stop = self.create_client(StopExcitation, "/stop_excitation")
        self._action = ActionClient(self, RunExcitation, "run_excitation")

        for name, cli in (("navigate", self._navigate), ("land", self._land),
                          ("stop_excitation", self._stop)):
            while not cli.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit(1)
                self.get_logger().info(f"Waiting for {name} service...")
        if not self._action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("run_excitation action server not available")
            sys.exit(1)
        self.get_logger().info("All interfaces ready")

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def takeoff(self):
        req = Navigate.Request()
        req.x, req.y, req.z, req.yaw = 0.0, 0.0, float(TAKEOFF_HEIGHT), 0.0
        req.speed = 0.4
        req.auto_arm = True
        req.frame_id = "body"
        resp = self._call(self._navigate, req, timeout=15.0)
        ok = bool(resp and resp.success)
        self.get_logger().info(f"Takeoff requested: success={ok}; climbing ~{WAIT_TAKEOFF}s")
        if ok:
            time.sleep(WAIT_TAKEOFF)
        return ok

    def land(self):
        self._call(self._land, Trigger.Request(), timeout=10.0)

    def _feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f"[{fb.phase:>10}] t={fb.elapsed:5.1f}s prog={fb.progress:4.2f} "
            f"ref={fb.current_ref:+.3f} rate={fb.current_rate:+.3f} "
            f"tilt={fb.current_tilt_deg:4.1f}deg drift={fb.current_drift_m:4.2f}m")

    def run_excitation(self):
        goal = RunExcitation.Goal()
        goal.axis = "roll"
        goal.signal = "doublet"
        goal.loop = RunExcitation.Goal.LOOP_ATTITUDE   # Pattern A
        goal.amplitude = 0.05          # rad (~2.9 deg) — small
        goal.duration = 0.0            # auto from waveform
        goal.f_max = 3.0               # sets doublet half-period
        goal.taper = 0.15
        goal.auto_arm = False          # already armed by takeoff
        # hold_alt=0 → hold the altitude captured at target start. Do NOT pass an
        # absolute AGL (e.g. TAKEOFF_HEIGHT): on a biased estimator that commands the
        # wrong height and has flown a chirp into the ground (P0-3).
        goal.hold_alt = 0.0
        goal.max_tilt_deg = 20.0
        goal.geofence_radius = 1.5
        goal.confirm = SAFETY_PHRASE

        self.get_logger().info("Sending excitation goal (roll doublet, attitude)...")
        send = self._action.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Excitation goal REJECTED")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().error("No result returned")
            return False
        res = wrapped.result
        self.get_logger().info(
            f"RESULT success={res.success} reason='{res.abort_reason}' "
            f"samples={res.samples_injected} rate={res.achieved_rate_hz:.0f}Hz "
            f"max_tilt={res.max_tilt_seen_deg:.1f}deg max_drift={res.max_drift_m:.2f}m "
            f"tag={res.run_tag}")

        ok = bool(res.success) and res.max_drift_m < DRIFT_LIMIT and res.samples_injected > 0
        if not ok:
            self.get_logger().error(
                f"Excitation check FAILED (success={res.success}, "
                f"drift={res.max_drift_m:.2f} < {DRIFT_LIMIT}?)")
        return ok


def main():
    rclpy.init()
    node = ExcitationTester()
    passed = False
    try:
        if not node.takeoff():
            node.get_logger().error("Takeoff failed — aborting test")
        else:
            passed = node.run_excitation()
    finally:
        node.get_logger().info("Landing...")
        node.land()
        node.get_logger().info(f"TEST {'PASSED' if passed else 'FAILED'}")
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
