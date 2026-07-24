"""
Example: the offboard hardening-block APIs in one flight.

Sequence:
1. takeoff via navigate_wait (rides the /navigate_to action, arrival tracked
   onboard);
2. navigate_to with live feedback (distance_remaining / eta / phase);
3. navigate_to with a short timeout to an unreachable point — the node parks
   the drone to hold and the call reports success=False ("Timeout ...");
4. stream_telemetry — onboard-paced telemetry samples;
5. get_status — node diagnostics (active maneuver, last abort reason);
6. emergency_stop while a navigate is in progress — instant hold;
7. return_to_launch — fly home and land (PX4 AUTO.RTL).

Run against SITL (or a real drone in a large safe area — step 3 flies 30 m!).
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_offboard_hardening")
try:
    # 1. Takeoff: navigate_wait returns when the drone actually got there.
    print("1. takeoff to 2 m ...")
    res = drone.control.navigate_wait(
        x=0.0, y=0.0, z=2.0, yaw=0.0, speed=0.8,
        frame_id="body", auto_arm=True, timeout=60.0, tolerance=0.3,
    )
    print(f"   arrived: {res.message}, error {res.final_distance_error:.2f} m")

    # 2. navigate_to with feedback — watch the approach live.
    print("2. navigate_to (1.5, 1.0, 2.0) with feedback ...")

    def on_progress(fb):
        print(f"   [{fb.phase}] remaining {fb.distance_remaining:.2f} m, "
              f"eta {fb.eta:.1f} s")

    # tolerance=0.2: on a vision-fused stack (VPE/ArUco) the pose noise can keep
    # the drone hovering just outside the fine 0.08 m arrival tolerance forever —
    # a realistic mission radius avoids running into the goal timeout.
    res = drone.control.navigate_to(
        x=1.5, y=1.0, z=2.0, yaw=0.0, speed=0.6,
        frame_id="map", tolerance=0.2, feedback=on_progress,
    )
    print(f"   result: success={res.success}, err={res.final_distance_error:.2f} m")

    # 3. Unreachable target + tiny timeout: the action still finishes,
    #    the node parks the drone to hold.
    print("3. navigate_to far away with timeout=4 s (expected to fail) ...")
    res = drone.control.navigate_to(
        x=30.0, y=0.0, z=2.0, speed=0.6, frame_id="map", timeout=4.0,
    )
    print(f"   result: success={res.success} — {res.message}")

    # 4. Telemetry stream at the node's pace.
    print("4. stream_telemetry 10 Hz for 2 s ...")

    def on_sample(t):
        print(f"   z={t.z:.2f} vz={t.vz:.2f} mode={t.mode}")

    res = drone.control.stream_telemetry(on_sample, rate_hz=10.0, duration=2.0)
    print(f"   samples_sent={res.samples_sent}")

    # 5. Node diagnostics.
    s = drone.control.get_status()
    if s is not None:
        print(f"5. status: setpoint={s.setpoint_type} maneuver='{s.active_maneuver}' "
              f"last_abort='{s.last_abort_reason}' battery={s.battery_pct:.0f}%")

    # 6. emergency_stop chops a maneuver mid-flight.
    print("6. navigate + emergency_stop after 1.5 s ...")
    drone.control.navigate(x=-2.0, y=-2.0, z=2.0, speed=0.6, frame_id="map")
    time.sleep(1.5)
    res = drone.control.emergency_stop()   # land=False -> hold in place
    print(f"   emergency_stop: {res.message}")
    time.sleep(2.0)

    # 7. Return to launch: fly home and land there.
    print("7. return_to_launch ...")
    res = drone.control.return_to_launch()
    print(f"   rtl: {res.message}")
finally:
    drone.close()
