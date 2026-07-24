"""
Example: flying a circular trajectory using set_position.
"""

import math
import time
import sverk_interfaces

radius = 0.6   # circle radius, m
omega = 0.3    # angular velocity, rad/s
duration = 20  # circle movement duration, s

drone = sverk_interfaces.init(Nodename="example_circle_trajectory")
try:
    # 1. Takeoff to 1.5 m relative to body (frame_id='body'), with auto-arm.
    drone.control.navigate(
        x=0.0,
        y=0.0,
        z=1.5,
        yaw=0.0,
        speed=0.5,
        frame_id="body",
        auto_arm=True
    )
    time.sleep(10.0)

    # Start point and altitude in map frame.
    start = drone.control.get_telemetry(frame_id="map")

    # Takeoff / climb.
    target_z = start.z, 1.0

    start_time = time.monotonic()
    rate = 10.0  # Hz
    dt = 1.0 / rate

    while time.monotonic() - start_time < duration:
        t = time.monotonic() - start_time
        angle = omega * t
        x = start.x + math.sin(angle) * radius
        y = start.y + math.cos(angle) * radius

        drone.control.set_position(
            x=x,
            y=y,
            z=target_z,
            yaw=start.yaw,
            frame_id="map",
            auto_arm=False,
        )

        time.sleep(dt)

    # Return to start point and land.
    drone.control.navigate(
        x=start.x,
        y=start.y,
        z=target_z,
        yaw=start.yaw,
        speed=0.5,
        frame_id="map",
        auto_arm=False,
    )
    time.sleep(5.0)
    drone.control.land(timeout=10.0)
finally:
    drone.close()