"""
Example: flying a "cube" in the body frame.

Figure:
- takeoff upward;
- square forward/right/back/left in body frame;
- climb another level and repeat the square (second "shelf" of the cube);
- return to start point in body frame and land.

navigate_wait rides the /navigate_to action: arrival is tracked onboard by
offboard_control, an unreachable target raises TimeoutError (drone parks).
"""

import time
import sverk_interfaces

side = 1.0   # square side length, m
level = 1.0  # height of one "floor", m

drone = sverk_interfaces.init(Nodename="example_cube_mission")
try:
    # 1. Takeoff to first level height relative to body.
    drone.control.navigate(
        x=0.0,
        y=0.0,
        z=level,
        yaw=0.0,
        speed=0.5,
        frame_id="body",
        auto_arm=True
    )
    time.sleep(5.0)

    # 2. Square on the first level (forward, right, back, left).
    first_level_segments = [
        (side, 0.0, level),   # forward
        (0.0, -side, level),  # right (Y<0 in body)
        (-side, 0.0, level),  # back
        (0.0, side, level),   # left
    ]
    for dx, dy, dz in first_level_segments:
        drone.control.navigate_wait(
            x=dx,
            y=dy,
            z=dz,
            yaw=0.0,
            speed=0.5,
            frame_id="map",
            auto_arm=False,
            tolerance=0.25,
            timeout=60.0,
        )
        time.sleep(0.5)

    # 3. Climb to the second level height.
    drone.control.navigate_wait(
        x=0.0,
        y=0.0,
        z=level*2,
        yaw=0.0,
        speed=0.5,
        frame_id="map",
        auto_arm=False,
        tolerance=0.25,
        timeout=60.0,
    )

    # 4. Square on the second level.
    for dx, dy, dz in first_level_segments:
        drone.control.navigate_wait(
            x=dx,
            y=dy,
            z=dz*2,
            yaw=0.0,
            speed=0.5,
            frame_id="map",
            auto_arm=False,
            tolerance=0.25,
            timeout=60.0,
        )
        time.sleep(0.5)

    # 5. Return to original height (descend by level).
    drone.control.navigate_wait(
        x=0.0,
        y=0.0,
        z=level,
        yaw=0.0,
        speed=0.5,
        frame_id="map",
        auto_arm=False,
        tolerance=0.25,
        timeout=60.0,
    )

    # Short pause and land.
    time.sleep(2.0)
    drone.control.land(timeout=10.0)
finally:
    drone.close()