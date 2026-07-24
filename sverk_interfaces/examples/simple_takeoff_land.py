"""
Simple example: takeoff, fly 1 m forward in body frame, land.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_simple_takeoff_land")
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

    # 2. Fly 1 m forward in body frame (body X axis).
    drone.control.navigate(
        x=1.0,
        y=0.0,
        z=0.0,
        yaw=0.0,
        speed=0.5,
        frame_id="body",
        auto_arm=False
    )

    # Short pause in the air.
    time.sleep(5.0)

    # 3. Land.
    land_resp = drone.control.land(timeout=10.0)
    print("land:", land_resp.success, land_resp.message)
finally:
    drone.close()