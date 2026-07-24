"""
Safety and calibration via drone.fcu (all over uXRCE-DDS, no MAVLink):
- arm / normal disarm / force disarm / kill switch / reboot;
- "still" calibrations exposed here: gyro, level.

The high-level API only wraps the calibrations that finish WITHOUT moving the
vehicle (gyro, level). Interactive ones (mag/accel — rotate/place by prompts) and
ESC are driven through the fmu_control package directly:
  action  /fmu_control/calibrate   (streams [cal] progress; ESC needs a phrase)
  topic   /fmu_control/request_calibration

WARNING: arm / force_disarm / kill_switch change motor state immediately.
Run only with propellers removed / drone secured.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_safety_and_calibration")
try:
    print("Arm (props off!)...")
    resp = drone.fcu.arm()
    print("arm:", resp.success, resp.message)

    time.sleep(2.0)

    print("Normal disarm (if drone is on ground)...")
    resp = drone.fcu.disarm()
    print("disarm:", resp.success, resp.message)

    time.sleep(2.0)

    print("Force disarm (ignores 'not landed')...")
    resp = drone.fcu.force_disarm()
    print("force_disarm:", resp.success, resp.message)

    # "Still" calibrations (must be pre-arm, drone not in flight).
    print("Gyroscope calibration (hold still)...")
    drone.fcu.calibrate_gyro()
    time.sleep(1.0)

    print("Level horizon calibration (place level)...")
    drone.fcu.calibrate_level()
    time.sleep(1.0)

    # Interactive/ESC calibrations are NOT in drone.fcu — use fmu_control:
    #   ros2 action send_goal --feedback /fmu_control/calibrate \
    #     offboard_interfaces/action/Calibrate '{sensor: 3, confirm: ""}'   # accel
    #   ESC needs confirm: "I_UNDERSTAND_WHAT_I_AM_DOING"

    print("Kill switch (needs CBRK_FLIGHTTERM=0):  drone.fcu.kill_switch()")
    print("Reboot (only when disarmed):            drone.fcu.reboot()")
finally:
    drone.close()
