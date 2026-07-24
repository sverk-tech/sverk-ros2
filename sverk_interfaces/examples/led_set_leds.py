"""
Example of low-level LED control: setting color by index
and reading the strip state from the /led_control/state topic.

Requires the led_control node to be running (ros2 launch led_control led.launch.py)
and the led_interfaces package in the workspace.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_led_set_leds")
try:
    if not drone.led:
        print("LED API unavailable: install led_interfaces and start led_control.")
        exit(1)

    # Option 1: list of tuples (index, r, g, b)
    print("Setting first 5 LEDs: red, green, blue, yellow, white...")
    drone.led.set_leds([
        (0, 255, 0, 0),
        (1, 0, 255, 0),
        (2, 0, 0, 255),
        (3, 255, 255, 0),
        (4, 255, 255, 255),
    ])
    time.sleep(2.0)

    # Option 2: list of dictionaries
    print("Changing colors by index using dictionaries...")
    drone.led.set_leds([
        {"index": 0, "r": 255, "g": 100, "b": 100},
        {"index": 1, "r": 100, "g": 255, "b": 100},
        {"index": 2, "r": 100, "g": 100, "b": 255},
    ])
    time.sleep(2.0)

    # Reading the current strip state from the /led_control/state topic
    print("Reading strip state (once from /led_control/state)...")
    state = drone.led.get_state(timeout=3.0)
    if state:
        print(f"  Total LEDs: {len(state.leds)}")
        for i, led in enumerate(state.leds[:5]):
            print(f"  LED[{led.index}] R={led.r} G={led.g} B={led.b}")
    else:
        print("  Timeout: message not received.")

    # Return to effect (fill) — after set_leds the node can show effects again
    print("Enabling fill effect blue...")
    drone.led.set_effect("fill", r=0, g=0, b=255)
    time.sleep(1.0)

    print("Done.")
finally:
    drone.close()