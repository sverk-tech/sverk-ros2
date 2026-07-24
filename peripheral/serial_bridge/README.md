# serial_bridge

`serial_bridge` is an `ament_python` package for building ROS 2 nodes that
communicate with Arduino-like devices over serial.

The package provides reusable transport helpers:

* `SerialTransport`
* `SerialTransportConfig`
* `LineProtocol`
* `SerialConnectionError`

In addition, the package now includes an example servo bridge:

* ROS 2 node:
  * `serial_bridge/arduino_servo_node.py`
* launch file:
  * `launch/arduino_servo.launch.py`
* default parameters:
  * `config/arduino_servo.yaml`
* Arduino sketch:
  * `examples/serial_servo_bridge_d2/serial_servo_bridge_d2.ino`

## Example Servo Protocol

The example servo bridge uses a compact byte-oriented protocol that proved more
robust on Arduino Uno than line-based textual commands.

Commands sent from ROS 2 node to Arduino:

* `0..180` — target angle in degrees
* `0xFD` — move to center angle
* `0xFE` — enable servo output
* `0xFF` — disable servo output

The example node is intentionally write-only and publishes the last commanded
state. It does not rely on serial feedback from the Arduino sketch.

## Typical Usage

Run the example node:

```bash
ros2 launch serial_bridge arduino_servo.launch.py
```

Send a target angle:

```bash
ros2 param set /arduino_servo_control command_angle_deg 20.0
ros2 param set /arduino_servo_control command_angle_deg 160.0
ros2 param set /arduino_servo_control command_angle_deg 90.0
```

Or publish to the target topic:

```bash
ros2 topic pub --once /arduino_servo_control/target_angle_deg std_msgs/msg/Float32 "{data: 20.0}"
```
