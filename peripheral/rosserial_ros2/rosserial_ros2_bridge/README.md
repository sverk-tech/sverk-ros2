# rosserial_ros2_bridge

ROS 2 bridge for Arduino-class microcontroller sketches that use the rosserial protocol.

The preferred Arduino-side API uses ROS 2-style names while keeping the
original rosserial Arduino API available for compatibility:

- `ros2.h`
- `ros::Node`
- `ros::Publisher`
- `ros::Subscription`
- `ros::Service`
- `ros::Client`
- `node.now()`, `node.log*()`, and `node.get_parameter()`
- generated-style `ros_lib` message headers

The host side is ROS 2-native and uses `rclpy`.

Legacy `ros.h`, `ros::NodeHandle`, `ros::Subscriber`,
`ros::ServiceServer`, `ros::ServiceClient`, `advertise()`, `subscribe()`,
`spinOnce()`, and `getParam()` still work.

## Arduino API

New sketches should start with:

```cpp
#include <ros2.h>
#include <std_msgs/String.h>

ros::Node node;

std_msgs::String msg;
ros::Publisher pub("status", &msg);

void setup()
{
  node.init();
  node.create_publisher(pub);
}

void loop()
{
  node.spin_some();

  if (!node.is_connected())
    return;

  msg.data = (char*)"hello";
  pub.publish();
}
```

Use `ros::Subscription<T>`, `ros::Service<Request, Response>`, and
`ros::Client<Request, Response>` for subscriptions and services.

## Arduino transport selection

By default, `ros2.h` uses `SerialHardware`, which sends rosserial bytes through
the board's Arduino `Serial` object:

```cpp
#include <ros2.h>

ros::Node node;
```

The old `ArduinoHardware` name is still available as a compatibility alias, but
new sketches should prefer `SerialHardware` when explicitly customizing buffer
sizes:

```cpp
#include <ros2.h>

ros::Node_<SerialHardware, 4, 4, 128, 128> node;
```

Future transports can be selected at compile time before including `ros2.h`:

```cpp
#define ROSSERIAL_HARDWARE_HEADER "MyTcpHardware.h"
#define ROSSERIAL_HARDWARE_CLASS MyTcpHardware

#include <ros2.h>
```

The selected hardware class must provide the same small stream interface:
`init()`, `read()`, `write(uint8_t*, int)`, and `time()`.

## Build

From the project root:

```bash
cd <rosserial_ros2>
colcon build --base-paths rosserial_ros2_bridge --packages-select rosserial_ros2_bridge
source install/setup.bash
```

If `colcon` is missing:

```bash
sudo apt install python3-colcon-common-extensions
```

## Run

Direct serial port:

```bash
ros2 launch rosserial_ros2_bridge bridge.launch.py port:=/dev/ttyUSB0 baud:=57600
```

Windows COM port forwarded to TCP:

```bash
ros2 launch rosserial_ros2_bridge bridge.launch.py port:=socket://<WINDOWS_IP>:7777 baud:=57600
```

The same node can also be run without launch:

```bash
ros2 run rosserial_ros2_bridge bridge_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=57600
```

The bridge keeps retrying when the serial/TCP transport is missing or drops:

```bash
ros2 launch rosserial_ros2_bridge bridge.launch.py \
  port:=/dev/ttyUSB0 \
  baud:=57600 \
  reconnect_delay:=2.0 \
  max_payload_size:=4096
```

Malformed rosserial frames are discarded and counted. If checksum, protocol, or
decode errors occur, the bridge periodically prints packet stats instead of
crashing the node.

The bridge also publishes a standard `diagnostic_msgs/DiagnosticArray` status
on `/rosserial_ros2_bridge/diagnostics` by default. It reports the transport state, configured port,
known rosserial endpoints, and packet parser counters:

```bash
ros2 topic echo /rosserial_ros2_bridge/diagnostics
```

## Generate headers and schemas

Generate Arduino `ros_lib` headers and the bridge runtime schema:

```bash
ros2 run rosserial_ros2_bridge generate_ros_lib \
  --packages std_msgs geometry_msgs sensor_msgs std_srvs
```

Custom message/service packages can be generated from source without installing
them first. Pass either a package directory or a workspace `src` directory:

```bash
ros2 run rosserial_ros2_bridge generate_ros_lib \
  --packages std_msgs geometry_msgs sensor_msgs std_srvs \
  --package-paths ~/my_robot_ws/src
```

The generator scans directories that contain `package.xml` plus `msg/` or
`srv/`, writes Arduino headers to `libraries/ros_lib/<package_name>/`,
and adds runtime schemas to `generated_interfaces.json`. If a custom message
contains nested custom types, pass every package that defines those nested
types with `--package-paths` or `--packages`.

The source scan is only for code generation. At runtime the bridge still needs
the matching ROS 2 message/service package to be built and sourced, because it
imports the Python classes from `<package>.msg` and `<package>.srv`.

By default the generator keeps existing headers and only writes missing files. Use `--overwrite` only when you deliberately want to replace existing generated or hand-written headers.

The generator writes ROS1/rosserial-style MD5 strings into `getMD5()` and into `generated_interfaces.json`. During topic/service negotiation the bridge compares Arduino `TopicInfo.md5sum` with the generated schema and logs a warning if they differ.

Generated variable-length arrays use `realloc()` in the Arduino message
deserializer, matching the classic rosserial style. This keeps headers usable
for strings, nested messages, and custom arrays, but large dynamic arrays are
still a poor fit for small AVR boards like Arduino Nano.

## Estimate buffer sizes

Use `buffer_size` to estimate the payload size, full rosserial frame size, and
a rounded buffer recommendation before choosing `INPUT_SIZE` or `OUTPUT_SIZE`:

```bash
ros2 run rosserial_ros2_bridge buffer_size std_msgs/String data:=32
ros2 run rosserial_ros2_bridge buffer_size geometry_msgs/Twist
ros2 run rosserial_ros2_bridge buffer_size sensor_msgs/LaserScan \
  header.frame_id:=8 ranges:=30 intensities:=0 --buffer 280 --details
```

Dynamic strings and variable-length arrays default to length `0`. Pass field
paths with `:=` to model the largest value expected in your sketch. For service
types, choose the side with `--part request` or `--part response`.

## Services

The bridge currently supports these rosserial services:

- `std_srvs/SetBool`
- `std_srvs/Trigger`

Arduino service server example:

```bash
ros2 service call /set_led std_srvs/srv/SetBool "{data: true}"
```

Arduino service client example:

```bash
ros2 service type /nano_trigger
```

## Arduino logs

Arduino sketches can use the classic rosserial logging API:

```cpp
node.logdebug("debug message");
node.loginfo("sensor initialized");
node.logwarn("sensor timeout");
node.logerror("sensor read failed");
node.logfatal("fatal device error");
```

The bridge handles `rosserial_msgs/TopicInfo::ID_LOG` and forwards messages to
the ROS 2 logger. They appear in the bridge console, `/rosout`, and the normal
ROS 2 log files under `~/.ros/log`.

## Arduino parameters

Arduino sketches can read parameters from the bridge node:

```cpp
int period_ms = 1000;

if (node.get_parameter("period_ms", period_ms)) {
  node.loginfo("parameter loaded");
}
```

The classic array form also works:

```cpp
int values[1];
node.get_parameter("period_ms", values, 1);
```

Pass parameters to the ROS 2 bridge in the usual ROS 2 way:

```bash
ros2 run rosserial_ros2_bridge bridge_node \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=57600 \
  -p period_ms:=250
```

The bridge answers `rosserial_msgs/TopicInfo::ID_PARAMETER_REQUEST` using its
own ROS 2 parameters. Supported values are `bool`, `int`, `float`, `string`, and
homogeneous arrays of those types. Booleans are returned through the rosserial
`ints` array because that is how the Arduino `getParam(bool*)` helper expects
them.
