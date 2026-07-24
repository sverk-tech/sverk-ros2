# ros_uwb

ROS2 UWB driver node using pyuwb library.

## Description

This package provides a ROS2 node that communicates with UWB (Ultra-Wideband) modules via serial UART. It publishes ranging results and peer information, and provides a service to set user data on the UWB module.

## Features

- Publishes UWB ranging results (`/uwb/result`)
- Publishes peer information (`/uwb/peer_info`)
- Service to set user data on UWB module (`/uwb/set_user_data`)
- Configurable serial port and baud rate

## Dependencies

- ROS2 (Humble or later)
- Python 3.7+
- pyserial
- rclpy
- std_msgs

## Installation

### From source

1. Clone the repository with submodules:
```bash
git clone --recurse-submodules https://git.sverk.io/uwb/ros-uwb.git
cd ros-uwb
```

If you already cloned without submodules:
```bash
git submodule update --init --recursive
```

2. Build the package:
```bash
colcon build --packages-select ros_uwb
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic usage

Run the node with default parameters (`/dev/ttyAMA0`, 500000 baud):
```bash
ros2 run ros_uwb uwb_node
```

### Custom parameters

Specify a different serial port and baud rate:
```bash
ros2 run ros_uwb uwb_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

### Using a launch file

Create a launch file to configure parameters:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_uwb',
            executable='uwb_node',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        )
    ])
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyAMA0` | Serial port device path |
| `baudrate` | int | `500000` | Serial baud rate |

## Topics

### Published

#### `/uwb/result` (ros_uwb/UwbResult)

UWB ranging results.

```
std_msgs/Header header
uint8 id
float32 range
float32 aoa
```

#### `/uwb/peer_info` (ros_uwb/UwbPeerInfo)

Information about discovered UWB peers.

```
std_msgs/Header header
uint8 id
uint8[64] user_data
```

### Services

#### `/uwb/set_user_data` (ros_uwb/SetUserData)

Set user data on the UWB module.

**Request:**
```
uint8[64] user_data
```

**Response:**
```
bool success
string message
```

## Testing

### Running tests

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_uwb
source install/setup.bash
colcon test --packages-select ros_uwb
colcon test-result --verbose
```

### Test structure

- **`test/test_uwb_node.py`** — unit tests for `UwbNode`: initialization, publishers, service, message dispatch
- **`pyuwb/tests/`** — unit tests for pyuwb: encoder, parser, messages

### Notes

- `test/conftest.py` adds `scripts/` to `sys.path` so tests can import `uwb_node.py` directly
- The ROS2 workspace must be built before running tests (`ros_uwb.msg` is generated during the build)

## Submodule

This package uses [pyuwb](https://git.sverk.io/uwb/pyuwb) as a git submodule for UWB protocol handling.
