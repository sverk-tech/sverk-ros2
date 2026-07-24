# ld19_lidar

ROS 2 driver for LDROBOT LD19 over USB serial or Raspberry Pi UART.

## USB mode

Use this when LD19 is connected through a USB-UART adapter and appears as
`/dev/ttyUSB0`:

```bash
ros2 launch ld19_lidar ld19_usb.launch.py
```

Override the port if needed:

```bash
ros2 launch ld19_lidar ld19_usb.launch.py port_name:=/dev/ttyUSB1
```

## UART mode

Use this when LD19 TX is connected to Raspberry Pi UART RX:

```bash
ros2 launch ld19_lidar ld19_uart.launch.py
```

The node tries `/dev/ttyAMA3`, `/dev/ttyAMA0`, `/dev/serial0`,
`/dev/ttyAMA10`, then `/dev/ttyS0`. Override explicitly if needed:

```bash
ros2 launch ld19_lidar ld19_uart.launch.py port_name:=/dev/serial0
```

## Published topic

Default output:

```bash
/ld19_lidar/scan
```

Message type:

```bash
sensor_msgs/msg/LaserScan
```

## UART wiring

LD19 connector: `ZH1.5T-4P`, 1.5 mm pitch.

```text
LD19 Tx  -> Raspberry Pi UART RX
LD19 PWM -> Raspberry Pi GND, when external speed control is not used
LD19 GND -> Raspberry Pi GND
LD19 P5V -> 5V power
```

LD19 UART is one-way at `230400 8N1`; no start command is required.

With `dtoverlay=uart3-pi5`, UART3 uses:

```text
GPIO8  / physical pin 24 -> UART3 TX
GPIO9  / physical pin 21 -> UART3 RX
```

For LD19 UART3 mode, connect `LD19 Tx` to `GPIO9 / physical pin 21`.
