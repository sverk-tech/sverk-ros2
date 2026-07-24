# pyuwb

UWB Protocol Library

## Installation

### From git repository

```bash
git clone https://git.sverk.io/uwb/pyuwb.git
cd pyuwb
pip install -e .
```

## Requirements

- Python >= 3.7
- pyserial >= 3.5

## Usage
### UART

```python
import serial
from pyuwb import (
    UwbParser,
    UwbEncoder,
    ModuleResult,
    PeerInfo,
    SetUserData,
)

# Open serial port
ser = serial.Serial('/dev/ttyAMA0', baudrate=500000, timeout=0.1)
ser.reset_input_buffer()

# Create parser
parser = UwbParser()

# Read and process data
while True:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        for msg_type, payload in parser.process_data(data):
            if msg_type == ModuleResult.TYPE_ID:
                result = ModuleResult.parse(payload)
                print(f"Result: id={result.id}, range={result.range}, aoa={result.aoa}")
            elif msg_type == PeerInfo.TYPE_ID:
                peer_info = PeerInfo.parse(payload)
                print(f"Peer: id={peer_info.id}, user_data={peer_info.user_data}")

# Send data
user_data_msg = SetUserData(user_data=bytes([1, 2, 3, 4]))
packet = UwbEncoder.encode_packet(SetUserData.TYPE_ID, user_data_msg.encode())
ser.write(packet)
```

## Running Tests

Install development dependencies:

```bash
pip install -e ".[dev]"
```

Run tests:

```bash
pytest tests/ -v
```

Run tests with coverage:

```bash
pytest tests/ -v --cov=pyuwb --cov-report=term
```
