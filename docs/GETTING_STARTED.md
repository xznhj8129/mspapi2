# Getting Started with mspapi2

## Installation

```bash
pip install mspapi2
# Or for development:
pip install -e .
```

## Common Use Cases

mspapi2 is designed for:

- **Ground Stations** - Monitor telemetry, configure settings
- **Flight Computers** - Autonomous navigation on Raspberry Pi/companion computers
- **Mission Planning** - Upload waypoints, configure missions
- **Telemetry Logging** - Record flight data
- **Testing & Development** - Test MSP implementations

**Running on a flight computer?** See **[Flight Computer Guide](./FLIGHT_COMPUTER.md)** for Raspberry Pi setup and autonomous navigation examples.

## Your First Connection

```python
from mspapi2 import MSPApi

# Connect to your flight controller
with MSPApi(port="/dev/ttyACM0", baudrate=115200) as api:
    # Get the firmware version
    info, version = api.get_api_version()
    print(f"API Version: {version['apiVersionMajor']}.{version['apiVersionMinor']}")
```

**That's it!** You're now connected and communicating with your flight controller.

## Reading Sensor Data

### Attitude (Roll, Pitch, Yaw)

```python
with MSPApi(port="/dev/ttyACM0") as api:
    info, attitude = api.get_attitude()
    print(f"Roll: {attitude['roll']}°")
    print(f"Pitch: {attitude['pitch']}°")
    print(f"Heading: {attitude['yaw']}°")
```

### GPS Position

```python
with MSPApi(port="/dev/ttyACM0") as api:
    info, gps = api.get_raw_gps()

    if gps['fixType'] > 0:
        print(f"Position: {gps['latitude']}, {gps['longitude']}")
        print(f"Altitude: {gps['altitude']}m")
        print(f"Satellites: {gps['numSat']}")
        print(f"Speed: {gps['speed']} cm/s")
    else:
        print("No GPS fix")
```

### Battery & Power

```python
with MSPApi(port="/dev/ttyACM0") as api:
    info, analog = api.get_inav_analog()

    print(f"Battery: {analog['vbat'] / 100.0:.2f}V")  # vbat is in centivolts
    print(f"Current: {analog['amperage'] / 100.0:.2f}A")  # amperage is in centiamps
    print(f"mAh drawn: {analog['mAhDrawn']}mAh")
```

### RC Channels

```python
with MSPApi(port="/dev/ttyACM0") as api:
    info, channels = api.get_rc_channels()

    # channels is a list of PWM values (typically 1000-2000)
    print(f"Throttle: {channels[2]}")
    print(f"Roll: {channels[0]}")
    print(f"Pitch: {channels[1]}")
    print(f"Yaw: {channels[3]}")
```

## Sending Commands

### Setting RC Channels (for testing)

```python
with MSPApi(port="/dev/ttyACM0") as api:
    # Set specific channels
    info, result = api.set_rc_channels({
        "throttle": 1000,  # By name
        "roll": 1500,
        5: 1800,           # Or by number
    })
```

### Setting a Waypoint

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyACM0") as api:
    info, _ = api.set_waypoint(
        waypointIndex=0,
        action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
        latitude=37.123456,
        longitude=-122.123456,
        altitude=50.0,
        flag=0,
    )
    print("Waypoint set!")
```

## Connection Options

### Serial Connection

```python
# Default serial connection
api = MSPApi(port="/dev/ttyACM0", baudrate=115200)

# Custom timeouts
api = MSPApi(
    port="/dev/ttyACM0",
    baudrate=115200,
    read_timeout=0.1,
    write_timeout=0.5
)
```

### TCP Connection (via MSP Server)

```python
# Connect to MSP server instead of direct serial
api = MSPApi(tcp_endpoint="localhost:9000")
```

## Working with Messages Not in Convenience Methods

mspapi2 has ~40 convenience methods like `get_attitude()`, but supports **all 249 MSP messages**. For messages without convenience methods:

### Step 1: Find the Message

```python
from mspapi2 import InavMSP

# List all available messages
for name in dir(InavMSP):
    if name.startswith('MSP'):
        print(name)

# Or search for specific ones
messages = [m for m in dir(InavMSP) if 'LOGIC' in m]
print(messages)  # ['MSP2_INAV_LOGIC_CONDITIONS', ...]
```

### Step 2: Find Out What Fields It Has

See **[Discovering Message Fields](./DISCOVERING_FIELDS.md)** for details, or use the helper:

```python
# See examples/introspection.py
from examples.introspection import print_message_info
from mspapi2 import InavMSP

print_message_info(InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE)
# Shows: request fields, reply fields, types, descriptions
```

### Step 3: Use the Message

```python
from mspapi2 import MSPApi, InavMSP

with MSPApi(port="/dev/ttyACM0") as api:
    # Pack request with required fields
    request = api._pack_request(
        InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
        {"conditionIndex": 0}
    )

    # Send request, get reply
    info, reply = api._request(
        InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
        request
    )

    # reply is a dict with field names
    print(f"Enabled: {reply['enabled']}")
    print(f"Operation: {reply['operation']}")
```

## Using Enums for Safety

mspapi2 includes enums for firmware constants:

```python
from mspapi2 import MSPApi, InavMSP
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyACM0") as api:
    info, nav_status = api.get_nav_status()

    # Use enums instead of magic numbers
    if nav_status["navState"] == InavEnums.navigationFSMState_t.NAV_FSM_HOLD_INFINITELY:
        print("Holding position")

    if nav_status["navError"] != InavEnums.navSystemStatus_Error_e.NAV_ERROR_NONE:
        print(f"Navigation error: {nav_status['navError'].name}")
```

## Getting Request Metadata

Every request returns an `info` dict with metadata:

```python
info, reply = api.get_attitude()

print(f"Latency: {info['latency_ms']}ms")
print(f"Cached: {info['cached']}")
print(f"Transport: {info['transport']}")  # 'serial' or 'tcp'
print(f"Attempt: {info['attempt']}")      # Retry count
```

## Error Handling

```python
from mspapi2 import MSPApi, InavMSP

try:
    with MSPApi(port="/dev/ttyACM0") as api:
        info, reply = api.get_attitude()
except RuntimeError as e:
    print(f"MSP request failed: {e}")
except Exception as e:
    print(f"Connection error: {e}")
```

## Common Patterns

### Polling Loop

```python
import time
from mspapi2 import MSPApi

with MSPApi(port="/dev/ttyACM0") as api:
    while True:
        info, attitude = api.get_attitude()
        print(f"Roll: {attitude['roll']:6.1f}° | Pitch: {attitude['pitch']:6.1f}°")
        time.sleep(0.1)
```

### Reading Multiple Values

```python
with MSPApi(port="/dev/ttyACM0") as api:
    # Read everything you need
    info, attitude = api.get_attitude()
    info, gps = api.get_raw_gps()
    info, battery = api.get_inav_analog()
    info, nav = api.get_nav_status()

    # Process all at once
    print(f"Position: {gps['latitude']}, {gps['longitude']}")
    print(f"Altitude: {gps['altitude']}m")
    print(f"Battery: {battery['vbat'] / 100.0:.2f}V")
```

### Context Manager (Recommended)

```python
# Automatically opens and closes connection
with MSPApi(port="/dev/ttyACM0") as api:
    info, version = api.get_api_version()
    # ... do stuff ...
# Connection automatically closed
```

### Manual Connection Management

```python
api = MSPApi(port="/dev/ttyACM0")
api.open()
try:
    info, version = api.get_api_version()
finally:
    api.close()
```

## What's Next?

- **[Discovering Message Fields](./DISCOVERING_FIELDS.md)** - How to find message structures
- **[Examples](../examples/)** - More complete code examples
- **[Server Setup](./SERVER.md)** - Using the TCP server for multi-client access
- **[API Reference](../README.md)** - Complete list of convenience methods

## Quick Reference

### Common Convenience Methods

```python
api.get_api_version()       # Firmware version
api.get_fc_variant()        # INAV/Betaflight
api.get_board_info()        # Board type
api.get_attitude()          # Roll/pitch/yaw
api.get_altitude()          # Altitude
api.get_imu()               # Accelerometer, gyro, magnetometer
api.get_raw_gps()           # GPS data
api.get_rc_channels()       # RC channel values
api.get_battery_config()    # Battery configuration
api.get_nav_status()        # Navigation state
api.get_waypoint(index)     # Get waypoint
api.set_waypoint(...)       # Set waypoint
api.set_rc_channels(...)    # Override RC (testing)
```

See the main [README](../README.md) for the complete list.
