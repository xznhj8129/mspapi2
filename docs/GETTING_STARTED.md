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
    version = api.get_api_version()
    print(f"API Version: {version['apiVersionMajor']}.{version['apiVersionMinor']}")
    print(f"Latency: {api.info['latency_ms']:.1f}ms")
```

**That's it!** You're now connected and communicating with your flight controller.

> **Note:** All API methods return their data directly. Request metadata (latency, timestamp, etc.) is available via `api.info` after each call.

## Reading Sensor Data

### Attitude (Roll, Pitch, Yaw)

```python
with MSPApi(port="/dev/ttyACM0") as api:
    attitude = api.get_attitude()
    print(f"Roll: {attitude['roll']:.1f}°")
    print(f"Pitch: {attitude['pitch']:.1f}°")
    print(f"Heading: {attitude['yaw']:.1f}°")
```

### GPS Position

```python
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyACM0") as api:
    gps = api.get_raw_gps()

    if gps['fixType'] != InavEnums.gpsFixType_e.GPS_NO_FIX:
        print(f"Position: {gps['latitude']:.7f}, {gps['longitude']:.7f}")
        print(f"Altitude: {gps['altitude']:.1f}m")
        print(f"Satellites: {gps['numSat']}")
        print(f"Speed: {gps['speed']:.1f} m/s")
        print(f"Ground course: {gps['groundCourse']:.1f}°")
    else:
        print("No GPS fix")
```

### Battery & Power

```python
with MSPApi(port="/dev/ttyACM0") as api:
    analog = api.get_inav_analog()

    print(f"Battery: {analog['vbat']:.2f}V")  # Already in volts
    print(f"Current: {analog['amperage']:.2f}A")  # Already in amps
    print(f"Power: {analog['powerDraw']:.1f}W")  # Already in watts
    print(f"Consumed: {analog['mAhDrawn']}mAh / {analog['mWhDrawn']}mWh")
    print(f"Remaining: {analog['percentageRemaining']}%")
```

### RC Channels

```python
with MSPApi(port="/dev/ttyACM0") as api:
    channels = api.get_rc_channels()

    # channels is a list of PWM values (typically 1000-2000)
    print(f"Roll: {channels[0]}")
    print(f"Pitch: {channels[1]}")
    print(f"Throttle: {channels[2]}")
    print(f"Yaw: {channels[3]}")
```

## Sending Commands

### Setting RC Channels

```python
# Option 1: List of values (all channels)
with MSPApi(port="/dev/ttyACM0") as api:
    # Set channels by position: [roll, pitch, throttle, yaw, ch5, ch6, ...]
    api.set_rc_channels([1500, 1500, 1800, 1500])

# Option 2: Dict with channel names or indices (partial override)
with MSPApi(port="/dev/ttyACM0") as api:
    api.set_rc_channels({
        "throttle": 1800,  # By name
        "roll": 1500,
        5: 1800,           # Or by channel number (0-indexed)
    })
    # Channels not specified remain at their current values
```

### Setting a Waypoint

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyACM0") as api:
    api.set_waypoint(
        waypointIndex=0,
        action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
        latitude=37.123456,
        longitude=-122.123456,
        altitude=50.0,  # meters
        param1=0,
        param2=0,
        param3=0,
        flag=0,
    )
    print("Waypoint set!")
```

## Connection Options

### Serial Connection

```python
# Default serial connection
api = MSPApi(port="/dev/ttyACM0", baudrate=115200)

# Custom timeouts (in milliseconds)
api = MSPApi(
    port="/dev/ttyACM0",
    baudrate=115200,
    read_timeout_ms=10.0,
    write_timeout_ms=10.0
)
```

### TCP Connection (via MSP Server)

```python
# Connect to MSP server instead of direct serial
api = MSPApi(tcp_endpoint="localhost:9000")
```

### UDP Connection (via MSP Multiplexer)

```python
# Connect via UDP (uses MSPv2)
api = MSPApi(udp_endpoint="localhost:27072")
```

### Forcing MSP v2

```python
# Force MSPv2 framing on any transport
api = MSPApi(
    port="/dev/ttyACM0",
    force_msp_v2=True
)
```

## Working with Messages Not in Convenience Methods

mspapi2 convenience methods cover common operations, but the library supports **all MSP messages** defined in the schema. For messages without convenience methods:

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
    reply = api._request(
        InavMSP.MSP2_INAV_LOGIC_CONDITIONS_SINGLE,
        request
    )

    # reply is a dict with field names
    print(f"Enabled: {reply['enabled']}")
    print(f"Operation: {reply['operation']}")
```

## Using Enums for Safety

mspapi2 includes enums for firmware constants. Many API methods return enum values directly:

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyACM0") as api:
    nav_status = api.get_nav_status()

    # Many fields are already enums
    if nav_status["navState"] == InavEnums.navigationFSMState_t.NAV_FSM_HOLD_INFINITELY:
        print("Holding position")

    # You can access the name and value
    print(f"Nav state: {nav_status['navState'].name} ({nav_status['navState'].value})")

    if nav_status["navError"] != InavEnums.navSystemStatus_Error_e.NAV_ERROR_NONE:
        print(f"Navigation error: {nav_status['navError'].name}")
```

## Getting Request Metadata

Request metadata is available via `api.info` after each call:

```python
attitude = api.get_attitude()

# Access metadata from the last request
print(f"Code: {api.info['code']}")
print(f"Latency: {api.info['latency_ms']:.1f}ms")
print(f"Transport: {api.info['transport']}")  # 'serial', 'tcp', or 'udp'
print(f"Attempt: {api.info['attempt']}")       # Retry count
print(f"Timestamp: {api.info['timestamp']}")  # Unix timestamp
```

## Error Handling

```python
from mspapi2 import MSPApi

try:
    with MSPApi(port="/dev/ttyACM0") as api:
        attitude = api.get_attitude()
        print(f"Roll: {attitude['roll']:.1f}°")
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
        attitude = api.get_attitude()
        print(f"Roll: {attitude['roll']:6.1f}° | Pitch: {attitude['pitch']:6.1f}°")
        time.sleep(0.1)
```

### Reading Multiple Values

```python
with MSPApi(port="/dev/ttyACM0") as api:
    # Read everything you need
    attitude = api.get_attitude()
    gps = api.get_raw_gps()
    battery = api.get_inav_analog()
    nav = api.get_nav_status()

    # Process all at once
    print(f"Position: {gps['latitude']:.7f}, {gps['longitude']:.7f}")
    print(f"Altitude: {gps['altitude']:.1f}m")
    print(f"Battery: {battery['vbat']:.2f}V")  # Already in volts
```

### Context Manager (Recommended)

```python
# Automatically opens and closes connection
with MSPApi(port="/dev/ttyACM0") as api:
    version = api.get_api_version()
    # ... do stuff ...
# Connection automatically closed
```

### Manual Connection Management

```python
api = MSPApi(port="/dev/ttyACM0")
api.open()
try:
    version = api.get_api_version()
    print(f"API v{version['apiVersionMajor']}.{version['apiVersionMinor']}")
finally:
    api.close()
```

## What's Next?

- **[Discovering Message Fields](./DISCOVERING_FIELDS.md)** - How to find message structures
- **[Examples](../examples/)** - More complete code examples
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
api.get_waypoint(waypoint_index)     # Get waypoint
api.set_waypoint(...)       # Set waypoint
api.set_rc_channels(...)    # Override RC (list or dict)
api.get_inav_status()       # INAV-specific status
api.get_inav_analog()      # Battery, current, RSSI
api.get_active_modes()     # List of active mode boxes
```

See the main [README](../README.md) for the complete list.
