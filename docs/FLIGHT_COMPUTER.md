# Using mspapi2 on Flight Computers

Flight computers (also called companion computers) are single-board computers like Raspberry Pi that run alongside your flight controller to add autonomous capabilities.

## What is a Flight Computer?

A flight computer connects to your INAV flight controller via serial (UART) and can:

- **Read telemetry** - GPS position, attitude, battery, navigation state
- **Control navigation** - Set waypoints, change modes, adjust settings
- **Make decisions** - Obstacle avoidance, follow-me, mission planning
- **Partial automation** - Override specific RC channels while pilot controls others

**The key concept:** Your flight computer sets *goals* (waypoints, modes), and INAV handles the actual flight control.

## Hardware Setup

### Typical Configuration

```
┌─────────────────────┐
│  Raspberry Pi       │
│  (Flight Computer)  │
│                     │
│  - Camera           │
│  - Sensors          │
│  - Your code        │
└──────────┬──────────┘
           │ UART (Serial)
           │ /dev/ttyAMA0
           │
┌──────────┴──────────┐
│  Flight Controller  │
│  (INAV)             │
│                     │
│  - Handles flight   │
│  - GPS navigation   │
│  - Motor control    │
└─────────────────────┘
```

### Wiring

Connect UART from flight computer to a UART port on your flight controller:

- **TX** (Pi) → **RX** (FC UART port)
- **RX** (Pi) → **TX** (FC UART port)
- **GND** (Pi) → **GND** (FC)

**Important:** Use 3.3V logic levels. Most modern FCs use 3.3V, but verify first.

### INAV Configuration

In INAV Configurator, configure the UART port for MSP:

```
# CLI
serial 2 1024 115200 115200 0 115200
save
```

This sets UART2 for MSP at 115200 baud.

## Software Setup on Flight Computer

### Installation

```bash
# On Raspberry Pi
sudo apt-get update
sudo apt-get install python3-pip

# Install mspapi2
pip3 install mspapi2
```

### Find the Serial Port

```bash
# List serial devices
ls /dev/tty*

# Common on Raspberry Pi:
# /dev/ttyAMA0 - Primary UART
# /dev/ttyS0   - Mini UART (less stable)
# /dev/ttyUSB0 - USB adapter

# Test connection
sudo chmod 666 /dev/ttyAMA0  # Give permissions
```

### Verify Connection

```python
from mspapi2 import MSPApi

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    info, version = api.get_api_version()
    print(f"Connected! API Version: {version['apiVersionMajor']}.{version['apiVersionMinor']}")
```

## Common Use Cases

### 1. Telemetry Monitoring

Monitor flight state and make decisions:

```python
from mspapi2 import MSPApi
import time

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    while True:
        # Read telemetry
        _, attitude = api.get_attitude()
        _, gps = api.get_raw_gps()
        _, battery = api.get_inav_analog()
        _, nav = api.get_nav_status()

        # Make decisions
        if battery['vbat'] / 100.0 < 10.5:  # vbat is in centivolts
            print("Low battery - trigger RTH!")
            # Set mode or waypoint for return to home

        if gps['fixType'] == 0:
            print("No GPS fix - stay in position hold")

        time.sleep(0.1)
```

### 2. Waypoint Navigation

Control where the aircraft goes:

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    # Set waypoint #0 to a specific location
    api.set_waypoint(
        waypointIndex=0,
        action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
        latitude=37.12345,    # Target latitude
        longitude=-122.6789,  # Target longitude
        altitude=50.0,        # 50 meters altitude
        flag=0
    )

    # Aircraft will fly to this waypoint when in WP mode
    print("Waypoint set - activate WP mode on transmitter")
```

### 3. Follow-Me Mode

Update waypoint #255 to create a moving target:

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums
import time

def get_ground_station_position():
    """Get current position from GPS or other source"""
    # Replace with actual GPS reading
    return (37.12345, -122.6789)

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    while True:
        # Get current ground station position
        lat, lon = get_ground_station_position()

        # Update special waypoint #255 (Follow-Me)
        api.set_waypoint(
            waypointIndex=255,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=lat,
            longitude=lon,
            altitude=20.0,  # Hover 20m above ground station
            flag=0
        )

        time.sleep(1)  # Update every second
```

### 4. Obstacle Avoidance

Detect obstacles and calculate safe waypoints:

```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums
import time

def detect_obstacle():
    """Check sensors for obstacles"""
    # Replace with actual sensor reading (lidar, camera, etc.)
    return False

def calculate_avoidance_waypoint(current_lat, current_lon, current_heading):
    """Calculate waypoint to avoid obstacle"""
    # Simple example: turn 45° right and go 50m
    # Replace with actual path planning logic
    new_lat = current_lat + 0.0005  # Example offset
    new_lon = current_lon + 0.0005
    return new_lat, new_lon

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    while True:
        if detect_obstacle():
            # Get current position
            _, gps = api.get_raw_gps()
            _, attitude = api.get_attitude()

            # Calculate safe waypoint
            new_lat, new_lon = calculate_avoidance_waypoint(
                gps['latitude'], gps['longitude'], attitude['yaw']
            )

            # Command aircraft to avoid
            api.set_waypoint(
                waypointIndex=0,
                action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
                latitude=new_lat,
                longitude=new_lon,
                altitude=gps['altitude'],  # Maintain current altitude
                flag=0
            )
            print("Obstacle detected - avoiding!")

        time.sleep(0.1)
```

### 5. RC Channel Override

Override specific RC channels while pilot controls others:

```python
from mspapi2 import MSPApi

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    # Pilot controls roll/pitch/yaw
    # Flight computer controls throttle and modes
    api.set_rc_channels({
        "throttle": 1500,  # Auto-throttle
        5: 1800,           # Activate NAV WP mode
    })
    # Channels not specified remain under pilot control
```

## Running as a Service

### systemd Service

Create `/etc/systemd/system/flight-computer.service`:

```ini
[Unit]
Description=Flight Computer Script
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/flight-computer
ExecStart=/usr/bin/python3 /home/pi/flight-computer/main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable flight-computer
sudo systemctl start flight-computer
sudo systemctl status flight-computer
```

### Auto-start on Boot

Add to `/etc/rc.local` (before `exit 0`):

```bash
# Set serial port permissions
chmod 666 /dev/ttyAMA0

# Start flight computer script
su - pi -c "cd /home/pi/flight-computer && python3 main.py &"
```

## Best Practices

### 1. Handle Disconnections

```python
from mspapi2 import MSPApi
import time

def main_loop():
    while True:
        try:
            with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
                while True:
                    # Your telemetry reading / control logic
                    _, attitude = api.get_attitude()
                    time.sleep(0.1)
        except Exception as e:
            print(f"Connection lost: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)

main_loop()
```

### 2. Low Latency Polling

```python
with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    # Fast loop for critical data
    while True:
        _, attitude = api.get_attitude()
        _, nav = api.get_nav_status()

        # React quickly to navigation state
        if nav['navState'] == InavEnums.navigationFSMState_t.NAV_FSM_LAND_IN_PROGRESS:
            print("Landing!")

        time.sleep(0.05)  # 20 Hz
```

### 3. Separate Threads for Different Tasks

```python
import threading
from mspapi2 import MSPApi
import time

def telemetry_reader(api):
    """Fast telemetry loop"""
    while True:
        _, attitude = api.get_attitude()
        # Process telemetry
        time.sleep(0.05)

def waypoint_controller(api):
    """Slower waypoint updates"""
    while True:
        # Calculate and set waypoints
        time.sleep(1.0)

with MSPApi(port="/dev/ttyAMA0", baudrate=115200) as api:
    t1 = threading.Thread(target=telemetry_reader, args=(api,), daemon=True)
    t2 = threading.Thread(target=waypoint_controller, args=(api,), daemon=True)

    t1.start()
    t2.start()

    # Main thread stays alive
    t1.join()
```

### 4. Log Everything

```python
import logging

logging.basicConfig(
    filename='/var/log/flight-computer.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s: %(message)s'
)

with MSPApi(port="/dev/ttyAMA0") as api:
    _, gps = api.get_raw_gps()
    logging.info(f"GPS: {gps['latitude']}, {gps['longitude']}, alt={gps['altitude']}m")
```

## Troubleshooting

### Serial Port Permission Denied

```bash
# Temporary fix
sudo chmod 666 /dev/ttyAMA0

# Permanent fix - add user to dialout group
sudo usermod -a -G dialout pi
# Then logout and login again
```

### Bluetooth Conflicts on Raspberry Pi

Bluetooth may use the primary UART. Disable it:

```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt

# Add this line:
dtoverlay=disable-bt

# Reboot
sudo reboot
```

### Connection Timeouts

```python
# Increase timeouts for slow connections
api = MSPApi(
    port="/dev/ttyAMA0",
    baudrate=115200,
    read_timeout=0.5,   # Increase from default
    write_timeout=1.0
)
```

### Check Serial Communication

```bash
# Monitor serial port
sudo apt-get install minicom
minicom -D /dev/ttyAMA0 -b 115200

# Or use screen
screen /dev/ttyAMA0 115200
```

## Performance Considerations

### Baudrate

- **115200** - Standard, works for most applications
- **230400** - Higher bandwidth, better for fast telemetry
- **460800** - Maximum for critical applications

Configure in INAV CLI:

```
serial 2 1024 230400 230400 0 230400
save
```

And in your code:

```python
api = MSPApi(port="/dev/ttyAMA0", baudrate=230400)
```

### Request Rate

Don't overwhelm the flight controller:

```python
# Good: Reasonable update rates
time.sleep(0.05)  # 20 Hz for critical data
time.sleep(0.1)   # 10 Hz for normal telemetry
time.sleep(1.0)   # 1 Hz for waypoint updates

# Bad: Too fast
time.sleep(0.001)  # 1000 Hz - will overload FC
```

## Safety Considerations

1. **Failsafe** - Always have RC failsafe configured
2. **GPS Required** - Don't command waypoints without GPS fix
3. **Battery Monitoring** - Always check battery voltage
4. **Mode Awareness** - Know which mode is active
5. **Testing** - Test on bench before flight
6. **Logs** - Log all actions for post-flight analysis

## See Also

- **[Getting Started](./GETTING_STARTED.md)** - Basic library usage
- **[Examples](../examples/flight_computer.py)** - Flight computer example
- **[INAV Wiki](https://github.com/iNavFlight/inav/wiki/INAV-Remote-Management%2C-Control-and-Telemetry)** - Official INAV documentation
