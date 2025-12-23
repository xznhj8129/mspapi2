# Using the MSP Server

The MSP server lets multiple clients connect to a single flight controller over TCP. It handles request deduplication, caching, and rate limiting automatically.

## Why Use the Server?

**Problem:** Multiple apps (telemetry logger, GCS, mission planner) all need FC data, but:
- Serial port can only be opened once
- Too many requests overwhelm the FC
- Each app duplicates the same requests

**Solution:** MSP server acts as a broker:
- Multiple clients connect via TCP
- Server handles one serial connection to FC
- Deduplicates identical requests
- Caches recent replies
- Rate limits to protect FC

## Quick Start

### 1. Create a Config File

```json
{
    "host": "0.0.0.0",
    "port": 9000,
    "fc_endpoint": "/dev/ttyACM0",
    "baudrate": 115200,
    "cache_ttl_s": 0.1,
    "global_req_limit_per_sec": 200
}
```

Save as `server_config.json`

### 2. Start the Server

```bash
python -m mspapi2.msp_server --config server_config.json
```

### 3. Connect Clients

```python
from mspapi2 import MSPApi

# Connect to server instead of direct serial
with MSPApi(tcp_endpoint="localhost:9000") as api:
    info, version = api.get_api_version()
    print(version)
```

That's it! Use the API exactly the same way, but via TCP.

## Server Features

### Request Deduplication

If 3 clients request `MSP_RAW_IMU` at the same time:
- Server sends ONE request to FC
- All 3 clients get the same reply
- Reduces FC load by 3x

### Caching

Recent replies are cached (default 100ms):
- If client requests same message within TTL, gets cached data
- Reduces latency for frequently-polled messages
- Configurable per-message via `no_cache` flag

### Rate Limiting

Server enforces global request limit (default 200 req/sec):
- Automatically distributes quota among active clients
- Prevents FC overwhelm
- Queues excess requests

### Scheduled Polling

Clients can schedule periodic requests:

```python
from mspapi2 import MSPClientAPI

client = MSPClientAPI(host="localhost", port=9000)
client.open()

# Poll MSP_RAW_IMU every 100ms
client.sched_set(
    code=int(InavMSP.MSP_RAW_IMU),
    delay=0.1  # seconds
)

# Get latest scheduled data
data = client.sched_data()
print(data)
```

## Configuration Options

Full config with all options:

```json
{
    "host": "0.0.0.0",
    "port": 9000,
    "fc_endpoint": "/dev/ttyACM0",
    "baudrate": 115200,
    "cache_ttl_s": 0.1,
    "global_req_limit_per_sec": 200,
    "rate_window_s": 1.0,
    "send_queue_limit": 100,
    "server_backlog": 5,
    "request_queue_poll_s": 0.01,
    "scheduler_tick_s": 0.05,
    "min_timeout_ms": 100,
    "max_pending_waiters": 1000,
    "log_path": "/var/log/msp_server.log"
}
```

| Option | Default | Description |
|--------|---------|-------------|
| `host` | "0.0.0.0" | Server listen address |
| `port` | 9000 | Server listen port |
| `fc_endpoint` | - | Serial port or TCP endpoint |
| `baudrate` | 115200 | Serial baudrate |
| `cache_ttl_s` | 0.1 | Reply cache lifetime (seconds) |
| `global_req_limit_per_sec` | 200 | Max requests/second to FC |
| `log_path` | None | Log file path (optional) |

## Using MSPApi with Server

```python
from mspapi2 import MSPApi

# Just change the connection method
with MSPApi(tcp_endpoint="localhost:9000") as api:
    # Everything else is identical
    info, attitude = api.get_attitude()
    info, gps = api.get_raw_gps()
```

## Using MSPClientAPI (Lower Level)

For more control, use `MSPClientAPI`:

```python
from mspapi2 import MSPClientAPI, InavMSP

client = MSPClientAPI(host="localhost", port=9000, client_id="my_app")
client.open()

try:
    # Make requests
    code, payload = client.request(int(InavMSP.MSP_API_VERSION), b'')

    # Set up scheduled polling
    client.sched_set(code=int(InavMSP.MSP_RAW_IMU), delay=0.1)

    # Get scheduled data
    data = client.sched_data(InavMSP.MSP_RAW_IMU)

    # Server health
    health = client.health()
    print(f"Uptime: {health['uptime_s']}s")

finally:
    client.close()
```

## Monitoring

### Health Check

```python
client = MSPClientAPI(host="localhost", port=9000)
client.open()

health = client.health()
print(f"Server uptime: {health['uptime_s']}s")
print(f"Total requests: {health['total_requests']}")
print(f"Cache hits: {health['cache_hits']}")
```

### Statistics

```python
stats = client.stats()
print(f"Active clients: {len(stats['clients'])}")
print(f"Request rate: {stats['utilization']['global_rate_hz']} Hz")
```

## Running as a Service

### systemd Service

```ini
[Unit]
Description=MSP Server
After=network.target

[Service]
Type=simple
User=your_user
ExecStart=/usr/bin/python3 -m mspapi2.msp_server --config /etc/msp_server.json
Restart=always

[Install]
WantedBy=multi-user.target
```

Save as `/etc/systemd/system/msp-server.service`, then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable msp-server
sudo systemctl start msp-server
```

## TCP vs Serial Connection

### Direct Serial (Single Client)

**Pros:**
- Lower latency
- Simpler setup
- No server dependency

**Cons:**
- Only one connection at a time
- No request deduplication
- No caching
- Client must handle rate limiting

```python
# Direct serial
api = MSPApi(port="/dev/ttyACM0", baudrate=115200)
```

### TCP via Server (Multiple Clients)

**Pros:**
- Multiple simultaneous clients
- Request deduplication
- Automatic caching
- Rate limiting built-in
- Network accessible

**Cons:**
- Slightly higher latency
- Server must be running
- Additional setup

```python
# Via server
api = MSPApi(tcp_endpoint="localhost:9000")
```

## Common Use Cases

### Telemetry Logger + Ground Station

```bash
# Terminal 1: Start server
python -m mspapi2.msp_server --config server_config.json

# Terminal 2: Run telemetry logger
python telemetry_logger.py  # connects to localhost:9000

# Terminal 3: Run ground station
python ground_station.py    # also connects to localhost:9000
```

Both apps share one serial connection!

### Remote Access

```json
{
    "host": "0.0.0.0",
    "port": 9000,
    ...
}
```

Now clients can connect from other machines:

```python
api = MSPApi(tcp_endpoint="192.168.1.100:9000")
```

### Scheduler for Background Polling

```python
client = MSPClientAPI(host="localhost", port=9000)
client.open()

# Set up continuous polling
client.sched_set(int(InavMSP.MSP_RAW_IMU), delay=0.05)    # 20 Hz
client.sched_set(int(InavMSP.MSP_RAW_GPS), delay=0.2)     # 5 Hz
client.sched_set(int(InavMSP.MSP_ALTITUDE), delay=0.1)    # 10 Hz

# Later, get latest data (no request needed!)
while True:
    data = client.sched_data()
    imu = data.get(int(InavMSP.MSP_RAW_IMU), {}).get('data', {})
    print(f"IMU: {imu}")
    time.sleep(0.1)
```

## Troubleshooting

### "Connection refused"

Server isn't running. Start it:

```bash
python -m mspapi2.msp_server --config server_config.json
```

### "Serial port busy"

Another process has the serial port open. Find and close it:

```bash
lsof | grep ttyACM0
```

### Slow responses

Check server utilization:

```python
client = MSPClientAPI(host="localhost", port=9000)
client.open()

util = client.utilization()
print(f"Request rate: {util['global_rate_hz']} Hz")
print(f"Queue depth: {util['queue_depth']}")
```

If rate is near limit, reduce polling frequency.

## See Also

- **[Getting Started](./GETTING_STARTED.md)** - Basic library usage
- **[Main README](../README.md)** - Server protocol details
- **[Examples](../examples/server_client.py)** - Server usage examples
