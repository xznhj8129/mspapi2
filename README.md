# mspapi2
MultiWii Serial Protocol tooling for INAV/BetaFlight with strict separation between codec, transport, API helpers, and a multi-client TCP broker.

## Requirements
- Python 3.9+
- The INAV MSP schema JSON (`msp_messages.json`) in `mspapi2/lib/` (already included)

## Install
```bash
pip install .
# or editable
pip install -e .
```

## Documentation

- **[Getting Started](docs/GETTING_STARTED.md)** - Quick start guide and basic usage
- **[Discovering Message Fields](docs/DISCOVERING_FIELDS.md)** - How to find message structure
- **[Server Setup](docs/SERVER.md)** - Using the TCP server for multi-client access
- **[Examples](examples/)** - Working code examples

## Components
- `mspcodec.py`: packs/unpacks MSP payloads from the schema. No defaults, errors are raised on mismatch.
- `msp_serial.py`: MSP v1/v2 transport over serial or TCP with bounded payloads (MAX_PAYLOAD_LEN=4096), background reader, retries, and explicit failures on queue overflow or keepalive errors.
- `msp_api.py`: High-level helpers that wrap `MSPSerial` (or a custom transport) and expose typed getters/setters. Callers must provide a port, TCP endpoint, or injected transport; nothing is assumed.
- `msp_server.py`: TCP broker that dedupes requests, rate-limits per client, caches replies, schedules periodic polls, and refuses to serve undecodable payloads.


## Server
* Message broker, handles multiple client connections
* uses message queues to queue MSP message requests and responses
* Programmable fixed interval message scheduler
* Keeps timing and latency information per message
* deduplicates identical `(code, payload)` requests so multiple clients share a single FC round-trip
* rate-limits and queues traffic so the FC isn't overwhelmed, while still keeping every client responsive

## Client-Server connection summary
* Transport is newline-delimited JSON over TCP (documented below)
* Clients identify themselves via `client_id` so diagnostics/rate-limit buckets are per-user
* Scheduler commands (`sched_set`, `sched_get`, `sched_remove`) let clients set periodic polling
* Server deduplicates identical `(code, payload)` requests using CRC, caches replies, and automatically throttles each client based on the global 200 req/s budget

Example:
```Client1: i want MSP_RAW_IMU (empty payload)!

Server: crc(MSP_RAW_IMU+(empty payload)) is hex a2b8e92, I'm setting requests_dict[Client1] = "a2b8e92". Since a2b8e92 isn't in the send queue already and the queue isn't full, i'm adding a2b8e92 to it. 

Client2: i want MSP_RAW_IMU (empty payload)!

Server: crc(MSP_RAW_IMU+(empty payload)) is hex a2b8e92, I'm setting requests_dict[Client2] = "a2b8e92". Since that's already in the send queue, we don't add it again.

Client3: i want MSP_RAW_IMU (empty payload)!

Server: crc(MSP_RAW_IMU+(empty payload)) is hex a2b8e92, I'm setting requests_dict[Client3] = "a2b8e92". Since that's already in the send queue, we don't add it again.

So they all want the same thing. 

Server: since i'm free right now, i'm popping from the send queue, it's "a2b8e92", so i'm requesting MSP_RAW_IMU (empty payload) from the API.

Client4: I want MSP_WP(waypointIndex=4)!

Server: crc is "f8b2bd8", I'm setting requests_dict[Client4] = "f8b2bd8". Since f8b2bd8 isn't in the send queue already and the queue isn't full, i'm adding a2b8e92 to it. I'm still waiting for the response to a2b8e92, so it will have to wait.

Server: the serial handler has replied to a2b8e92, so i'm setting responses_dict["a2b8e92"] = reply data. Client1, Client2, Client3 are all waiting for a2b8e92, so they all get their reply, which i only needed to request once.
I'm now getting f8b2bd8 from the send queue and sending the MSP_WP(waypointIndex=4) request to the API.

Client5: i want MSP_RAW_IMU (empty payload)!

Server: crc(MSP_RAW_IMU+(empty payload)) is hex a2b8e92, I'm setting requests_dict[Client1] = "a2b8e92". I was just asked that 25ms ago. The serial still busy waiting for the WP message data, and i still have the last a2b8e92 which age<ttl of 100ms, since you didn't insist on ABSOLUTELY NO CACHE, i'm giving you the last a2b8e92 after making you wait 50ms or something.```


## Running the server
`msp_server.py` now requires a JSON config (see `msp_server.config.example.json`) that specifies host/port, logging, and either a serial or TCP FC link:
```bash
python -m mspapi2.msp_server --config /etc/mspapi2/server.json
```
If the schema expects a payload, requests without one fail. Decode failures are returned as errors, not silent `None`.

### Server config
All runtime knobs live in a JSON config (see `msp_server.config.example.json`): host, port, log_path, serial or tcp endpoint, baudrate, cache_ttl_s, global_req_limit_per_sec, rate_window_s, send_queue_limit, server_backlog, request_queue_poll_s, scheduler_tick_s, min_timeout_ms, max_pending_waiters.

### Protocol (JSON-per-line over TCP)
Client requests (one per line):
```json
{"id":1,"client_id":"jetson","code":1}
{"id":2,"client_id":"jetson","code":102,"payload":{"accData":[0,0,0],"gyroData":[0,0,0],"magData":[0,0,0]}}
{"id":3,"client_id":"jetson","action":"sched_set","code":1,"delay":5.0}
{"id":4,"client_id":"jetson","action":"sched_get"}
{"id":5,"client_id":"jetson","action":"sched_data","codes":["MSP_RAW_IMU","MSP_ALTITUDE"]}
{"id":6,"client_id":"jetson","action":"health"}
{"id":7,"client_id":"jetson","action":"shutdown"}
```
Fields:
- `id` optional correlation tag echoed back.
- `client_id` optional label for diagnostics/rate limiting.
- `code` MSP numeric code or `InavMSP` name.
- `payload` structured data packed via codec (required when schema demands it).
- `raw` base64 payload bytes (takes precedence over `payload`).
- `timeout_ms` per-request timeout (ms).
- `no_cache` disable short cache.
- `action` `sched_set`/`sched_get`/`sched_remove`; `delay` (seconds) for schedules.
- `action` `sched_data` returns cached scheduled telemetry (optionally filtered by `codes` list). Non-MSP actions: `health`, `utilization`, `clients`, `stats`, `reset` (reopens transport and clears state), `shutdown` (graceful stop).

Server replies:
```json
{"id":1,"ok":true,"code":1,"name":"MSP_API_VERSION",
 "payload_b64":"AAECAw==","payload_len":3,"duration_ms":3.2,"cached":false,
 "data":{"mspProtocolVersion":0,"apiVersionMajor":2,"apiVersionMinor":5}}
```
Errors look like:
```json
{"id":42,"ok":false,"error":"decode failed for MSP_FOO: ..."}
```

Rate limiting: global budget 200 req/s is divided among active clients. Diagnostics report utilization and throttle time.

### Scheduler
`sched_set` installs/updates a single interval per code; `delay <= 0` removes it (or use `sched_remove`). Scheduled polls feed the cache and latency stats. `sched_get` lists current timers.

## Using the Python API
```python
from mspapi2 import MSPApi

api = MSPApi(port="/dev/ttyACM0", baudrate=115200)
api.open()
try:
    info, version = api.get_api_version()
    print(info, version)
finally:
    api.close()
```

Channel helpers honor the RX map: `api.get_ch("pitch")` resolves via the FC RX map; `api.set_rc_channels({"pitch": 1600, 2: 1700})` overwrites specific indices without reshuffling manually.

### Mission-oriented usage with enums
- `InavMSP` enumerates every MSP code; pass these names instead of bare integers to avoid mismatches (e.g., `api._request(InavMSP.MSP_NAV_STATUS)` or `MSPClientAPI.request(int(InavMSP.MSP_NAV_STATUS), ...)`).
- `InavEnums` contains the FC-side enums (navigation states, waypoint actions, arming flags, sensor types). Always construct or compare against these enums so mission logic cannot drift from firmware constants.
- Example: fetch nav status and branch safely:
```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums, InavMSP

api = MSPApi(port="/dev/ttyACM0", baudrate=115200)
api.open()
try:
    info, status = api.get_nav_status()  # uses InavMSP.MSP_NAV_STATUS under the hood
    if status["navState"] == InavEnums.navigationFSMState_t.NAV_FSM_HOLD_INFINITELY:
        # hold logic
        pass
    if status["navError"] != InavEnums.navSystemStatus_Error_e.NAV_ERROR_NONE:
        raise RuntimeError(f"Navigation error: {status['navError'].name}")
finally:
    api.close()
```
- Example: set a waypoint using explicit enum values (no magic numbers):
```python
from mspapi2 import MSPApi
from mspapi2.lib import InavEnums

api = MSPApi(port="/dev/ttyACM0", baudrate=115200)
api.open()
try:
    info, _ = api.set_waypoint(
        waypointIndex=0,
        action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
        latitude=37.123456,
        longitude=-122.123456,
        altitude=50.0,
        flag=0,
    )
finally:
    api.close()
```
If you drop to raw transport, still use `InavMSP` for codes and pack payloads with the codec to avoid implicit defaults.

## Stress testing
`stress_tester.py` drives the server with concurrent API workers and scheduler mutations:
```bash
python stress_tester.py --host 127.0.0.1 --port 9000 --api-workers 8 --scheduler-workers 4 --duration 60
```
It uses only valid MSP traffic; monitor server logs for backpressure and error reporting.
