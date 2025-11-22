# mspapi2
Clean rewrite of an API for MultiWii Serial Protocol for [BetaFlight](https://github.com/betaflight/betaflight) and [INAV](https://github.com/INAVFlight/INAV)

[unavlib](https://github.com/xznhj8129/uNAVlib) was horrifying spaghetti code and driving me insane; this will use clearly seperated layers to keep serial handling, message brokering and threaded blocking clients completely separated


* Uses canonical INAV MSP definition JSON from [msp-documentation](https://github.com/xznhj8129/msp_documentation) for API generation
* Direct use of source code enums and defines wherever possible
* Focus on parsing and converting actual flight controller code into API functions, not re-re-re-reimplementing stuff
* Uses parsed and interpreted direct source information to generate documentation (see msp-documentation)
* Includes a robust asynchronous broker (TCP server + shared rate limiting + caching) so many clients can share one FC connection safely


Will change quickly while i figure out cleanest way to seperate everything


## Installation

```bash
# from a clone
pip install .

# or for live development
pip install -e .
```

The package name is `mspapi2`. Once installed you can use the console entry point exposed by `setup.py`:

```bash
# identical to `python -m mspapi2`
mspapi2-server --help
mspapi2-server --serial /dev/ttyACM0 --baudrate 115200 --host 0.0.0.0 --port 9000
```

For programmatic access import the `MSPApi` class:

```python
from mspapi2 import MSPApi

with MSPApi(port="/dev/ttyACM0") as api:
    info, version = api.get_api_version()
    print("Latency (ms):", info["latency_ms"])
    print("Version:", version)
```

Channel helpers now honor the RX map. Call `api.get_ch("pitch")` (or any RX-map name) to read the corresponding value, and pass a mapping to `api.set_rc_channels({"pitch": 1600, 2: 1700})` to override specific channels without manually reshuffling indices.

# Architecture:
## MSP Handler (msp_serial.py)
* Single point of contact to FC and only serial socket handler
* socket communication to API
* packed bytes in -> serial req/resp -> packed bytes out
* robust
* will not block API
* Fail-safe, will keepalive and retry connection/message on error
* Wraps reply with code indicating success or error code

## API (msp_api.py)
* Encodes/decodes MSP bytes from/to parsed data using msp_codec
* Provides high level function API for common commands, returning `(info, data)` so callers can inspect latency/cache metadata alongside the parsed payload
* Can run directly over `MSPSerial` or via a custom `serial_transport` (e.g., `example_client.py` uses a TCP proxy)
* Preferred usage is explicit open/close:
  ```python
  api = MSPApi(port="/dev/ttyACM0")
  api.open()
  try:
      print(api.get_api_version())
      print(api.diag)
  finally:
      api.close()
  ```
* Exposes `sched_set`, `sched_get`, and `sched_remove` helpers when the transport supports the server-side scheduler

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
* Scheduler commands (`sched_set`, `sched_get`, `sched_remove`) let clients request periodic polls without spamming the FC
* Server deduplicates identical `(code, payload)` requests, caches replies, and automatically throttles each client based on the global 200 req/s budget

## Multi-client MSP server

`msp_server.py` implements the broker outlined above. It exposes a simple newline-delimited JSON protocol over TCP so that many MSP users can share the same physical FC connection (serial or TCP) without trampling each other.

### Running the server

```bash
python msp_server.py --serial /dev/ttyACM0 --baudrate 115200 --host 0.0.0.0 --port 9000
# or, to connect the server to an FC that is already reachable via TCP:
python msp_server.py --tcp-endpoint localhost:5760 --port 9000
```

The server:

* keeps a single `MSPSerial` instance alive with automatic keep-alives and retries,
* deduplicates identical `(code, payload)` requests that are in-flight so only one MSP round-trip hits the FC,
* caches identical requests for `cache_ttl` seconds (default 1 s) so repeated queries can be served immediately,
* returns both the raw payload (base64) and, when possible, the structured decode using `mspcodec`, along with diagnostic metadata (latency, averages, requests/min, reconnection count, queue depth, and client info),
* enforces a global 200 req/s budget split across active clients (rate limit = `200 / clients`), with automatic throttling/queueing; each response’s `diag.client.rate` field reports current utilization and throttle time.

### Protocol overview

Clients open a TCP socket to the server and send one JSON document per line:

```json
{"id":"req-1","client_id":"jetson","code":"MSP_API_VERSION","timeout_ms":1000}
{"id":"req-2","client_id":"jetson","code":"MSP_FC_VARIANT","no_cache":true}
{"id":"req-3","client_id":"jetson","code":102,"payload":{"accData":[0,0,0],"gyroData":[0,0,0],"magData":[0,0,0]}}
{"id":"req-4","client_id":"jetson","action":"sched_set","code":"MSP_API_VERSION","delay":5.0}
{"id":"req-5","client_id":"jetson","action":"sched_get"}
```

Fields:

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string (optional) | Echoed back so clients can correlate responses. |
| `client_id` | string (optional) | Friendly identifier recorded with the server's client metadata. |
| `code` | int or string | MSP code (numeric or `InavMSP` name). |
| `payload` | object/array (optional) | Structured payload that will be packed using `mspcodec`. |
| `raw` | base64 string (optional) | Raw payload bytes (takes precedence over `payload`). |
| `timeout_ms` | number (optional) | Per-request timeout, default 1000 ms. |
| `no_cache` | bool (optional) | Skip the short-term response cache. |
| `action` | string (optional) | Special commands such as `sched_set` or `sched_get`. |
| `delay` | number (optional) | Scheduler interval (seconds) used with `sched_set`. |

Server replies are also JSON lines:

```json
{
  "id": "req-1",
  "ok": true,
  "code": 1,
  "name": "MSP_API_VERSION",
  "payload_b64": "AAECAw==",
  "payload_len": 3,
  "duration_ms": 3.21,
  "cached": false,
  "data": {"mspProtocolVersion":0,"apiVersionMajor":2,"apiVersionMinor":5},
  "diag": {
    "code": 1,
    "duration_ms": 3.21,
    "cached": false,
    "cache_age_ms": 0,
    "scheduled": false,
    "code_stats": {"count": 42, "avg_ms": 3.10, "requests_per_min": 12, "last_ms": 3.21},
    "server": {"requests_total": 420, "reconnections": 1, "requests_per_min": 55,
               "inflight_total": 3, "rate_limit_per_client": 100},
    "client": {"client_id": "jetson", "session_id": "...", "address": "127.0.0.1", "port": 54321,
                "connected_at": 1700000000.0, "last_seen": 1700000001.0, "pending": 0,
                "rate": {"current_per_sec": 95, "limit_per_sec": 100, "utilization": 0.95, "throttle_ms": 0}}
  }
}
```

In addition to the `data` payload, each response carries a `diag` block with latency information, per-code averages, queue depth, rate-limit utilization, and per-client/server metadata (pending requests, inflight totals, reconnections, etc.). The high-level `MSPApi` stores the last response's diagnostics on `api.diag`, so client applications can make decisions without having to parse raw transport data.

On failure:

```json
{"id":"req-42","ok":false,"error":"MSP request timeout for code 105"}
```

This JSON/line protocol is intentionally simple so it can be used from shell tools, Python, Node, etc. A minimal Python client would look like:

```python
import json, socket

with socket.create_connection(("127.0.0.1", 9000)) as sock:
    sock.sendall(b'{"id":"caps","code":"MSP_API_VERSION"}\n')
    print(json.loads(sock.makefile().readline()))
```

See `example_client.py` for a richer CLI that tunnels the high-level `MSPApi` calls through the server using the `serial_transport` hook, identifies itself with `client_id`, configures scheduler entries, demonstrates forced uncached requests, and logs the diagnostics that come back for each request. Binary blobs in the `data` field are base64 strings, whereas printable char arrays are decoded to text for readability.

### Scheduler API

The server can poll specific MSP codes at fixed intervals and keep the responses hot in the cache. Use `action":"sched_set"` to add/update a timer (use `delay <= 0` or `action":"sched_remove"` to remove it) and `action":"sched_get"` to inspect the current schedule. Each code can have at most one timer.

```json
{"action":"sched_set","code":"MSP_API_VERSION","delay":5.0}
{"action":"sched_get"}
```

The Python `MSPApi` exposes `sched_set` / `sched_get` / `sched_remove` helpers that forward these commands when the underlying transport supports them (e.g., the TCP proxy used in `example_client.py`). Scheduled polls behave like normal requests: they update the cache, feed into the per-code latency statistics, and appear in `api.diag` for the next client call.

To force a non-cached response for any single request, send `"no_cache": true` (or call the underlying transport with `cacheable=False` as demonstrated in `example_client.py`).

### Stress testing

`stress_tester.py` hammers the server with concurrent MSP API workers and random scheduler mutations (without sending garbage to the FC):

```bash
python stress_tester.py --host 127.0.0.1 --port 9000 --api-workers 8 --scheduler-workers 4 --duration 60
```

It uses `MSPServerTransport`/`MSPApi` for legitimate traffic only, focusing on server-side queue/rate-limit behavior. Monitor the server console (now logging full tracebacks to stdout) to see how it copes.

Scheduling/deduplication/publishing hooks live server-side so multiple independent apps (GCS, logging, custom dashboards, etc.) can safely share the FC MSP link. The original `multi.py` still shows the low-level message broker pattern if you need to embed the architecture inside another process.
