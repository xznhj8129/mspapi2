# mspapi2

Lightweight Python client and server for INAV’s MultiWii Serial Protocol (MSP). It wraps a generated codec with a small, typed helper API so you can query or command a flight controller over serial or TCP without hand-crafting binary payloads.

## What’s here
- `MSPApi` (mspapi2/msp_api.py): high-level helper that mirrors MSP messages with typed fields and basic unit conversion.
- `MSPCodec` (mspapi2/mspcodec.py): packs/unpacks MSP payloads from a JSON schema.
- `mspapi2-server`: tiny multi-client bridge that relays MSP over TCP.
- Examples: `example_api.py` (read a bunch of MSP data and push RC/waypoint writes).

## Getting the API working
1) Ensure you have an up-to-date MSP schema and enums list from INAV that matches your firmware. The repository includes a `syncjson.sh` helper that fetches the current INAV JSON specs.
2) Boxes and defines are currently hardcoded snapshots (`mspapi2/lib/boxes.py`, `mspapi2/lib/inav_defines.py`).

## Install
```bash
python -m venv .venv
. .venv/bin/activate
pip install -e .
```

## Quick use
```python
from mspapi2.msp_api import MSPApi

with MSPApi(port="/dev/ttyACM0", baudrate=115200) as api:
    api_version = api.get_api_version()
    status = api.get_inav_status()
    rc_ack = api.set_rc_channels([1500, 1500, 1500, 1500])
```
- For TCP transports, pass `tcp_endpoint="host:port"` and omit `port`.
- For UDP transports (e.g. via MSP multiplexer), pass `udp_endpoint="host:port"`; MSP v2 will be used.
- To force MSP v2 framing on serial/TCP, set `force_msp_v2=True`.
- Diagnostics for the last call live on `api.info` (latency, transport, attempt, timestamp).

## Notes and caveats
- The codec trusts the JSON schema; if the schema is stale, calls will misdecode. Keep `msp_messages.json` and `inav_enums.json` fresh from INAV.
- Mode boxes and INAV defines are static snapshots; automatic updates are WIP
- Exceptions are allowed to surface; crashes are preferable to silently masking protocol drift.
