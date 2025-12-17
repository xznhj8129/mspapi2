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

## Components
- `mspcodec.py`: packs/unpacks MSP payloads from the schema. No defaults, errors are raised on mismatch.
- `msp_serial.py`: MSP v1/v2 transport over serial or TCP with bounded payloads (MAX_PAYLOAD_LEN=4096), background reader, retries, and explicit failures on queue overflow or keepalive errors.
- `msp_api.py`: High-level helpers that wrap `MSPSerial` (or a custom transport) and expose typed getters/setters. Callers must provide a port, TCP endpoint, or injected transport; nothing is assumed.

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