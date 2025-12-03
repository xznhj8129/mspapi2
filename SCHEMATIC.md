# MSP API / Server schematic

```
Client app
  |
  | JSON over TCP (client_id, code, payload/raw, timeout_ms, sched_*)
  v
MSPClientAPI (TCP)
  |
  | JSON line
  v
msp_server.py
  - queues per-client requests
  - dedupes (code+payload CRC)
  - short cache
  - optional sched_set/sched_get/sched_remove timers
  - sched_data returns latest scheduled payloads
  - health/utilization/clients/stats/reset admin actions
  |
  | MSP binary request/response
  v
MSPSerial (serial or TCP to FC)
  |
  | MSP frames V1/V2
  v
Flight controller
```

```
MSPApi (client side)
  in : app calls (get_api_version, get_fc_variant, ...)
  out: decoded dicts/list enums
  op : packs via MSPCodec, sends via transport (MSPSerial or MSPClientAPI)

MSPCodec
  in : schema JSON (msp_messages.json)
  op : pack_request / unpack_reply
  out: payload bytes <-> structured values

MSPClientAPI
  in : API calls from MSPApi
  op : wrap in JSON, send to msp_server.py, parse reply
  out: raw payload bytes + diag
```
