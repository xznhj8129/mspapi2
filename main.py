from msp_enum import MultiWii
from inav_enums import InavEnums
from inav_defines import InavDefines
from mspcodec import MSPCodec
from mspapi import *
codec = MSPCodec.from_json_file("msp_messages.json")

msp = MSPSerial("/dev/ttyACM0", 115200, read_timeout=0.05)
msp.open()
try:
    code, payload, ver = msp.request(MultiWii.MSP_API_VERSION, b"", timeout=1.0)
    rep = codec.unpack_reply(code, payload)
    print("Got", MultiWii(code).name, rep, "via MSPv", ver)

    req_bytes = codec.pack_request(MultiWii.MSP_SET_WP, {
        'waypointIndex': 1, 
        'action': InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT, 
        'latitude': int(1.234e7), 
        'longitude': int(2.345e7),
        'altitude': 1500, 
        'param1': 0, 
        'param2': 0, 
        'param3': 0, 
        'flag': 0
    })
    code, payload, ver = msp.request(MultiWii.MSP_SET_WP, req_bytes, timeout=1.0)
    rep = codec.unpack_reply(code, payload)
    print("Got", MultiWii(code).name, rep, "via MSPv", ver)

    req_bytes = codec.pack_request(MultiWii.MSP_WP, {
        'waypointIndex': 1
    })
    code, payload, ver = msp.request(MultiWii.MSP_WP, req_bytes, timeout=1.0)
    rep = codec.unpack_reply(code, payload)
    print("Got", MultiWii(code).name, rep, "via MSPv", ver)

finally:
    msp.close()