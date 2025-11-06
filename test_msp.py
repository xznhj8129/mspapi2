import struct
import time
from typing import Mapping
from lib import *
import lib.boxes as boxes
from mspcodec import MSPCodec
from msp_serial import *
codec = MSPCodec.from_json_file("lib/msp_messages.json")
fc = MSPSerial("/dev/ttyACM0", 115200, read_timeout=0.05)
fc.open()

# IMPORTANT:
# NB: No x.get("var",0), if value is invalid or missing, it's a bug and must except, do not substitute
# NB: in final payload presentation, always use same key names as payload if applicable
# NB: in final payload presentation, convert integerized unit (ie: cm, deci-degrees) into float customary units (ie: m, degrees)
# NB: do not "try except" enum gets, if value is missing it's a bug and must except
# NB: return raw enum type, not pure value/str: vbat_source = InavEnums.batVoltageSource_e(rep.get("vbatSource"))

try:
    print()
    code, payload = fc.request(InavMSP.MSP_API_VERSION)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    # ex: {'mspProtocolVersion': 0, 'apiVersionMajor': 2, 'apiVersionMinor': 5}

    print()
    code, payload = fc.request(InavMSP.MSP_FC_VARIANT)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    #ex: {'fcVariantIdentifier': b'INAV'}
    variant = rep.get("fcVariantIdentifier").rstrip(b"\x00").decode("ascii", errors="ignore")
    print("FC variant:", variant)

    # TODO: Implement test for message: MSP_FC_VERSION

    # TODO: Implement test for message: MSP_BUILD_INFO

    print()
    code, payload = fc.request(InavMSP.MSP_BOARD_INFO)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    board_identifier_raw = rep.get("boardIdentifier")
    target_name_raw = rep.get("targetName")
    comm_capabilities = rep.get("commCapabilities")
    hardware_revision = rep.get("hardwareRevision")
    osd_support = rep.get("osdSupport")
    board_identifier = board_identifier_raw.rstrip(b"\x00").decode("ascii", errors="ignore")
    target_name = target_name_raw.rstrip(b"\x00").decode("ascii", errors="ignore")
    comm_capabilities_summary = {
        "vcp": bool(comm_capabilities & 0x1),
        "softSerial": bool(comm_capabilities & 0x2),
    }
    print("Board info:", {
        "boardIdentifier": board_identifier,
        "hardwareRevision": hardware_revision if hardware_revision else 0,
        "osdSupport": osd_support,
        "commCapabilities": comm_capabilities_summary,
        "targetName": target_name,
    })
    # ex: {'boardIdentifier': 'JH45', 'hardwareRevision': 0, 'osdSupport': 2, 'commCapabilities': {'vcp': True, 'softSerial': False}, 'targetName': 'JHEMCUF405'}

    # TODO: Implement test for message: MSP_UID

    # TODO: Implement test for message: MSP_NAME

    print()
    code, payload = fc.request(InavMSP.MSP_SENSOR_CONFIG)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    sensor_enums = {
        "accHardware": InavEnums.accelerationSensor_e,
        "baroHardware": InavEnums.baroSensor_e,
        "magHardware": InavEnums.magSensor_e,
        "pitotHardware": InavEnums.pitotSensor_e,
        "rangefinderHardware": InavEnums.rangefinderType_e,
        "opflowHardware": InavEnums.opticalFlowSensor_e,
    }
    sensor_config_summary = {}
    for field, enum_cls in sensor_enums.items():
        raw_value = rep.get(field)
        if raw_value is None:
            continue
        sensor_config_summary[field] = enum_cls(raw_value)
    print("Sensor config summary:", sensor_config_summary)
    # ex: {'accHardware': 'ACC_ICM42605', 'baroHardware': 'BARO_SPL06', 'magHardware': 'MAG_NONE', 'pitotHardware': 'PITOT_NONE', 'rangefinderHardware': 'RANGEFINDER_NONE', 'opflowHardware': 'OPFLOW_NONE'}

    print()
    code, payload = fc.request(InavMSP.MSP_BOXIDS)
    rep = codec.unpack_reply(code, payload)
    box_ids = rep.get("boxIds")
    box_index_map = {pid: idx for idx, pid in enumerate(box_ids)}
    print("Got", InavMSP(code).name, rep)
    #Got MSP_BOXIDS {'boxIds': [0, 51, 61, 1, 2, 35, 5, 8, 6, 7, 32, 11, 10, 28, 53, 45, 30, 31, 55, 59, 46, 3, 13, 60, 19, 26, 27, 39, 40, 41, 42, 43, 44, 52, 62, 63, 65, 66, 67]}

    print()
    code, payload = fc.request(InavMSP.MSP_MODE_RANGES)
    mode_entries = codec.unpack_reply(code, payload)
    min_pwm = InavDefines.CHANNEL_RANGE_MIN
    step_width = InavDefines.CHANNEL_RANGE_STEP_WIDTH
    mode_ranges = []

    for entry in mode_entries:
        mode_id = entry.get("modePermanentId")
        aux_index = entry.get("auxChannelIndex")
        start_step = entry.get("rangeStartStep")
        end_step = entry.get("rangeEndStep")

        if mode_id == 0:
            continue
        permanent_id = mode_id
        mode_info = boxes.MODEBOXES.get(permanent_id)
        box_name = mode_info["boxName"] if mode_info else f"UNKNOWN_{permanent_id}"
        box_index = box_index_map.get(permanent_id)
        mode_ranges.append({
            "mode": box_name,
            "boxIndex": box_index,
            "permanentId": permanent_id,
            "auxChannelIndex": aux_index,
            "pwmRange": (
                min_pwm + start_step * step_width,
                min_pwm + end_step * step_width,
            ),
        })
    print("Got", InavMSP(code).name)
    for i in mode_ranges:
        print(i)
    # ex: {'mode': 'ANGLE', 'auxChannelIndex': 1, 'pwmRange': (1300, 1700)} ...

    print()
    code, payload = fc.request(InavMSP.MSP2_INAV_STATUS)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    #ex: {'cycleTime': 503, 'i2cErrors': 0, 'sensorStatus': 32899, 'cpuLoad': 11, 'profileAndBattProfile': 1, 'armingFlags': 297216, 'activeModes': 51606716424, 'mixerProfile': 1}
    activeModes = rep['activeModes']
    sensorStatus = rep['sensorStatus']
    armingflags = rep.get('armingFlags')

    armingFlagsDecoded = [
        flag.name
        for flag in InavEnums.armingFlag_e
        if flag is not InavEnums.armingFlag_e.ARMING_DISABLED_ALL_FLAGS and (armingflags & flag.value)
    ]
    print('Arming flags:',armingFlagsDecoded)
    # ex: ['ARMING_DISABLED_NOT_LEVEL', 'ARMING_DISABLED_NAVIGATION_UNSAFE', 'ARMING_DISABLED_HARDWARE_FAILURE', 'ARMING_DISABLED_RC_LINK']

    # Active modes decoding
    active_modes = []
    for idx, permanent_id in enumerate(box_ids):
        if not (activeModes & (1 << idx)):
            continue
        mode_info = boxes.MODEBOXES.get(permanent_id, {})
        active_modes.append({
            "boxIndex": idx,
            "permanentId": permanent_id,
            "boxName": mode_info.get("boxName", f"UNKNOWN_{permanent_id}"),
        })
    print('Active modes:', active_modes)
    # ex:  [{'boxIndex': 3, 'permanentId': 1, 'boxName': 'ANGLE'}, {'boxIndex': 26, 'permanentId': 27, 'boxName': 'FAILSAFE'}, {'boxIndex': 34, 'permanentId': 62, 'boxName': 'MIXER PROFILE 2'}, {'boxIndex': 35, 'permanentId': 63, 'boxName': 'MIXER TRANSITION'}]

    sensorStatusDecoded = [
        sensor
        for sensor in InavEnums.sensors_e
        if sensorStatus & sensor.value
    ]
    print('Sensor status:', sensorStatusDecoded)

    print()
    code, payload = fc.request(InavMSP.MSP2_INAV_ANALOG)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    battery_flags = rep.get("batteryFlags")
    battery_state_bits = (battery_flags >> 2) & 0x3
    battery_state = InavEnums.batteryState_e(battery_state_bits)
    battery_flags_decoded = {
        "fullOnPlugIn": bool(battery_flags & 0x1),
        "useCapacityThreshold": bool(battery_flags & 0x2),
        "state": battery_state,
        "cellCount": battery_flags >> 4,
    }
    analog_summary = {
        "vbat": rep.get("vbat") / 100.0,
        "amperage": rep.get("amperage") / 100.0,
        "powerDraw": rep.get("powerDraw") / 1000.0,
        "mAhDrawn": rep.get("mAhDrawn"),
        "mWhDrawn": rep.get("mWhDrawn"),
        "remainingCapacity": rep.get("remainingCapacity"),
        "percentRemaining": rep.get("percentageRemaining"),
        "rssi": rep.get("rssi"),
    }
    print("Battery flags:", battery_flags_decoded)
    # ex: {'fullOnPlugIn': False, 'useCapacityThreshold': False, 'state': 'BATTERY_NOT_PRESENT', 'cellCount': 0}
    print("Analog summary:", analog_summary)
    # ex: {'vbat': 0.02, 'amperage': 13.6, 'powerDraw': 0.027, 'mAhDrawn': 54153, 'mWhDrawn': 1059, 'remainingCapacity': 0, 'percentRemaining': 0, 'rssi': 0}

    print()
    code, payload = fc.request(InavMSP.MSP_RX_CONFIG)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    rx_config = {
        "serialRxProvider": InavEnums.rxSerialReceiverType_e(rep["serialRxProvider"]),
        "maxCheck": rep["maxCheck"],
        "midRc": rep["midRc"],
        "minCheck": rep["minCheck"],
        "spektrumSatBind": rep["spektrumSatBind"],
        "rxMinUsec": rep["rxMinUsec"],
        "rxMaxUsec": rep["rxMaxUsec"],
        "bfCompatRcInterpolation": rep["bfCompatRcInterpolation"],
        "bfCompatRcInterpolationInt": rep["bfCompatRcInterpolationInt"],
        "bfCompatAirModeThreshold": rep["bfCompatAirModeThreshold"],
        "reserved1": rep["reserved1"],
        "reserved2": rep["reserved2"],
        "reserved3": rep["reserved3"],
        "bfCompatFpvCamAngle": rep["bfCompatFpvCamAngle"],
        "receiverType": InavEnums.rxReceiverType_e(rep["receiverType"]),
    }
    print("RX config:", rx_config)

    print()
    code, payload = fc.request(InavMSP.MSP2_INAV_LOGIC_CONDITIONS)
    reps = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name)
    logic_conditions = []
    for entry in reps or []:
        flags_raw = entry["flags"]
        logic_conditions.append({
            "enabled": bool(entry["enabled"]),
            "activatorId": None if entry["activatorId"] == 0xFF else entry["activatorId"],
            "operation": InavEnums.logicOperation_e(entry["operation"]),
            "operandAType": InavEnums.logicOperandType_e(entry["operandAType"]),
            "operandAValue": entry["operandAValue"],
            "operandBType": InavEnums.logicOperandType_e(entry["operandBType"]),
            "operandBValue": entry["operandBValue"],
            "flags": [
                flag
                for flag in InavEnums.logicConditionFlags_e
                if flags_raw & flag.value
            ],
        })
    print("Logic conditions:")
    for lc in logic_conditions:
        if lc["enabled"]:
            print(lc)

    # TODO: Implement test for message: MSP_MOTOR

    print()
    code, payload = fc.request(InavMSP.MSP_ATTITUDE)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    atti = {axis: rep.get(f"{axis}") / 10.0 for axis in ["roll","pitch","yaw"]}
    print("Attitide:", atti)
    # ex: {'roll': 68.9, 'pitch': 84.0, 'yaw': 4.8}

    print()
    code, payload = fc.request(InavMSP.MSP_ALTITUDE)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    alt = {h: rep.get(f"{h}") / 100.0 for h in ["estimatedAltitude","variometer","baroAltitude"]}
    print("Altitude:", alt)
    # ex: {'estimatedAltitude': 0, 'variometer': 0, 'baroAltitude': -29.69}

    print()
    code, payload = fc.request(InavMSP.MSP_RAW_IMU)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    axes = ("X", "Y", "Z")
    imu_summary = {
        "accelG": {axis: rep.get(f"acc{axis}") / 512.0 for axis in axes},
        "gyroDPS": {axis: rep.get(f"gyro{axis}") for axis in axes},
        "magRaw": {axis: rep.get(f"mag{axis}") for axis in axes},
    }
    print("IMU summary:", imu_summary)
    # ex: {'accelG': {'X': -0.98046875, 'Y': 0.095703125, 'Z': 0.037109375}, 'gyroDPS': {'X': 0, 'Y': -1, 'Z': 0}, 'magRaw': {'X': 0, 'Y': 0, 'Z': 0}}

    print()
    code, payload = fc.request(InavMSP.MSP_RC)
    channel_width = 2  
    channel_count = len(payload) // channel_width if payload else 0
    rc_channels = list(struct.unpack(f"<{channel_count}H", payload)) if channel_count else []
    print("Got", InavMSP(code).name, {"rcChannels": rc_channels})
    #ex: {'rcChannels': [1500, 1500, 1500, 885, 1775, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]}

    print()
    target_rc_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
    rc_payload = struct.pack(f"<{len(target_rc_channels)}H", *target_rc_channels)
    code, payload = fc.request(InavMSP.MSP_SET_RAW_RC, rc_payload)
    ack = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, {"rcChannels": target_rc_channels, "ack": ack})
    #ex: {'rcChannels': [1500, 1500, 1500, 885, 1775, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]}

    print()
    code, payload = fc.request(InavMSP.MSP2_INAV_BATTERY_CONFIG)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    vbat_source = InavEnums.batVoltageSource_e(rep.get("vbatSource"))
    capacity_unit = InavEnums.batCapacityUnit_e(rep.get("capacityUnit"))
    voltage_keys = ("vbatCellDetect", "vbatMinCell", "vbatMaxCell", "vbatWarningCell")
    cell_voltages = {key: rep.get(key) / 100.0 for key in voltage_keys}
    battery_config_summary = {
        "vbatScale": rep.get("vbatScale"),
        "voltageSource": vbat_source,
        "cellCount": rep.get("cellCount"),
        "cellVoltagesV": cell_voltages,
        "currentOffset": rep.get("currentOffset"),
        "currentScale": rep.get("currentScale"),
        "capacityValue": rep.get("capacityValue"),
        "capacityWarning": rep.get("capacityWarning"),
        "capacityCritical": rep.get("capacityCritical"),
        "capacityUnit": capacity_unit,
    }
    print("Battery config summary:", battery_config_summary)
    # ex: {'rcChannels': [1500, 1500, 1500, 885, 1775, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500], 'ack': {}}

    print()
    code, payload = fc.request(InavMSP.MSP_GPSSTATISTICS)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    gps_stats_summary = {
        "lastMessageSeconds": rep.get("lastMessageDt") / 1000.0,
        "errorRate": rep.get("errors") / max(rep.get("packetCount", 1), 1),
        "timeoutRate": rep.get("timeouts") / max(rep.get("packetCount", 1), 1),
        "hdop": rep.get("hdop") / 100.0,
        "horizontalAccM": rep.get("eph") / 100.0,
        "verticalAccM": rep.get("epv") / 100.0,
    }
    print("GPS statistics summary:", gps_stats_summary)
    # ex: {'vbatScale': 1100, 'voltageSource': 'BAT_VOLTAGE_RAW', 'cellCount': 0, 'cellVoltagesV': {'vbatCellDetect': 4.25, 'vbatMinCell': 3.3, 'vbatMaxCell': 4.2, 'vbatWarningCell': 3.5}, 'currentOffsetmV': 0, 'currentScale': 250, 'capacityValue': 0, 'capacityWarning': 0, 'capacityCritical': 0, 'capacityUnit': 'BAT_CAPACITY_UNIT_MAH'}

    print()
    code, payload = fc.request(InavMSP.MSP_WP_GETINFO)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    mission_valid = bool(rep.get("missionValid"))
    wp_info_summary = {
        "wpCapabilities": rep.get("wpCapabilities"),
        "maxWaypoints": rep.get("maxWaypoints"),
        "missionValid": mission_valid,
        "waypointCount": rep.get("waypointCount"),
    }
    if mission_valid:
        wp_info_summary["waypointsRemaining"] = max(
            (rep.get("maxWaypoints") or 0) - (rep.get("waypointCount") or 0), 0
        )
    print("Waypoint info summary:", wp_info_summary)
    # ex: {'capabilities': 0, 'maxWaypoints': 120, 'missionValid': False, 'waypointCount': 1}

    print()
    code, payload = fc.request(InavMSP.MSP_RAW_GPS)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    raw_gps = {
        "fixType": InavEnums.gpsFixType_e(rep.get('fixType')),
        "numSat": rep.get("numSat"),
        "latitude": rep.get('latitude') / 1e7,
        "longitude": rep.get('longitude') / 1e7,
        "altitude": rep.get('altitude') / 100.0,
        "speed": rep.get('speed') / 100.0,
        "groundCourse": rep.get('speed') / 10.0,

    }
    print(raw_gps)
    # ex: {'fixType': 0, 'numSat': 0, 'latitude': 0, 'longitude': 0, 'altitude': 0, 'speed': 0, 'groundCourse': 0, 'hdop': 9999}

    print()
    packed = codec.pack_request(InavMSP.MSP_SET_WP, {
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
    code, payload = fc.request(InavMSP.MSP_SET_WP, packed)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    # ex: {'fixType': 0, 'numSat': 0, 'latitude': 0, 'longitude': 0, 'altitude': 0, 'speed': 0, 'groundCourse': 0, 'hdop': 9999}

    # Test MSP_WP
    print()
    packed = codec.pack_request(InavMSP.MSP_WP, {
        'waypointIndex': 1
    })
    code, payload = fc.request(InavMSP.MSP_WP, packed)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    # ex: {'waypointIndex': 1, 'action': 1, 'latitude': 12340000, 'longitude': 23450000, 'altitude': 1500, 'param1': 0, 'param2': 0, 'param3': 0, 'flag': 0}

    print()
    code, payload = fc.request(InavMSP.MSP_NAV_STATUS)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    nav_status_summary = {
        "mode": InavEnums.navSystemStatus_Mode_e(rep.get("navMode")),
        "state": InavEnums.navigationFSMState_t(rep.get("navState")),
        "activeWaypoint": {
            "action": InavEnums.navWaypointActions_e(rep.get("activeWpAction")),
            "number": rep.get("activeWpNumber"),
        },
        "error": InavEnums.navSystemStatus_Error_e(rep.get("navError")),
        "targetHeadingDeg": rep.get("targetHeading"),
    }
    print("Navigation status summary:", nav_status_summary)
    # ex: {'mode': 'MW_GPS_MODE_NONE', 'state': 'NAV_STATE_UNDEFINED', 'activeWaypoint': {'action': 'NAV_WP_ACTION_WAYPOINT', 'number': 1}, 'error': 'MW_NAV_ERROR_NONE', 'targetHeadingDeg': 353}

    #TODO: test MSP_RX_CONFIG

finally:
    fc.close()
