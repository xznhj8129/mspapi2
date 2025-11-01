import struct
import time
from lib import *
import lib.boxes as boxes
from mspcodec import MSPCodec
from mspparser import *
codec = MSPCodec.from_json_file("lib/msp_messages.json")




msp = MSPSerial("/dev/ttyACM0", 115200, read_timeout=0.05)
msp.open()
try:
    # Test MSP_API_VERSION
    print()
    code, payload = msp.request(InavMSP.MSP_API_VERSION)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)

    # Test MSP_ATTITUDE
    print()
    code, payload = msp.request(InavMSP.MSP_ATTITUDE)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)

    # TODO: Implement test for message: MSP_FC_VARIANT

    # TODO: Implement test for message: MSP_FC_VERSION

    # TODO: Implement test for message: MSP_BUILD_INFO

    # TODO: Implement test for message: MSP_BOARD_INFO

    # TODO: Implement test for message: MSP_UID

    # TODO: Implement test for message: MSP_NAME

    # Test MSP_SENSOR_CONFIG
    print()
    code, payload = msp.request(InavMSP.MSP_SENSOR_CONFIG)
    rep = codec.unpack_reply(code, payload)
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
        try:
            sensor_config_summary[field] = enum_cls(raw_value).name
        except ValueError:
            sensor_config_summary[field] = f"UNKNOWN_{raw_value}"
    print("Got", InavMSP(code).name, rep)
    print("Sensor config summary:", sensor_config_summary)

    # Test MSP2_INAV_STATUS
    print()
    code, payload = msp.request(InavMSP.MSP2_INAV_STATUS)
    rep = codec.unpack_reply(code, payload)
    armingflags = rep['armingFlags']
    armingFlagsDecoded = [
        flag.name
        for flag in InavEnums.armingFlag_e
        if flag is not InavEnums.armingFlag_e.ARMING_DISABLED_ALL_FLAGS and (armingflags & flag.value)
    ]
    print("Got", InavMSP(code).name, rep)
    print('Arming flags:',armingFlagsDecoded)

    # Test MSP2_INAV_ANALOG
    print()
    code, payload = msp.request(InavMSP.MSP2_INAV_ANALOG)
    rep = codec.unpack_reply(code, payload)
    battery_flags = rep.get("batteryFlags", 0)
    battery_state_bits = (battery_flags >> 2) & 0x3
    try:
        battery_state = InavEnums.batteryState_e(battery_state_bits).name
    except ValueError:
        battery_state = f"UNKNOWN_{battery_state_bits}"
    battery_flags_decoded = {
        "fullOnPlugIn": bool(battery_flags & 0x1),
        "useCapacityThreshold": bool(battery_flags & 0x2),
        "state": battery_state,
        "cellCount": battery_flags >> 4,
    }
    analog_summary = {
        "voltageV": rep.get("vbat", 0) / 100.0,
        "currentA": rep.get("amperage", 0) / 100.0,
        "powerW": rep.get("powerDraw", 0) / 1000.0,
        "mAhDrawn": rep.get("mAhDrawn", 0),
        "mWhDrawn": rep.get("mWhDrawn", 0),
        "remainingCapacity": rep.get("remainingCapacity", 0),
        "percentRemaining": rep.get("percentageRemaining", 0),
        "rssi": rep.get("rssi", 0),
    }
    print("Got", InavMSP(code).name, rep)
    print("Battery flags:", battery_flags_decoded)
    print("Analog summary:", analog_summary)

    # TODO: Implement test for message: MSP_MOTOR

    # Test MSP_ALTITUDE
    print()
    code, payload = msp.request(InavMSP.MSP_ALTITUDE)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)

    # Test MSP_RAW_IMU
    print()
    code, payload = msp.request(InavMSP.MSP_RAW_IMU)
    rep = codec.unpack_reply(code, payload)
    axes = ("X", "Y", "Z")
    imu_summary = {
        "accelG": {axis: rep.get(f"acc{axis}", 0) / 512.0 for axis in axes},
        "gyroDegPerSec": {axis: rep.get(f"gyro{axis}", 0) for axis in axes},
        "magRaw": {axis: rep.get(f"mag{axis}", 0) for axis in axes},
    }
    print("Got", InavMSP(code).name, rep)
    print("IMU summary:", imu_summary)

    # Test MSP_MODE_RANGES
    print()
    code, payload = msp.request(InavMSP.MSP_MODE_RANGES)
    entry_struct = struct.Struct("<BBBB")
    min_pwm = InavDefines.CHANNEL_RANGE_MIN
    step_width = InavDefines.CHANNEL_RANGE_STEP_WIDTH
    mode_ranges = []
    for offset in range(0, len(payload), entry_struct.size):
        chunk = payload[offset:offset + entry_struct.size]
        if len(chunk) < entry_struct.size:
            break
        mode_id, aux_index, start_step, end_step = entry_struct.unpack(chunk)
        if mode_id == 0:
            continue
        mode_info = boxes.MODEBOXES.get(mode_id)
        box_name = mode_info["boxName"] if mode_info else f"UNKNOWN_{mode_id}"
        mode_ranges.append({
            "mode": box_name,
            "auxChannelIndex": aux_index,
            "pwmRange": (
                min_pwm + start_step * step_width,
                min_pwm + end_step * step_width,
            ),
        })
    print("Got", InavMSP(code).name)
    for i in mode_ranges:
        print(i)

    # Test MSP_RC
    print()
    code, payload = msp.request(InavMSP.MSP_RC)
    channel_width = 2  
    channel_count = len(payload) // channel_width if payload else 0
    rc_channels = list(struct.unpack(f"<{channel_count}H", payload)) if channel_count else []
    print("Got", InavMSP(code).name, {"rcChannels": rc_channels})

    # Test MSP_SET_RAW_RC
    print()
    target_rc_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
    rc_payload = struct.pack(f"<{len(target_rc_channels)}H", *target_rc_channels)
    code, payload = msp.request(InavMSP.MSP_SET_RAW_RC, rc_payload)
    ack = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, {"rcChannels": target_rc_channels, "ack": ack})

    # Test MSP2_INAV_BATTERY_CONFIG
    print()
    code, payload = msp.request(InavMSP.MSP2_INAV_BATTERY_CONFIG)
    rep = codec.unpack_reply(code, payload)
    try:
        vbat_source = InavEnums.batVoltageSource_e(rep.get("vbatSource", 0)).name
    except ValueError:
        vbat_source = f"UNKNOWN_{rep.get('vbatSource')}"
    try:
        capacity_unit = InavEnums.batCapacityUnit_e(rep.get("capacityUnit", 0)).name
    except ValueError:
        capacity_unit = f"UNKNOWN_{rep.get('capacityUnit')}"
    voltage_keys = ("vbatCellDetect", "vbatMinCell", "vbatMaxCell", "vbatWarningCell")
    cell_voltages = {key: rep.get(key, 0) / 100.0 for key in voltage_keys}
    battery_config_summary = {
        "vbatScale": rep.get("vbatScale"),
        "voltageSource": vbat_source,
        "cellCount": rep.get("cellCount"),
        "cellVoltagesV": cell_voltages,
        "currentOffsetmV": rep.get("currentOffset"),
        "currentScale": rep.get("currentScale"),
        "capacityValue": rep.get("capacityValue"),
        "capacityWarning": rep.get("capacityWarning"),
        "capacityCritical": rep.get("capacityCritical"),
        "capacityUnit": capacity_unit,
    }
    print("Got", InavMSP(code).name, rep)
    print("Battery config summary:", battery_config_summary)

    # Test MSP_GPSSTATISTICS
    print()
    code, payload = msp.request(InavMSP.MSP_GPSSTATISTICS)
    rep = codec.unpack_reply(code, payload)
    gps_stats_summary = {
        "lastMessageSeconds": rep.get("lastMessageDt", 0) / 1000.0,
        "errorRate": rep.get("errors", 0) / max(rep.get("packetCount", 1), 1),
        "timeoutRate": rep.get("timeouts", 0) / max(rep.get("packetCount", 1), 1),
        "hdop": rep.get("hdop", 0) / 100.0,
        "horizontalAccM": rep.get("eph", 0) / 100.0,
        "verticalAccM": rep.get("epv", 0) / 100.0,
    }
    print("Got", InavMSP(code).name, rep)
    print("GPS statistics summary:", gps_stats_summary)

    # TODO: Implement test for message: MSP_NAV_STATUS

    # TODO: Implement test for message: MSP_WP_GETINFO

    # Test MSP_RAW_GPS
    print()
    code, payload = msp.request(InavMSP.MSP_RAW_GPS)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)
    print(rep)

    # Test MSP_SET_WP
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
    code, payload = msp.request(InavMSP.MSP_SET_WP, packed)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)

    # Test MSP_WP
    print()
    packed = codec.pack_request(InavMSP.MSP_WP, {
        'waypointIndex': 1
    })
    code, payload = msp.request(InavMSP.MSP_WP, packed)
    rep = codec.unpack_reply(code, payload)
    print("Got", InavMSP(code).name, rep)

finally:
    msp.close()
