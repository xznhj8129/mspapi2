from __future__ import annotations

import argparse
from functools import partial

from mspapi2.lib import InavEnums, InavMSP
from mspapi2.msp_api import MSPApi
from mspapi2.utils import format_nested_dict
import time
from typing import Any, Dict

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSP API demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path (ignored if --tcp or --udp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect using TCP socket instead of serial, e.g. localhost:5760")
    parser.add_argument("--udp", metavar="HOST:PORT", help="Connect using UDP socket instead of serial, e.g. localhost:27072")
    parser.add_argument("--force-mspv2", action="store_true", help="Force all requests to MSPv2 framing")
    parser.add_argument("--read-timeout", type=float, default=10.0, help="Read timeout in milliseconds")
    parser.add_argument("--write-timeout", type=float, default=250.0, help="Write timeout in milliseconds")
    return parser.parse_args()


def show_info(info: Dict[str, Any]) -> None:
    parts = []
    latency = info.get("latency_ms")
    if isinstance(latency, (int, float)):
        parts.append(f"latency={latency:.2f}ms")
    if info.get("cached") is not None:
        parts.append(f"cached={info.get('cached')}")
    cache_age = info.get("cache_age_ms")
    if isinstance(cache_age, (int, float)):
        parts.append(f"cache_age={cache_age:.0f}ms")
    attempt = info.get("attempt")
    if attempt is not None:
        parts.append(f"attempt={attempt}")
    transport = info.get("transport")
    if transport:
        parts.append(f"transport={transport}")
    if parts:
        print(f", ".join(parts))


def main() -> None:
    args = parse_args()
    if args.tcp and args.udp:
        raise ValueError("Provide only one of --tcp or --udp")
    pp = partial(format_nested_dict, start_indent=1)
    port = None if (args.tcp or args.udp) else args.port
    with MSPApi(
        port=port,
        baudrate=args.baudrate,
        read_timeout_ms=args.read_timeout,
        write_timeout_ms=args.write_timeout,
        tcp_endpoint=args.tcp,
        udp_endpoint=args.udp,
        force_msp_v2=args.force_mspv2,
    ) as api:
        print()
        api_version = api.get_api_version()
        print("MSP API version:\n" + pp(api_version))
        show_info(api.info)

        print()
        fc_variant = api.get_fc_variant()
        print("Flight controller variant:\n" + pp(fc_variant))
        show_info(api.info)

        print()
        board_info = api.get_board_info()
        print("Board info:\n" + pp(board_info))
        show_info(api.info)

        print()
        sensor_cfg = api.get_sensor_config()
        print("Sensor configuration:\n" + pp(sensor_cfg))
        show_info(api.info)

        print()
        mode_ranges = api.get_mode_ranges()
        print("Mode ranges:\n" + pp(mode_ranges))
        show_info(api.info)

        print()
        status = api.get_inav_status()
        print("INAV status:\n" + pp(status))
        show_info(api.info)

        print()
        analog = api.get_inav_analog()
        print("Analog readings:\n" + pp(analog))
        show_info(api.info)

        print()
        rx_config = api.get_rx_config()
        print("RX config:\n" + pp(rx_config))
        show_info(api.info)

        print()
        logic_condition = api.get_logic_condition(0)
        print("Logic condition[0]:\n" + pp(logic_condition))
        show_info(api.info)

        print()
        try:
            api._request_raw(InavMSP.MSP_SET_ACC_TRIM, b"")
        except Exception as exc:
            print(f"Deprecated/invalid MSP example (MSP_SET_ACC_TRIM): {exc}")

        try:
            api._request_raw(InavMSP.MSP2_INAV_GLOBAL_FUNCTIONS, b"")
        except Exception as exc:
            print(f"Unimplemented MSP example (MSP2_INAV_GLOBAL_FUNCTIONS): {exc}")

        print()
        rx_map = api.get_rx_map()
        print("RX map:\n" + pp(rx_map))
        show_info(api.info)

        print()
        attitude = api.get_attitude()
        print("Attitude:\n" + pp(attitude))
        show_info(api.info)

        print()
        altitude = api.get_altitude()
        print("Altitude:\n" + pp(altitude))
        show_info(api.info)

        print()
        imu = api.get_imu()
        print("IMU summary:\n" + pp(imu))
        show_info(api.info)

        print()
        rc_channels = api.get_rc_channels()
        print("RC channels:\n" + pp(rc_channels[:6] if rc_channels else []))
        show_info(api.info)

        print()
        target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
        ack = api.set_rc_channels(target_channels)
        print("SET_RAW_RC ack:\n" + pp(ack))
        show_info(api.info)

        print()
        bat_cfg = api.get_battery_config()
        print("Battery config:\n" + pp(bat_cfg))
        show_info(api.info)

        print()
        gps_stats = api.get_gps_statistics()
        print("GPS statistics:\n" + pp(gps_stats))
        show_info(api.info)

        print()
        wp_info = api.get_waypoint_info()
        print("Waypoint info:\n" + pp(wp_info))
        show_info(api.info)

        print()
        raw_gps = api.get_raw_gps()
        print("Raw GPS:\n" + pp(raw_gps))
        show_info(api.info)

        print()
        set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
            param1 = 0,
            param2 = 0,
            param3 = 0,
            flag = 0
        )
        print("SET_WP ack:\n" + pp(set_wp_ack))
        show_info(api.info)

        print()
        waypoint = api.get_waypoint(1)
        print("Waypoint:\n" + pp(waypoint))
        show_info(api.info)

        print()
        nav_status = api.get_nav_status()
        print("Navigation status:\n" + pp(nav_status))
        show_info(api.info)

        print()
        try:
            ack_heading = api.set_heading(heading_deg=90)
            print("SET_HEAD ack:\n" + pp(ack_heading))
            show_info(api.info)
        except Exception as exc:
            print(f"SET_HEAD failed: {exc}")

        print()
        active_modes = api.get_active_modes()
        print("Active modes:\n" + pp(active_modes))
        show_info(api.info)

        """Result:
        MSP API version:
            mspProtocolVersion: 0
            apiVersionMajor: 2
            apiVersionMinor: 5
        latency=10.29ms, attempt=1, transport=serial

        Flight controller variant:
            fcVariantIdentifier: INAV
        latency=10.14ms, attempt=1, transport=serial

        Board info:
            boardIdentifier: H743
            hardwareRevision: 0
            osdSupport: 2
            commCapabilities:
                vcp: True
                softSerial: True
            targetName: MATEKH743
        latency=20.14ms, attempt=1, transport=serial

        Sensor configuration:
            accHardware: <accelerationSensor_e.ACC_ICM42605: 8>
            baroHardware: <baroSensor_e.BARO_SPL06: 7>
            magHardware: <magSensor_e.MAG_QMC5883: 7>
            pitotHardware: <pitotSensor_e.PITOT_VIRTUAL: 4>
            rangefinderHardware: <rangefinderType_e.RANGEFINDER_NONE: 0>
            opflowHardware: <opticalFlowSensor_e.OPFLOW_NONE: 0>
        latency=19.94ms, attempt=1, transport=serial

        Mode ranges:
            0:
                mode: ARM
                boxIndex: 0
                permanentId: 0
                auxChannelIndex: 0
                pwmRange:
                    0: 900
                    1: 900
        latency=20.36ms, attempt=1, transport=serial

        INAV status:
            cycleTime: 502
            i2cErrors: 60
            sensorStatus:
                0: <sensors_e.SENSOR_GYRO: 1>
                1: <sensors_e.SENSOR_ACC: 2>
                2: <sensors_e.SENSOR_OPFLOW: 64>
                3: <sensors_e.SENSOR_GPS: 128>
            cpuLoad: 1
            profileAndBattProfile: 0
            armingFlags:
                0: <armingFlag_e.ARMING_DISABLED_HARDWARE_FAILURE: 32768>
                1: <armingFlag_e.ARMING_DISABLED_RC_LINK: 262144>
            activeModes:
                0: <BoxEnum.BOXFAILSAFE: 27>
            mixerProfile: 0
        latency=9.88ms, attempt=1, transport=serial

        Analog readings:
            batteryFlags:
                fullOnPlugIn: False
                useCapacityThreshold: False
                state: <batteryState_e.BATTERY_NOT_PRESENT: 3>
                cellCount: 0
            vbat: 0.0
            amperage: 0.02
            powerDraw: 0.0
            mAhDrawn: 0
            mWhDrawn: 0
            remainingCapacity: 0
            percentageRemaining: 0
            rssi: 0
        latency=10.10ms, attempt=1, transport=serial

        RX config:
            serialRxProvider: <rxSerialReceiverType_e.SERIALRX_CRSF: 6>
            maxCheck: 1900
            midRc: 1500
            minCheck: 1100
            spektrumSatBind: 0
            rxMinUsec: 885
            rxMaxUsec: 2115
            bfCompatRcInterpolation: 0
            bfCompatRcInterpolationInt: 0
            bfCompatAirModeThreshold: 0
            reserved1: 0
            reserved2: 0
            reserved3: 0
            bfCompatFpvCamAngle: 0
            receiverType: <rxReceiverType_e.RX_TYPE_SERIAL: 1>
        latency=10.25ms, attempt=1, transport=serial

        Logic condition[0]:
            enabled: False
            activatorId: -1
            operation: <logicOperation_e.LOGIC_CONDITION_TRUE: 0>
            operandAType: <logicOperandType_e.LOGIC_CONDITION_OPERAND_TYPE_VALUE: 0>
            operandAValue: 0
            operandBType: <logicOperandType_e.LOGIC_CONDITION_OPERAND_TYPE_VALUE: 0>
            operandBValue: 0
            flags:
        latency=9.77ms, attempt=1, transport=serial

        Deprecated/invalid MSP example (MSP_SET_ACC_TRIM): MSP code 239 (MSP_SET_ACC_TRIM)(239) unsupported (! response)
        Unimplemented MSP example (MSP2_INAV_GLOBAL_FUNCTIONS): MSP code 8228 (MSP2_INAV_GLOBAL_FUNCTIONS)(8228) unsupported (! response)

        RX map:
            0:
                name: roll
                mappedTo: 0
            1:
                name: pitch
                mappedTo: 1
            2:
                name: throttle
                mappedTo: 3
            3:
                name: yaw
                mappedTo: 2
        latency=10.17ms, attempt=1, transport=serial

        Attitude:
            roll: 2.2
            pitch: -0.9
            yaw: 0.5
        latency=10.12ms, attempt=1, transport=serial

        Altitude:
            estimatedAltitude: 0.0
            variometer: 0.0
            baroAltitude: -0.25
        latency=10.13ms, attempt=1, transport=serial

        IMU summary:
            acc:
                X: 0.017578125
                Y: 0.03515625
                Z: 1.00390625
            gyro:
                X: 0
                Y: 0
                Z: 0
            mag:
                X: 0
                Y: 0
                Z: 0
        latency=10.01ms, attempt=1, transport=serial

        RC channels:
            0: 1500
            1: 1500
            2: 1500
            3: 885
            4: 1500
            5: 1500
        latency=10.11ms, attempt=1, transport=serial

        SET_RAW_RC ack:

        latency=9.99ms, attempt=1, transport=serial

        Battery config:
            vbatScale: 1100
            vbatSource: <batVoltageSource_e.BAT_VOLTAGE_RAW: 0>
            cellCount: 0
            vbatCellDetect: 4.25
            vbatMinCell: 3.3
            vbatMaxCell: 4.2
            vbatWarningCell: 3.5
            currentOffset: 0
            currentScale: 250
            capacityValue: 0
            capacityWarning: 0
            capacityCritical: 0
            capacityUnit: <batCapacityUnit_e.BAT_CAPACITY_UNIT_MAH: 0>
        latency=10.31ms, attempt=1, transport=serial

        GPS statistics:
            lastMessageDt: 0.0
            errors: 0.0
            timeouts: 43.0
            hdop: 99.99
            eph: 99.99
            epv: 99.99
        latency=9.97ms, attempt=1, transport=serial

        Waypoint info:
            wpCapabilities: 0
            maxWaypoints: 120
            missionValid: False
            waypointCount: 1
        latency=10.11ms, attempt=1, transport=serial

        Raw GPS:
            fixType: <gpsFixType_e.GPS_NO_FIX: 0>
            numSat: 0
            latitude: 0.0
            longitude: 0.0
            altitude: 0.0
            speed: 0.0
            groundCourse: 0.0
        latency=10.14ms, attempt=1, transport=serial

        SET_WP ack:

        latency=9.81ms, attempt=1, transport=serial

        Waypoint:
            waypointIndex: 1
            action: <navWaypointActions_e.NAV_WP_ACTION_WAYPOINT: 1>
            latitude: 1.234
            longitude: 2.345
            altitude: 15.0
            param1: 0
            param2: 0
            param3: 0
            flag: 0
        latency=10.06ms, attempt=1, transport=serial

        Navigation status:
            navMode: <navSystemStatus_Mode_e.MW_GPS_MODE_NONE: 0>
            navState: <navigationFSMState_t.NAV_STATE_UNDEFINED: 0>
            activeWaypoint:
                action: <navWaypointActions_e.NAV_WP_ACTION_WAYPOINT: 1>
                number: 1
            navError: <navSystemStatus_Error_e.MW_NAV_ERROR_NONE: 0>
            targetHeading: 5
        latency=10.02ms, attempt=1, transport=serial

        SET_HEAD ack:

        latency=10.01ms, attempt=1, transport=serial

        Active modes:
            0: <BoxEnum.BOXFAILSAFE: 27>
        latency=10.21ms, attempt=1, transport=serial"""

        #part of branch, not implemented yet
        """print()
        local_target = api.get_local_target()
        print("Local target (NEU offsets, cm):\n" + pp(local_target))
        show_info(api.info)

        print()
        try:
            ack_local = api.set_local_target(x_cm=100.0, y_cm=0.0, z_cm=0.0)
            print("SET_LOCAL_TARGET ack:\n" + pp(ack_local))
            show_info(api.info)
        except Exception as exc:
            print(f"SET_LOCAL_TARGET failed (expected if GCSNAV/offboard not active): {exc}")

        print()
        nav_target = api.get_nav_target()
        print("NAV target (global):\n" + pp(nav_target))
        show_info(api.info)

        print()
        try:
            ack_global = api.set_global_target(
                latitude_deg=raw_gps["latitude"],
                longitude_deg=raw_gps["longitude"],
                altitude_m=None,  # keep current altitude
                altitude_datum=InavEnums.geoAltitudeDatumFlag_e.NAV_WP_TAKEOFF_DATUM,
            )
            print("SET_GLOBAL_TARGET ack:\n" + pp(ack_global))
            show_info(api.info)
        except Exception as exc:
            print(f"SET_GLOBAL_TARGET failed (expected if GCSNAV/offboard not active): {exc}")
        """

        """print()
        _, simulator_reply = api.set_simulator(
            simulator_version=1,
            flags=(
                int(InavEnums.simulatorFlags_t.HITL_ENABLE)
                | int(InavEnums.simulatorFlags_t.HITL_HAS_NEW_GPS_DATA)
                | int(InavEnums.simulatorFlags_t.HITL_AIRSPEED)
                | int(InavEnums.simulatorFlags_t.HITL_EXT_BATTERY_VOLTAGE)
            ),
            gps={
                "fix_type": InavEnums.gpsFixType_e.GPS_FIX_3D,
                "num_sat": 10,
                "lat": 37.7749,
                "lon": -122.4194,
                "alt": 35.0,
                "speed": 12.5,
                "course": 90.0,
                "vel_n": 0.4,
                "vel_e": -0.1,
                "vel_d": -0.05,
            },
            attitude={"roll": 1.5, "pitch": -0.8, "yaw": 180.0},
            acc=(0.01, -0.02, 0.98),
            gyro=(0.0, 0.0, 0.0),
            baro_pressure=1013.25,
            mag=(230, 5, -410),
            battery_voltage=12.4,
            airspeed=11.2,
            ext_flags=0,
        )
        print("SIMULATOR:", simulator_reply)"""


if __name__ == "__main__": 
    main()
