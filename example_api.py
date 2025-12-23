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
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path (ignored if --tcp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect using TCP socket instead of serial, e.g. localhost:5760")
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
    pp = partial(format_nested_dict, start_indent=1)
    port = None if args.tcp else args.port
    with MSPApi(
        port=port,
        baudrate=args.baudrate,
        read_timeout_ms=args.read_timeout,
        write_timeout_ms=args.write_timeout,
        tcp_endpoint=args.tcp,
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
        logic_info, logic_condition = api.get_logic_condition(0)
        print("Logic condition[0]:\n" + pp(logic_condition))
        show_info(logic_info)

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
        print("Raw GPS:", pp(raw_gps))
        show_info(api.info)

        print()
        set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
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
        active_modes = api.get_active_modes()
        print("Active modes:\n" + pp(active_modes))
        show_info(api.info)

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
