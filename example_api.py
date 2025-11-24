from __future__ import annotations

import argparse

from mspapi2.lib import InavEnums
from mspapi2.msp_api import MSPApi
import time
from typing import Any, Dict

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSP API demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path (ignored if --tcp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect using TCP socket instead of serial, e.g. localhost:5760")
    parser.add_argument("--read-timeout", type=float, default=0.05, help="Read timeout in seconds")
    parser.add_argument("--write-timeout", type=float, default=0.25, help="Write timeout in seconds")
    return parser.parse_args()


def show_info(label: str, info: Dict[str, Any]) -> None:
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
        print(f"  {label}: " + ", ".join(parts))


def main() -> None:
    args = parse_args()
    port = None if args.tcp else args.port
    with MSPApi(
        port=port,
        baudrate=args.baudrate,
        read_timeout=args.read_timeout,
        write_timeout=args.write_timeout,
        tcp_endpoint=args.tcp,
    ) as api:
        print()
        info, api_version = api.get_api_version()
        print("MSP API version:", api_version)
        show_info("MSP_API_VERSION", info)

        print()
        info, fc_variant = api.get_fc_variant()
        print("Flight controller variant:", fc_variant)
        show_info("MSP_FC_VARIANT", info)

        print()
        info, board_info = api.get_board_info()
        print("Board info:", board_info)
        show_info("MSP_BOARD_INFO", info)

        print()
        info, sensor_cfg = api.get_sensor_config()
        print("Sensor configuration:", sensor_cfg)
        show_info("MSP_SENSOR_CONFIG", info)

        print()
        info, mode_ranges = api.get_mode_ranges()
        print("Mode ranges:")
        for entry in mode_ranges:
            print(entry)
        show_info("MSP_MODE_RANGES", info)

        print()
        info, status = api.get_inav_status()
        print("INAV status:", status)
        show_info("MSP2_INAV_STATUS", info)

        print()
        info, analog = api.get_inav_analog()
        print("Analog readings:", analog)
        show_info("MSP2_INAV_ANALOG", info)

        print()
        info, rx_config = api.get_rx_config()
        print("RX config:", rx_config)
        show_info("MSP_RX_CONFIG", info)

        print()
        info, rx_map = api.get_rx_map()
        print("RX map:", rx_map)
        show_info("MSP_RX_MAP", info)

        print()
        info, attitude = api.get_attitude()
        print("Attitude:", attitude)
        show_info("MSP_ATTITUDE", info)

        print()
        info, altitude = api.get_altitude()
        print("Altitude:", altitude)
        show_info("MSP_ALTITUDE", info)

        print()
        info, imu = api.get_imu()
        print("IMU summary:", imu)
        show_info("MSP_RAW_IMU", info)

        print()
        info, rc_channels = api.get_rc_channels()
        print("RC channels:", rc_channels[:6])
        show_info("MSP_RC", info)

        target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
        info, ack = api.set_rc_channels(target_channels)
        print("SET_RAW_RC ack:", ack)
        show_info("MSP_SET_RAW_RC", info)

        print()
        info, bat_cfg = api.get_battery_config()
        print("Battery config:", bat_cfg)
        show_info("MSP2_INAV_BATTERY_CONFIG", info)

        print()
        info, gps_stats = api.get_gps_statistics()
        print("GPS statistics:", gps_stats)
        show_info("MSP_GPSSTATISTICS", info)

        print()
        info, wp_info = api.get_waypoint_info()
        print("Waypoint info:", wp_info)
        show_info("MSP_WP_GETINFO", info)

        print()
        info, raw_gps = api.get_raw_gps()
        print("Raw GPS:", raw_gps)
        show_info("MSP_RAW_GPS", info)

        print()
        info, set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
        )
        print("SET_WP ack:", set_wp_ack)
        show_info("MSP_SET_WP", info)

        print()
        info, waypoint = api.get_waypoint(1)
        print("Waypoint:", waypoint)
        show_info("MSP_WP", info)

        print()
        info, nav_status = api.get_nav_status()
        print("Navigation status:", nav_status)
        show_info("MSP_NAV_STATUS", info)

        info, active_modes = api.get_active_modes()
        print("Active modes:", active_modes)
        show_info("get_active_modes", info)

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
