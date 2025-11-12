from __future__ import annotations

import argparse

from lib import InavEnums
import lib.boxes as boxes
from msp_api import MSPApi
import time

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSP API demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path (ignored if --tcp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--tcp", metavar="HOST:PORT", help="Connect using TCP socket instead of serial, e.g. localhost:5760")
    parser.add_argument("--read-timeout", type=float, default=0.05, help="Read timeout in seconds")
    parser.add_argument("--write-timeout", type=float, default=0.25, help="Write timeout in seconds")
    return parser.parse_args()


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
        _, api_version = api.get_api_version()
        print("MSP API version:", api_version)

        print()
        _, fc_variant = api.get_fc_variant()
        print("Flight controller variant:", fc_variant)

        print()
        _, board_info = api.get_board_info()
        print("Board info:", board_info)

        print()
        _, sensor_cfg = api.get_sensor_config()
        print("Sensor configuration:", sensor_cfg)

        print()
        _, mode_ranges = api.get_mode_ranges()
        print("Mode ranges:")
        for entry in mode_ranges:
            print(entry)

        print()
        _, status = api.get_inav_status()
        print("INAV status:", status)

        print()
        _, analog = api.get_inav_analog()
        print("Analog readings:", analog)

        print()
        _, rx_config = api.get_rx_config()
        print("RX config:", rx_config)

        print()
        _, rx_map = api.get_rx_map()
        print("RX map:", rx_map)

        #skip this until patch, smashes the stack
        #print()
        #print("Logic conditions:")
        #for condition in api.get_logic_conditions():
        #    if condition["enabled"]:
        #        print(condition)

        print()
        _, attitude = api.get_attitude()
        print("Attitude:", attitude)

        print()
        _, altitude = api.get_altitude()
        print("Altitude:", altitude)

        print()
        _, imu = api.get_imu()
        print("IMU summary:", imu)

        print()
        _, rc_channels = api.get_rc_channels()
        print("RC channels:", rc_channels[:6])

        target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
        _, ack = api.set_rc_channels(target_channels)
        print("SET_RAW_RC ack:", ack)

        print()
        _, bat_cfg = api.get_battery_config()
        print("Battery config:", bat_cfg)

        print()
        _, gps_stats = api.get_gps_statistics()
        print("GPS statistics:", gps_stats)

        print()
        _, wp_info = api.get_waypoint_info()
        print("Waypoint info:", wp_info)

        print()
        _, raw_gps = api.get_raw_gps()
        print("Raw GPS:", raw_gps)

        print()
        _, set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
        )
        print("SET_WP ack:", set_wp_ack)

        print()
        _, waypoint = api.get_waypoint(1)
        print("Waypoint:", waypoint)

        print()
        _, nav_status = api.get_nav_status()
        print("Navigation status:", nav_status)

        _, box_ids = api.get_box_ids()
        active_mode_mask = 0
        for mode in status["activeModes"]:
            box_index = mode.get("boxIndex")
            if box_index is None:
                continue
            active_mode_mask |= 1 << box_index

        active_modes = []
        for idx, permanent_id in enumerate(box_ids):
            if not (active_mode_mask & (1 << idx)):
                continue
            mode_info = boxes.MODEBOXES.get(permanent_id, {})
            active_modes.append(
                {
                    "boxIndex": idx,
                    "permanentId": permanent_id,
                    "boxName": mode_info.get("boxName", f"UNKNOWN_{permanent_id}"),
                }
            )
        print("Active modes:", active_modes)

        """print()
        simulator_reply = api.set_simulator(
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
