from __future__ import annotations

from lib import InavEnums
from msp_api import MSPApi


def main() -> None:
    with MSPApi("/dev/ttyACM0", 115200, read_timeout=0.05) as api:
        print()
        print("MSP API version:", api.get_api_version())

        print()
        print("Flight controller variant:", api.get_fc_variant())

        print()
        print("Board info:", api.get_board_info())

        print()
        print("Sensor configuration:", api.get_sensor_config())

        print()
        print("Mode ranges:")
        for entry in api.get_mode_ranges():
            print(entry)

        print()
        status = api.get_inav_status()
        print("INAV status:", status)

        print()
        analog = api.get_inav_analog()
        print("Analog readings:", analog)

        print()
        print("RX config:", api.get_rx_config())

        print()
        print("Logic conditions:")
        for condition in api.get_logic_conditions():
            if condition["enabled"]:
                print(condition)

        print()
        print("Attitude:", api.get_attitude())

        print()
        print("Altitude:", api.get_altitude())

        print()
        print("IMU summary:", api.get_imu())

        print()
        rc_channels = api.get_rc_channels()
        print("RC channels:", rc_channels)
        target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
        ack = api.set_rc_channels(target_channels)
        print("SET_RAW_RC ack:", ack)

        print()
        print("Battery config:", api.get_battery_config())

        print()
        print("GPS statistics:", api.get_gps_statistics())

        print()
        print("Waypoint info:", api.get_waypoint_info())

        print()
        print("Raw GPS:", api.get_raw_gps())

        print()
        set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
        )
        print("SET_WP ack:", set_wp_ack)

        print()
        print("Waypoint:", api.get_waypoint(1))

        print()
        print("Navigation status:", api.get_nav_status())


if __name__ == "__main__":
    main()
