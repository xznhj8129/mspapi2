#!/usr/bin/env python3
"""Example MSP client that talks to msp_server via MSPApi."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Any, Dict

from mspapi2.lib import InavEnums, InavMSP
from mspapi2.msp_api import MSPApi, MSPServerTransport


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="High-level MSP API client via msp_server.py")
    parser.add_argument("--server", default="127.0.0.1:9000", help="MSP server host:port")
    parser.add_argument("--client-id", default="mspclient", help="Client identifier")
    parser.add_argument("--timeout", type=float, default=1.0, help="Per-request timeout (seconds)")
    return parser.parse_args()



def main() -> None:
    connected = False
    default_tele_hz = 5
    msp_telemetry_msgs = {
        InavMSP.MSP2_INAV_ANALOG: default_tele_hz,
        InavMSP.MSP2_INAV_STATUS: default_tele_hz,
        #InavMSP.MSP_MOTOR: default_tele_hz,
        InavMSP.MSP_RC: default_tele_hz,
        InavMSP.MSP_ATTITUDE: default_tele_hz,
        InavMSP.MSP_ALTITUDE: default_tele_hz,
        InavMSP.MSP_RAW_IMU: default_tele_hz
    }


    args = parse_args()
    if ":" not in args.server:
        raise ValueError("Server must be HOST:PORT")
    host, port_str = args.server.rsplit(":", 1)
    port = int(port_str)
    transport = MSPServerTransport(host, port, client_id=args.client_id)
    api = MSPApi(port=None, serial_transport=transport)
    api.open()
    try:

        while not connected:
            info, fc_variant = api.get_fc_variant()
            if fc_variant["fcVariantIdentifier"] == "INAV":
                connected = True
            else:
                time.sleep(0.5)

        info, schedules = api.sched_get()
        print("Scheduler:", schedules)

        for stream in msp_telemetry_msgs:
            if stream.name not in schedules:
                t = 1.0 / msp_telemetry_msgs[stream]
                info, _ = api.sched_set(stream, delay= t)
                print(f"Adding {stream.name} timer every {t}s")
            else:
                print(f"{stream.name} timer active")


        info, sensor_config = api.get_sensor_config()
        print("\nSensor configuration:", sensor_config)


        info, rx_config = api.get_rx_config()
        #if rx_config['receiverType'] != InavEnums.rxReceiverType_e.RX_TYPE_MSP:
        #    print('Error: Receiver is not MSP')
        print("\nRX config:", rx_config)

        info, rx_map = api.get_rx_map()
        print("\nRX map:", rx_map)

        info, mode_ranges = api.get_mode_ranges()
        print("\nMode ranges:")
        for entry in mode_ranges:
            print(entry)

        while 1:
            info, status = api.get_inav_status()
            print("\nINAV status:", status)

            info, analog = api.get_inav_analog()
            print("\nAnalog readings:", analog)

            info, attitude = api.get_attitude()
            print("\nAttitude:", attitude)

            info, altitude = api.get_altitude()
            print("\nAltitude:", altitude)

            info, imu = api.get_imu()
            print("\nIMU summary:", imu)

            info, rc_channels = api.get_rc_channels()
            print("\nRC channels:", rc_channels[:6])

            #target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
            #info, ack = api.set_rc_channels(target_channels)
            #print("SET_RAW_RC ack:", ack)

            #info, battery = api.get_battery_config()
            #print("\nBattery config:", battery)


            info, raw_gps = api.get_raw_gps()
            print("\nRaw GPS:", raw_gps)

            info, gps_stats = api.get_gps_statistics()
            print("\nGPS statistics:", gps_stats)


            info, active_modes = api.get_active_modes()
            print("Active modes:", active_modes)

            time.sleep(1)

        #info, schedules = api.sched_get()
        #print("\nCurrent scheduler:", schedules)


    finally:
        api.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
