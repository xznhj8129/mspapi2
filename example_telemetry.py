#!/usr/bin/env python3
"""Example MSP client that talks to msp_server via MSPApi."""

from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Any, Dict

from mspapi2.client import MSPClientAPI
from mspapi2.lib import InavMSP
from mspapi2.msp_api import MSPApi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="High-level MSP API client via msp_server.py")
    parser.add_argument("--server", default="127.0.0.1:9000", help="MSP server host:port")
    parser.add_argument("--client-id", default=socket.gethostname(), help="Client identifier")
    parser.add_argument("--timeout", type=float, default=1.0, help="Per-request timeout (seconds)")
    return parser.parse_args()


def show_info(label: str, info: Dict[str, Any]) -> None:
    parts = []
    latency = info.get("latency_ms")
    if isinstance(latency, (int, float)):
        parts.append(f"latency={latency:.2f}ms")
    cache_age = info.get("cache_age_ms")
    if isinstance(cache_age, (int, float)):
        parts.append(f"cache_age={cache_age:.0f}ms")
    if info.get("cached") is not None:
        parts.append(f"cached={info.get('cached')}")
    server = info.get("server") or {}
    recon = server.get("reconnections")
    if recon is not None:
        parts.append(f"reconnects={recon}")
    sched_failed = server.get("scheduler_failed")
    if sched_failed:
        parts.append(f"scheduler_failed={sched_failed}")
    if parts:
        print(f"  {label} info: " + ", ".join(parts))


def main() -> None:
    msp_telemetry_msgs = {
        InavMSP.MSP2_INAV_ANALOG: 5,
        InavMSP.MSP2_INAV_STATUS: 5,
        InavMSP.MSP_RC: 5,
        InavMSP.MSP_ATTITUDE: 5,
        InavMSP.MSP_ALTITUDE: 5,
        InavMSP.MSP_RAW_IMU: 5,
    }

    args = parse_args()
    if ":" not in args.server:
        raise ValueError("Server must be HOST:PORT")
    host, port_str = args.server.rsplit(":", 1)
    port = int(port_str)
    transport = MSPClientAPI(host, port, client_id=args.client_id)
    api = MSPApi(port=None, serial_transport=transport)
    api.open()
    try:

        print("Waiting for INAV FC...")
        while True:
            info, fc_variant = api.get_fc_variant()
            show_info("MSP_FC_VARIANT", info)
            if fc_variant["fcVariantIdentifier"] == "INAV":
                print("Connected to INAV flight controller.")
                break
            time.sleep(0.5)

        schedules = transport.sched_get()
        info = api.info_from_diag(transport.last_diag, None)
        print("Scheduler:", schedules)
        show_info("sched_get", info)

        for stream in msp_telemetry_msgs:
            if stream.name in schedules:
                print(f"{stream.name} timer active")
                continue
            t = 1.0 / msp_telemetry_msgs[stream]
            transport.sched_set(int(stream.value), delay=t)
            info = api.info_from_diag(transport.last_diag, stream)
            print(f"Adding {stream.name} timer every {t}s")
            show_info("sched_set", info)


        info, sensor_config = api.get_sensor_config()
        print("\nSensor configuration:", sensor_config)
        show_info("MSP_SENSOR_CONFIG", info)

        info, rx_config = api.get_rx_config()
        print("\nRX config:", rx_config)
        show_info("MSP_RX_CONFIG", info)

        info, rx_map = api.get_rx_map()
        print("\nRX map:", rx_map)
        show_info("MSP_RX_MAP", info)

        info, mode_ranges = api.get_mode_ranges()
        print("\nMode ranges:")
        for entry in mode_ranges:
            print(entry)
        show_info("MSP_MODE_RANGES", info)

        while True:
            telemetry = transport.sched_data(*msp_telemetry_msgs.keys())
            info = api.info_from_diag(transport.last_diag, None)
            print("\nScheduled telemetry snapshot:")
            for code_int, entry in telemetry.items():
                name = InavMSP(code_int).name
                interval = entry.get("interval")
                ts = entry.get("time")
                data = entry.get("data")
                print(f"{name} @ interval={interval}s time={ts}: {data}")
            show_info("sched_data", info)
            time.sleep(1)

    finally:
        api.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
