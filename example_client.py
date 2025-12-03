#!/usr/bin/env python3
"""Example MSP client that talks to msp_server via MSPApi."""

from __future__ import annotations

import argparse
import socket
import sys
from typing import Any, Dict

from mspapi2.client import MSPClientAPI
from mspapi2.lib import InavEnums, InavMSP
from mspapi2.msp_api import MSPApi

CLIENT_ID = "testclient"

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="High-level MSP API client via msp_server.py")
    parser.add_argument("--server", default="127.0.0.1:9000", help="MSP server host:port")
    parser.add_argument("--client-id", default=CLIENT_ID, help="Client identifier")
    parser.add_argument("--timeout", type=float, default=1.0, help="Per-request timeout (seconds)")
    return parser.parse_args()


def parse_endpoint(value: str) -> tuple[str, int]:
    if ":" not in value:
        raise ValueError("Server must be HOST:PORT")
    host, port_str = value.rsplit(":", 1)
    return host, int(port_str)


def show_info(label: str, info: Dict[str, Any]) -> None:
    if not info:
        return
    parts = []
    latency = info.get("latency_ms")
    if isinstance(latency, (int, float)):
        parts.append(f"latency={latency:.2f}ms")
    cache_age = info.get("cache_age_ms")
    if isinstance(cache_age, (int, float)):
        parts.append(f"cache_age={cache_age:.0f}ms")
    cached = info.get("cached")
    if cached is not None:
        parts.append(f"cached={cached}")
    if info.get("scheduled"):
        delay = info.get("schedule_delay_s")
        parts.append(f"scheduled@{delay}s" if delay else "scheduled")
    pending = info.get("pending")
    if isinstance(pending, int):
        parts.append(f"pending={pending}")
    rate = info.get("rate") or {}
    if rate:
        cur = rate.get("current_per_sec")
        limit = rate.get("limit_per_sec")
        util = rate.get("utilization")
        throttle = rate.get("throttle_ms")
        rate_bits = []
        if cur is not None and limit is not None:
            rate_bits.append(f"{cur}/{limit}")
        if isinstance(util, (int, float)):
            rate_bits.append(f"util={util:.2f}")
        if isinstance(throttle, (int, float)) and throttle > 0:
            rate_bits.append(f"throttle={throttle:.0f}ms")
        if rate_bits:
            parts.append("rate=" + " ".join(rate_bits))
    avg = (info.get("code_stats") or {}).get("avg_ms")
    if isinstance(avg, (int, float)):
        parts.append(f"avg={avg:.2f}ms")
    rpm = (info.get("code_stats") or {}).get("requests_per_min")
    if isinstance(rpm, (int, float)):
        parts.append(f"code_rpm={rpm}")
    server = info.get("server") or {}
    srpm = server.get("requests_per_min")
    if isinstance(srpm, (int, float)):
        parts.append(f"server_rpm={srpm}")
    recon = server.get("reconnections")
    if recon is not None:
        parts.append(f"reconnects={recon}")
    queue_total = server.get("inflight_total")
    if isinstance(queue_total, int):
        parts.append(f"queue_total={queue_total}")
    sched_failed = server.get("scheduler_failed")
    if sched_failed:
        parts.append(f"scheduler_failed={sched_failed}")
    if parts:
        print(f"  {label} info: ", ", ".join(parts))


def main() -> None:
    args = parse_args()
    host, port = parse_endpoint(args.server)
    transport = MSPClientAPI(host, port, client_id=args.client_id)
    api = MSPApi(port=None, serial_transport=transport)
    api.open()
    try:

        schedules = transport.sched_get()
        info = api.info_from_diag(transport.last_diag, None)
        print("Scheduler:", schedules)
        show_info("sched_get", info)

        info, api_version = api.get_api_version()
        print("\nMSP API version:", api_version)
        show_info("MSP_API_VERSION", info)

        info, fc_variant = api.get_fc_variant()
        print("\nFlight controller variant:", fc_variant)
        show_info("MSP_FC_VARIANT", info)

        info, board = api.get_board_info()
        print("\nBoard info:", board)
        show_info("MSP_BOARD_INFO", info)

        info, sensor_config = api.get_sensor_config()
        print("\nSensor configuration:", sensor_config)
        show_info("MSP_SENSOR_CONFIG", info)

        info, mode_ranges = api.get_mode_ranges()
        print("\nMode ranges:")
        for entry in mode_ranges:
            print(entry)
        show_info("MSP_MODE_RANGES", info)

        info, status = api.get_inav_status()
        print("\nINAV status:", status)
        show_info("MSP2_INAV_STATUS", info)

        info, analog = api.get_inav_analog()
        print("\nAnalog readings:", analog)
        show_info("MSP2_INAV_ANALOG", info)

        info, rx_config = api.get_rx_config()
        print("\nRX config:", rx_config)
        show_info("MSP_RX_CONFIG", info)

        info, rx_map = api.get_rx_map()
        print("\nRX map:", rx_map)
        show_info("MSP_RX_MAP", info)

        info, attitude = api.get_attitude()
        print("\nAttitude:", attitude)
        show_info("MSP_ATTITUDE", info)

        info, altitude = api.get_altitude()
        print("\nAltitude:", altitude)
        show_info("MSP_ALTITUDE", info)

        info, imu = api.get_imu()
        print("\nIMU summary:", imu)
        show_info("MSP_RAW_IMU", info)

        info, rc_channels = api.get_rc_channels()
        print("\nRC channels:", rc_channels[:6])
        show_info("MSP_RC", info)

        target_channels = rc_channels[:] if rc_channels else [1500, 1500, 1500, 1500]
        info, ack = api.set_rc_channels(target_channels)
        print("SET_RAW_RC ack:", ack)
        show_info("MSP_SET_RAW_RC", info)

        info, battery = api.get_battery_config()
        print("\nBattery config:", battery)
        show_info("MSP2_INAV_BATTERY_CONFIG", info)

        info, gps_stats = api.get_gps_statistics()
        print("\nGPS statistics:", gps_stats)
        show_info("MSP_GPSSTATISTICS", info)

        info, waypoint_info = api.get_waypoint_info()
        print("\nWaypoint info:", waypoint_info)
        show_info("MSP_WP_GETINFO", info)

        info, raw_gps = api.get_raw_gps()
        print("\nRaw GPS:", raw_gps)
        show_info("MSP_RAW_GPS", info)

        info, set_wp_ack = api.set_waypoint(
            waypointIndex=1,
            action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT,
            latitude=1.234,
            longitude=2.345,
            altitude=15.0,
        )
        print("\nSET_WP ack:", set_wp_ack)
        show_info("MSP_SET_WP", info)

        info, waypoint = api.get_waypoint(1)
        print("\nWaypoint:", waypoint)
        show_info("MSP_WP", info)

        info, nav_status = api.get_nav_status()
        print("\nNavigation status:", nav_status)
        show_info("MSP_NAV_STATUS", info)

        info, active_modes = api.get_active_modes()
        print("\nActive modes:", active_modes)
        show_info("get_active_modes", info)


        x = transport.health()
        print("Health:")
        print(x)

        x = transport.utilization()
        print("utilization:")
        print(x)

        x = transport.clients()
        print("clients:")
        print(x)

        x = transport.stats()
        print("stats:")
        print(x)

        print("\nForced uncached MSP_API_VERSION (no cache):")
        _, raw_payload = transport.request(int(InavMSP.MSP_API_VERSION), cacheable=False)
        info_uncached = api.info_from_diag(transport.last_diag, InavMSP.MSP_API_VERSION)
        version_uncached = api._codec.unpack_reply(InavMSP.MSP_API_VERSION, raw_payload)
        print("  ", version_uncached)
        show_info("MSP_API_VERSION (no cache)", info_uncached)

        schedules = transport.sched_get()
        print("\nCurrent scheduler:", schedules)

        _ = transport.sched_set(InavMSP.MSP_API_VERSION, delay=5.0)
        print("Setting MSP_API_VERSION poll every 5s")

        schedules = transport.sched_get()
        print("Scheduler after removal:", schedules)

        _ = transport.sched_remove(InavMSP.MSP_API_VERSION)
        print("Removing MSP_API_VERSION schedule")

        schedules = transport.sched_get()
        print("Scheduler after removal:", schedules)

    finally:
        api.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
