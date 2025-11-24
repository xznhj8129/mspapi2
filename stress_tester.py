#!/usr/bin/env python3
"""MSP server stress tester: hammers MSPApi and scheduler paths."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import random
import time
from typing import Any, Callable, Dict, List, Tuple

from mspapi2.lib import InavMSP
from mspapi2.msp_api import MSPApi, MSPServerTransport

API_CALLS: List[Tuple[str, Callable[[MSPApi], Tuple[Dict[str, Any], Any]]]] = []


def _register_call(name: str):
    def decorator(func: Callable[[MSPApi], Tuple[Dict[str, Any], Any]]):
        API_CALLS.append((name, func))
        return func

    return decorator


@_register_call("MSP_API_VERSION")
def _(api: MSPApi):
    return api.get_api_version()


@_register_call("MSP_FC_VARIANT")
def _(api: MSPApi):
    return api.get_fc_variant()


@_register_call("MSP_BOARD_INFO")
def _(api: MSPApi):
    return api.get_board_info()


@_register_call("MSP_SENSOR_CONFIG")
def _(api: MSPApi):
    return api.get_sensor_config()


@_register_call("MSP_MODE_RANGES")
def _(api: MSPApi):
    return api.get_mode_ranges()


@_register_call("MSP2_INAV_STATUS")
def _(api: MSPApi):
    return api.get_inav_status()


@_register_call("MSP2_INAV_ANALOG")
def _(api: MSPApi):
    return api.get_inav_analog()


@_register_call("MSP_RX_CONFIG")
def _(api: MSPApi):
    return api.get_rx_config()


@_register_call("MSP_ATTITUDE")
def _(api: MSPApi):
    return api.get_attitude()


@_register_call("MSP_ALTITUDE")
def _(api: MSPApi):
    return api.get_altitude()


@_register_call("MSP_RAW_IMU")
def _(api: MSPApi):
    return api.get_imu()


@_register_call("MSP_RC")
def _(api: MSPApi):
    return api.get_rc_channels()


@_register_call("MSP_SET_RAW_RC")
def _(api: MSPApi):
    channels = [random.randint(900, 2100) for _ in range(8)]
    return api.set_rc_channels(channels)


@_register_call("MSP_RAW_GPS")
def _(api: MSPApi):
    return api.get_raw_gps()


async def api_worker(idx: int, host: str, port: int, duration: float) -> None:
    ident = f"stress-api-{idx}"
    transport = MSPServerTransport(host, port, client_id=ident)
    api = MSPApi(port=None, serial_transport=transport)
    api.open()
    end = time.time() + duration
    try:
        while time.time() < end:
            name, func = random.choice(API_CALLS)
            try:
                info, _ = func(api)
                logging.info("API[%s] %-20s latency=%s cached=%s", ident, name, info.get("latency_ms"), info.get("cached"))
            except Exception:
                logging.exception("API[%s] call %s failed", ident, name)
            await asyncio.sleep(0.001)
    finally:
        api.close()


async def scheduler_worker(idx: int, host: str, port: int, duration: float) -> None:
    ident = f"stress-sched-{idx}"
    transport = MSPServerTransport(host, port, client_id=ident)
    api = MSPApi(port=None, serial_transport=transport)
    api.open()
    codes = list(InavMSP)
    end = time.time() + duration
    try:
        while time.time() < end:
            code = random.choice(codes)
            delay = random.uniform(0.5, 10.0)
            try:
                info, _ = api.sched_set(code, delay=delay)
                logging.info("SCHED[%s] set %s delay=%.2fs", ident, code.name, delay)
                describe_sched_info(info)
                await asyncio.sleep(random.uniform(0.5, 2.0))
                info, schedules = api.sched_get()
                logging.debug("SCHED[%s] snapshot: %s", ident, schedules)
                describe_sched_info(info)
                if random.random() < 0.3:
                    info, _ = api.sched_remove(code)
                    logging.info("SCHED[%s] removed %s", ident, code.name)
                    describe_sched_info(info)
            except Exception:
                logging.exception("SCHED[%s] error", ident)
            await asyncio.sleep(random.uniform(0.2, 1.0))
    finally:
        api.close()


def describe_sched_info(info: Dict[str, Any]) -> None:
    if not info:
        return
    logging.debug("diag=%s", info)


async def run(args: argparse.Namespace) -> None:
    duration = args.duration
    tasks = []
    for i in range(args.api_workers):
        tasks.append(asyncio.create_task(api_worker(i, args.host, args.port, duration)))
    #for i in range(args.scheduler_workers):
    #    tasks.append(asyncio.create_task(scheduler_worker(i, args.host, args.port, duration)))
    await asyncio.gather(*tasks, return_exceptions=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="MSP server stress tester")
    parser.add_argument("--host", default="127.0.0.1", help="Server host")
    parser.add_argument("--port", type=int, default=9000, help="Server port")
    parser.add_argument("--duration", type=float, default=30.0, help="Test duration in seconds")
    parser.add_argument("--api-workers", type=int, default=4, help="Number of MSPApi workers")
    parser.add_argument("--scheduler-workers", type=int, default=2, help="Number of scheduler workers")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level), format="[%(asctime)s] %(levelname)s: %(message)s")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
