#!/usr/bin/env python3
"""
Usage:
  python3 test/test_nav_roi_msp.py --tcp 127.0.0.1:5760
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from mspapi2.lib import InavMSP
from mspapi2.msp_api import MSPApi

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
DEFAULT_TCP_ENDPOINT = "127.0.0.1:5760"
DEFAULT_UDP_ENDPOINT = None
DEFAULT_READ_TIMEOUT_MS = 10.0
DEFAULT_WRITE_TIMEOUT_MS = 250.0

ROI_KEYS = ("latitude", "longitude", "altitude", "param1", "param2", "alt_datum", "action", "flag")
ROI_REPLY_FIELDS = ["lat", "lon", "alt", "p1", "p2", "alt_datum", "action", "flag"]
ROI_REQUEST_FIELDS = ["lat", "lon", "alt", "p1", "p2", "alt_datum", "action", "flag"]
ROI_FLOAT_TOLERANCE = {
    "latitude": 1e-7,
    "longitude": 1e-7,
    "altitude": 1e-2,
}

ROI_TEST_SET_INPUT = {
    "latitude": 47.3977420,
    "longitude": 8.5455940,
    "altitude": 123.45,
    "p1": 321,
    "p2": -654,
    "alt_datum": InavEnums.geoAltitudeDatumFlag_e.NAV_WP_TAKEOFF_DATUM,
    "action": 1,
    "flag": 1,
}

ROI_TEST_SET_EXPECT = {
    "latitude": ROI_TEST_SET_INPUT["latitude"],
    "longitude": ROI_TEST_SET_INPUT["longitude"],
    "altitude": ROI_TEST_SET_INPUT["altitude"],
    "param1": ROI_TEST_SET_INPUT["p1"],
    "param2": ROI_TEST_SET_INPUT["p2"],
    "alt_datum": InavEnums.geoAltitudeDatumFlag_e.NAV_WP_TAKEOFF_DATUM,
    "action": 1,
    "flag": 1,
}

ROI_TEST_CLEAR_INPUT = {
    "latitude": ROI_TEST_SET_INPUT["latitude"],
    "longitude": ROI_TEST_SET_INPUT["longitude"],
    "altitude": ROI_TEST_SET_INPUT["altitude"],
    "p1": ROI_TEST_SET_INPUT["p1"],
    "p2": ROI_TEST_SET_INPUT["p2"],
    "alt_datum": ROI_TEST_SET_INPUT["alt_datum"],
    "action": ROI_TEST_SET_INPUT["action"],
    "flag": 0,
}

ROI_EXPECT_CLEARED = {
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "param1": 0,
    "param2": 0,
    "alt_datum": 0,
    "action": 0,
    "flag": 0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify MSP2_INAV_NAV_ROI and MSP2_INAV_SET_NAV_ROI against SITL.")
    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="Serial device path (ignored if --tcp or --udp is used)")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="Serial baud rate")
    parser.add_argument("--tcp", default=DEFAULT_TCP_ENDPOINT, metavar="HOST:PORT", help="Connect using TCP endpoint")
    parser.add_argument("--udp", default=DEFAULT_UDP_ENDPOINT, metavar="HOST:PORT", help="Connect using UDP endpoint")
    parser.add_argument("--force-mspv2", action="store_true", help="Force MSP v2 framing")
    parser.add_argument("--read-timeout-ms", type=float, default=DEFAULT_READ_TIMEOUT_MS, help="Read timeout in milliseconds")
    parser.add_argument("--write-timeout-ms", type=float, default=DEFAULT_WRITE_TIMEOUT_MS, help="Write timeout in milliseconds")
    return parser.parse_args()


def assert_roi_payload(label: str, payload: dict, expected: dict) -> None:
    for key in ROI_KEYS:
        actual_value = payload[key]
        expected_value = expected[key]
        if key in ROI_FLOAT_TOLERANCE:
            tolerance = ROI_FLOAT_TOLERANCE[key]
            assert abs(actual_value - expected_value) <= tolerance, (
                f"{label} mismatch for {key}: expected={expected_value} actual={actual_value} tol={tolerance}"
            )
        else:
            assert actual_value == expected_value, (
                f"{label} mismatch for {key}: expected={expected_value} actual={actual_value}"
            )


def main() -> None:
    args = parse_args()
    assert not (args.tcp and args.udp), "Provide only one of --tcp or --udp"
    assert hasattr(InavMSP, "MSP2_INAV_NAV_ROI"), "MSP2_INAV_NAV_ROI is missing from schema enum"
    assert hasattr(InavMSP, "MSP2_INAV_SET_NAV_ROI"), "MSP2_INAV_SET_NAV_ROI is missing from schema enum"

    port = None if (args.tcp or args.udp) else args.port
    api_kwargs = {
        "port": port,
        "baudrate": args.baudrate,
        "read_timeout_ms": args.read_timeout_ms,
        "write_timeout_ms": args.write_timeout_ms,
        "tcp_endpoint": args.tcp,
        "udp_endpoint": args.udp,
        "force_msp_v2": args.force_mspv2,
    }

    with MSPApi(**api_kwargs) as api:
        get_spec = api._codec._get_spec(InavMSP.MSP2_INAV_NAV_ROI)
        set_spec = api._codec._get_spec(InavMSP.MSP2_INAV_SET_NAV_ROI)
        assert get_spec.reply.field_names == ROI_REPLY_FIELDS, (
            f"Unexpected NAV_ROI reply fields: {get_spec.reply.field_names}"
        )
        assert set_spec.request.field_names == ROI_REQUEST_FIELDS, (
            f"Unexpected SET_NAV_ROI request fields: {set_spec.request.field_names}"
        )
        roi_before = api.get_nav_roi()
        print(f"roi_before={roi_before}")
        
        set_ack = api.set_nav_roi(**ROI_TEST_SET_INPUT)
        print(f"set_ack={set_ack}")

        roi_after_set = api.get_nav_roi()
        print(f"roi_after_set={roi_after_set}")
        assert_roi_payload("set", roi_after_set, ROI_TEST_SET_EXPECT)

        clear_ack = api.set_nav_roi(**ROI_TEST_CLEAR_INPUT)
        print(f"clear_ack={clear_ack}")

        roi_after_clear = api.get_nav_roi()
        print(f"roi_after_clear={roi_after_clear}")
        assert_roi_payload("clear", roi_after_clear, ROI_EXPECT_CLEARED)

    print("ROI MSP verification passed.")


if __name__ == "__main__":
    main()
