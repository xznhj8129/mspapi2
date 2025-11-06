from __future__ import annotations

import argparse

from lib import InavEnums
from msp_api import MSPApi
import time

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSP API demo")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path (ignored if --tcp is used)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--read-timeout", type=float, default=0.05, help="Read timeout in seconds")
    parser.add_argument("--write-timeout", type=float, default=0.25, help="Write timeout in seconds")
    return parser.parse_args()


tcpport = 5760
