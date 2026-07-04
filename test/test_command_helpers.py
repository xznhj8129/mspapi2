from __future__ import annotations

import struct
import unittest

from mspapi2.lib import InavMSP
from mspapi2.msp_api import MSPApi


class FakeSerial:
    def __init__(self) -> None:
        self.requests = []
        self.last_diag = None

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def request(self, code: int, payload: bytes, *, timeout: float):
        self.requests.append((code, payload, timeout))
        reply = struct.pack("<Q", 123456789000) if code == InavMSP.MSP2_INAV_TIMESYNC else b""
        return code, reply


class CommandHelperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.serial = FakeSerial()
        self.api = MSPApi(serial_transport=self.serial)

    def test_set_armed_packs_boolean_request(self) -> None:
        self.assertEqual(self.api.set_armed(True), {})
        self.assertEqual(
            self.serial.requests[-1],
            (InavMSP.MSP2_INAV_ARM_DISARM, b"\x01", 1.0),
        )

    def test_activate_rth_has_empty_request(self) -> None:
        self.assertEqual(self.api.activate_rth(), {})
        self.assertEqual(
            self.serial.requests[-1],
            (InavMSP.MSP2_INAV_ACTIVATE_RTH, b"", 1.0),
        )

    def test_activate_landing_has_empty_request(self) -> None:
        self.assertEqual(self.api.activate_landing(), {})
        self.assertEqual(
            self.serial.requests[-1],
            (InavMSP.MSP2_INAV_ACTIVATE_LANDING, b"", 1.0),
        )

    def test_get_timesync_ns_decodes_uint64(self) -> None:
        self.assertEqual(self.api.get_timesync_ns(), 123456789000)
        self.assertEqual(
            self.serial.requests[-1],
            (InavMSP.MSP2_INAV_TIMESYNC, b"", 1.0),
        )


if __name__ == "__main__":
    unittest.main()
