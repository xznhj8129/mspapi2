from __future__ import annotations

import struct
import unittest
from pathlib import Path

from mspapi2.lib import InavMSP
from mspapi2.mspcodec import MSPCodec


SCHEMA_PATH = Path(__file__).parents[1] / "mspapi2" / "lib" / "msp_messages.json"


class CodecTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.codec = MSPCodec.from_json_file(str(SCHEMA_PATH))

    def test_empty_variable_character_array_decodes(self) -> None:
        self.assertEqual(self.codec.unpack_reply(InavMSP.MSP_NAME, b""), {"craftName": b""})

    def test_variable_request_round_trip(self) -> None:
        values = {
            "definitionByte": 0x63,
            "channelData": [0x40, 0x06, 0x78, 0x05],
        }
        payload = self.codec.pack_request(InavMSP.MSP2_INAV_SET_AUX_RC, values)
        self.assertEqual(self.codec.unpack_request(InavMSP.MSP2_INAV_SET_AUX_RC, payload), values)

    def test_optional_trailing_request_field_can_be_omitted(self) -> None:
        values = {
            "latitude": 10000000,
            "longitude": 20000000,
            "altitudeTarget": 300,
            "altitudeDatum": 0,
        }
        payload = self.codec.pack_request(InavMSP.MSP2_INAV_SET_GLOBAL_TARGET, values)
        self.assertEqual(len(payload), 13)
        self.assertEqual(self.codec.unpack_request(InavMSP.MSP2_INAV_SET_GLOBAL_TARGET, payload), values)

    def test_repeating_reply_round_trip(self) -> None:
        values = [
            {
                "modePermanentId": 0,
                "auxChannelIndex": 0,
                "rangeStartStep": 16,
                "rangeEndStep": 48,
            },
            {
                "modePermanentId": 1,
                "auxChannelIndex": 1,
                "rangeStartStep": 16,
                "rangeEndStep": 32,
            },
        ]
        payload = self.codec.pack_reply(InavMSP.MSP_MODE_RANGES, values)
        self.assertEqual(self.codec.unpack_reply(InavMSP.MSP_MODE_RANGES, payload), values)

    def test_opaque_led_config_round_trip(self) -> None:
        values = {
            "ledIndex": 2,
            "ledConfig": b"\x01\x02\x03\x04\x05\x06",
        }
        payload = self.codec.pack_request(InavMSP.MSP2_INAV_SET_LED_STRIP_CONFIG_EX, values)
        self.assertEqual(self.codec.unpack_request(InavMSP.MSP2_INAV_SET_LED_STRIP_CONFIG_EX, payload), values)

    def test_esc_telemetry_struct_array_decodes(self) -> None:
        payload = b"\x01" + struct.pack("<BxhhxxiI", 2, 45, 1200, 340, 5000)
        self.assertEqual(
            self.codec.unpack_reply(InavMSP.MSP2_INAV_ESC_TELEM, payload),
            {
                "motorCount": 1,
                "escData": [2, 45, 1200, 340, 5000],
            },
        )


if __name__ == "__main__":
    unittest.main()
