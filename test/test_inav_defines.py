from __future__ import annotations

import unittest

from mspapi2.lib import InavDefines


class InavDefinesTest(unittest.TestCase):
    def test_required_msp_defines_match_inav(self) -> None:
        self.assertEqual(
            {
                "CHANNEL_RANGE_MIN": InavDefines.CHANNEL_RANGE_MIN,
                "CHANNEL_RANGE_STEP_WIDTH": InavDefines.CHANNEL_RANGE_STEP_WIDTH,
                "SETTING_TYPE_OFFSET": InavDefines.SETTING_TYPE_OFFSET,
                "SETTING_SECTION_OFFSET": InavDefines.SETTING_SECTION_OFFSET,
                "SETTING_MODE_OFFSET": InavDefines.SETTING_MODE_OFFSET,
            },
            {
                "CHANNEL_RANGE_MIN": 900,
                "CHANNEL_RANGE_STEP_WIDTH": 25,
                "SETTING_TYPE_OFFSET": 0,
                "SETTING_SECTION_OFFSET": 3,
                "SETTING_MODE_OFFSET": 6,
            },
        )


if __name__ == "__main__":
    unittest.main()
