#!/usr/bin/env python3
"""
Usage:
    python msp_shell.py --tcp 127.0.0.1:5760
"""

import sys

from mspapi2.msp_shell import main


if __name__ == "__main__":
    sys.exit(main())
