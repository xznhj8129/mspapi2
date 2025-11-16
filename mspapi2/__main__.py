"""Console entry point for running the MSP request server."""

from __future__ import annotations

from .msp_server import main as _server_main


def main() -> None:
    _server_main()


if __name__ == "__main__":
    main()
