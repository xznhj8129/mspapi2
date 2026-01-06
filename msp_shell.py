#!/usr/bin/env python3
"""
Minimal MSP API shell.

Examples:
  python msp_shell.py --tcp-endpoint 127.0.0.1:5760
  > get_raw_gps
  > set_flight_axis_angle_override pitch_deg=10
  > set_altitude_target altitude_datum=1 altitude_m=50
"""

import argparse
import ast
import enum
import inspect
import sys
import traceback
from typing import get_args, get_origin, get_type_hints

try:
    import readline  # type: ignore  # noqa: F401
except ImportError:
    pass

import enum

from mspapi2.msp_api import MSPApi


def _parse_arg_token(token: str):
    if "=" not in token:
        raise ValueError(f"Argument '{token}' is missing '='")
    key, raw = token.split("=", 1)
    if not key:
        raise ValueError(f"Argument '{token}' has empty key")
    try:
        value = ast.literal_eval(raw)
    except (ValueError, SyntaxError):
        value = raw
    return key, value


def _format_value(v):
    if isinstance(v, enum.Enum):
        return f"{v.__class__.__name__}.{v.name} ({int(v)})"
    if isinstance(v, dict):
        return v
    if isinstance(v, (list, tuple)):
        return [ _format_value(x) for x in v ]
    return v


def _render_mapping(title: str, data, indent: int = 0):
    pad = " " * indent
    if title:
        print(f"{pad}{title}:")
        pad += "    "
    for k, v in data.items():
        formatted = _format_value(v)
        if isinstance(formatted, dict):
            _render_mapping(str(k), formatted, indent + 4 if title else indent)
        else:
            print(f"{pad}{k}: {formatted}")


def _render_result(cmd: str, result):
    if isinstance(result, tuple) and len(result) == 2:
        info, payload = result
        if isinstance(info, dict):
            err = info.get("status")
            if err not in (None, 0):
                print(f"{cmd}: ERROR status={err} info={info}")
                return
        if isinstance(payload, dict):
            _render_mapping(f"{cmd}", payload)
            return
    if isinstance(result, dict):
        _render_mapping(f"{cmd}", result)
        return
    print(result)


def _list_commands(api):
    names = []
    for name in dir(api):
        if name.startswith("_"):
            continue
        if callable(getattr(api, name)):
            names.append(name)
    return sorted(names)


def _signature_help(func):
    sig = inspect.signature(func)
    params = []
    for name, param in sig.parameters.items():
        if param.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
            continue
        params.append(name)
    return params


def main():
    ap = argparse.ArgumentParser(description="Interactive shell for MSPApi")
    ap.add_argument("--tcp-endpoint", help="MSP TCP endpoint in HOST:PORT format")
    ap.add_argument("--port", help="Serial MSP port")
    ap.add_argument("--baudrate", type=int, default=115200, help="Baudrate for serial MSP")
    args = ap.parse_args()

    init_kwargs = {}
    if args.tcp_endpoint:
        init_kwargs["tcp_endpoint"] = args.tcp_endpoint
        init_kwargs["port"] = None
    else:
        if not args.port:
            ap.error("Provide --tcp-endpoint or --port")
        init_kwargs["port"] = args.port
        init_kwargs["baudrate"] = args.baudrate

    api = MSPApi(**init_kwargs)
    api.open()

    print("MSP shell. Type 'help' for commands, 'quit' to exit.")
    commands = _list_commands(api)

    while True:
        try:
            line = input("> ")
        except KeyboardInterrupt:
            print()
            break
        except EOFError:
            print()
            break
        if not line:
            continue
        text = line.strip()
        if not text:
            continue
        if text in ("quit", "exit"):
            break
        if text == "help":
            print("Commands:")
            for name in commands:
                print(f"  {name}")
            continue

        parts = text.split()
        cmd = parts[0]
        if cmd not in commands:
            print(f"Unknown command '{cmd}'. Type 'help' for list.")
            continue
        func = getattr(api, cmd)
        if len(parts) == 2 and parts[1] == "help":
            params = _signature_help(func)
            hints = {}
            try:
                hints = get_type_hints(func)
            except Exception:
                pass
            if params:
                print("Parameters:")
                for p in params:
                    ann = hints.get(p, inspect.signature(func).parameters[p].annotation)
                    enum_type = None
                    origin = get_origin(ann)
                    if origin is None and isinstance(ann, type) and issubclass(ann, enum.IntEnum):
                        enum_type = ann
                    if origin is not None:
                        for arg in get_args(ann):
                            if isinstance(arg, type) and issubclass(arg, enum.IntEnum):
                                enum_type = arg
                                break
                    if enum_type:
                        members = ", ".join(f"{m.name}={m.value}" for m in enum_type)
                        print(f"  {p}: {members}")
                    else:
                        print(f"  {p}")
            else:
                print("No parameters.")
            continue

        kwargs = {}
        positional = []
        try:
            for token in parts[1:]:
                if "=" in token:
                    key, value = _parse_arg_token(token)
                    if key in kwargs:
                        raise ValueError(f"Argument '{key}' provided multiple times")
                    kwargs[key] = value
                else:
                    positional.append(token)
        except ValueError as exc:
            print(f"{exc}")
            continue

        params = _signature_help(func)

        if cmd in ("set_flight_axis_angle_override", "set_flight_axis_rate_override"):
            if positional:
                raise ValueError(
                    "Use named axes for overrides (angle: roll_deg/pitch_deg/yaw_deg, rate: roll_dps/pitch_dps/yaw_dps). "
                    "Positional args would implicitly enable all axes."
                )
            if not kwargs:
                print("No axes provided -> sending empty mask to disable all overrides.")

        if positional:
            if len(positional) > len(params):
                raise ValueError(f"{cmd} expects at most {len(params)} arguments")
            for idx, raw in enumerate(positional):
                key = params[idx]
                if key in kwargs:
                    raise ValueError(f"Argument '{key}' provided both positionally and by name")
                try:
                    value = ast.literal_eval(raw)
                except (ValueError, SyntaxError):
                    value = raw
                kwargs[key] = value

        try:
            result = func(**kwargs)
            _render_result(cmd, result)
        except KeyboardInterrupt:
            print("\nInterrupted.")
            break
        except Exception:
            traceback.print_exc()

    api.close()
    sys.exit(0)


if __name__ == "__main__":
    main()
