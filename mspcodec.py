# msp_codec.py
# Minimal payload codec for MSP using InavMSP enum + msp_messages.json schema.
# Focus: pack/unpack payloads ONLY (binary <-> Python values), no transport.

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Iterable, Mapping, Any, Union
import enum
import json
import struct
from pathlib import Path

bin_type_map = {
    "enum": "B",
    "uint8_t": "B",
    "uint16_t": "H",
    "uint32_t": "I",
    "uint64_t": "Q",
    "int8_t": "b",
    "int16_t": "h",
    "int32_t": "i",
    "int64_t": "q",
    "float": "f",
    "double": "d",
    "char": "c",
    "bool": "?",
    "boolean": "?",  # normalize to bool
    "boxBitmask_t": "Q",
}

"""TODO: rewrite codec api, no "messagespec" helpers, load messages as classes using enums class per message, ex: msp.MSP_WP(waypointIndex=1)
example:
encoding:
MSPCodec.MSP_messagename is implicit encode
msg = mspMessage.MSP_SET_WP(
   waypointIndex=1, 
   action=InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT, 
   latitude=int(1.234e7), 
   longitude=int(2.345e7),
   altitude=1500, 
   param1=0, 
   param2=0, 
   param3=0, 
   flag=0 or InavEnums.navWaypointFlags_e.something
msg.code is InavMSP.MSP_SET_WP enum
msg.bytes is packed bytes

decoding:
(send MSP_WP request, receive response)
msg = MSPCodec.decode(received) 
msg is mspMessage class
msg.code is InavMSP.MSP_WP enum
msg.action is InavEnums.navWaypointActions_e.NAV_WP_ACTION_WAYPOINT
msg.waypointIndex is 1
msg.latitude is 12340000
msg.param3 is 0 or InavEnums.navWaypointP3Flags_e.something
etc

"""

def _load_multiwii_enum(schema_path: Path) -> type[enum.IntEnum]:
    try:
        with schema_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError as exc:
        raise RuntimeError(f"InavMSP schema not found: {schema_path}") from exc

    members: Dict[str, int] = {}
    for name, node in data.items():
        if not isinstance(name, str) or not name.isidentifier():
            continue
        code_raw = node.get("code") if isinstance(node, Mapping) else None
        try:
            code = int(code_raw)
        except (TypeError, ValueError):
            continue
        if name in members:
            continue
        members[name] = code

    if not members:
        raise RuntimeError(f"No MSP messages found in {schema_path}")

    return enum.IntEnum("InavMSP", members)


InavMSP = _load_multiwii_enum(Path(__file__).with_name("lib") / "msp_messages.json")

@dataclass(frozen=True)
class _PayloadSide:
    struct_fmt: Optional[str]          # e.g. "<hhhhhhhhh" or None
    field_names: List[str]             # ordered to match struct fields

    @property
    def size(self) -> Optional[int]:
        return None if self.struct_fmt is None else struct.calcsize(self.struct_fmt)


@dataclass(frozen=True)
class MessageSpec:
    code: InavMSP
    name: str
    mspv: Optional[int]
    direction: Optional[int]
    request: _PayloadSide
    reply: _PayloadSide


class MSPCodec:
    """
    Tiny codec that uses a JSON schema to (un)pack MSP payloads.

    Public API:
      - pack_request(code: InavMSP, values) -> bytes
      - unpack_reply(code: InavMSP, payload: bytes) -> dict

    'values' can be:
      - sequence matching schema order, OR
      - mapping keyed by field names.
    """
    def __init__(self, specs_by_code: Dict[InavMSP, MessageSpec]) -> None:
        self._specs = specs_by_code

    # ----- Loaders -----
    @classmethod
    def from_json_file(cls, path: str) -> "MSPCodec":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(cls._parse_schema(data))

    @classmethod
    def from_json_obj(cls, obj: Mapping[str, Any]) -> "MSPCodec":
        return cls(cls._parse_schema(obj))

    @staticmethod
    def _normalize_fmt(fmt: Optional[str]) -> Optional[str]:
        if fmt is None:
            return None
        return fmt if fmt and fmt[0] in "<>@=!" else "<" + fmt  # default little-endian

    @staticmethod
    def _struct_code_for_field(field: Mapping[str, Any]) -> Optional[str]:
        ctype_val = field.get("ctype")
        if not isinstance(ctype_val, str):
            return None
        ctype_val = ctype_val.strip()
        if not ctype_val:
            return None

        direct = bin_type_map.get(ctype_val)
        if direct:
            return direct

        if "[" in ctype_val and ctype_val.endswith("]"):
            base, size_spec = ctype_val.split("[", 1)
            base = base.strip()
            size_spec = size_spec[:-1].strip()

            size: Optional[int] = None
            if size_spec and size_spec.isdigit():
                size = int(size_spec)
            else:
                array_size = field.get("array_size")
                if isinstance(array_size, int):
                    size = array_size
                elif isinstance(array_size, str) and array_size.isdigit():
                    size = int(array_size)

            if not size or size <= 0:
                return None

            if base == "char":
                return f"{size}s"

            base_code = bin_type_map.get(base)
            if base_code:
                return f"{size}{base_code}"
            return None

        return None

    @classmethod
    def _payload_side_from_dict(cls, side: Optional[Mapping[str, Any]]) -> _PayloadSide:
        if not side:
            return _PayloadSide(struct_fmt=None, field_names=[])
        fields_desc = side.get("payload") or []
        names = [fd.get("name", f"f{i}") for i, fd in enumerate(fields_desc)]
        fmt_codes: List[str] = []
        fmt_valid = True
        for field in fields_desc:
            code = cls._struct_code_for_field(field)
            if code is None:
                fmt_valid = False
                break
            fmt_codes.append(code)

        if fmt_valid and fmt_codes:
            fmt = "<" + "".join(fmt_codes)
        elif fmt_valid:
            fmt = None
        else:
            fmt = cls._normalize_fmt(side.get("struct"))

        return _PayloadSide(struct_fmt=fmt, field_names=names)

    @classmethod
    def _parse_one(cls, name: str, node: Mapping[str, Any]) -> Optional[MessageSpec]:
        code = InavMSP(int(node["code"]))

        req = cls._payload_side_from_dict(node.get("request"))
        rep = cls._payload_side_from_dict(node.get("reply"))

        return MessageSpec(
            code=code,
            name=name,
            mspv=node.get("mspv"),
            direction=node.get("direction"),
            request=req,
            reply=rep,
        )

    @classmethod
    def _parse_schema(cls, obj: Mapping[str, Any]) -> Dict[InavMSP, MessageSpec]:
        specs: Dict[InavMSP, MessageSpec] = {}
        for name, node in obj.items():
            if not isinstance(node, Mapping):
                continue
            spec = cls._parse_one(name, node)
            if spec:
                specs[spec.code] = spec
        return specs

    # ----- Helpers -----

    def _get_spec(self, code: InavMSP) -> MessageSpec:
        try:
            return self._specs[code]
        except KeyError:
            raise KeyError(f"No schema for {getattr(code, 'name', code)} ({int(code)})")

    @staticmethod
    def _values_in_order(values: Union[Iterable[Any], Mapping[str, Any]], names: List[str]) -> List[Any]:
        if isinstance(values, Mapping):
            return [values[n] for n in names]
        vals = list(values)
        if len(vals) != len(names):
            raise ValueError(f"Expected {len(names)} values, got {len(vals)}")
        return vals

    # ----- Public API -----

    def pack_request(self, code: InavMSP, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
        spec = self._get_spec(code)
        fmt = spec.request.struct_fmt
        if fmt is None:
            if values not in ((), [], {}):
                raise ValueError(f"{spec.name}: request has no payload")
            return b""
        ordered = self._values_in_order(values, spec.request.field_names)
        try:
            return struct.pack(fmt, *ordered)
        except struct.error as e:
            raise ValueError(f"{spec.name}: pack_request error: {e}")

    def unpack_reply(self, code: InavMSP, payload: bytes) -> Dict[str, Any]:
        spec = self._get_spec(code)
        fmt = spec.reply.struct_fmt
        #if fmt is None:
            #if payload not in (b"",):
            #    raise ValueError(f"{spec.name}: reply expected empty payload, got {len(payload)} bytes")
            #return {}
        #size = struct.calcsize(fmt)
        #if len(payload) != size:
        #    raise ValueError(f"{spec.name}: reply size {len(payload)} != expected {size}")
        if len(payload)>0:
            try:
                values = struct.unpack(fmt, payload)
            except struct.error as e:
                raise ValueError(f"{spec.name}: unpack_reply error: {e}")
            return dict(zip(spec.reply.field_names, values))
        else:
            return {}

    # why
    def unpack_request(self, code: InavMSP, payload: bytes) -> Dict[str, Any]:
        spec = self._get_spec(code)
        fmt = spec.request.struct_fmt
        if fmt is None:
            if payload not in (b"",):
                raise ValueError(f"{spec.name}: request expected empty payload, got {len(payload)} bytes")
            return {}
        size = struct.calcsize(fmt)
        if len(payload) != size:
            raise ValueError(f"{spec.name}: request size {len(payload)} != expected {size}")
        return dict(zip(spec.request.field_names, struct.unpack(fmt, payload)))

    def pack_reply(self, code: InavMSP, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
        spec = self._get_spec(code)
        fmt = spec.reply.struct_fmt
        if fmt is None:
            if values not in ((), [], {}):
                raise ValueError(f"{spec.name}: reply has no payload")
            return b""
        ordered = self._values_in_order(values, spec.reply.field_names)
        try:
            return struct.pack(fmt, *ordered)
        except struct.error as e:
            raise ValueError(f"{spec.name}: pack_reply error: {e}")
