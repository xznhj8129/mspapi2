# msp_codec.py
# Minimal payload codec for MSP using MultiWii enum + msp_messages.json schema.
# Focus: pack/unpack payloads ONLY (binary <-> Python values), no transport.

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Iterable, Mapping, Any, Union
import enum
import json
import struct
from pathlib import Path

def _load_multiwii_enum(schema_path: Path) -> type[enum.IntEnum]:
    try:
        with schema_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError as exc:
        raise RuntimeError(f"MultiWii schema not found: {schema_path}") from exc

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

    return enum.IntEnum("MultiWii", members)


MultiWii = _load_multiwii_enum(Path(__file__).with_name("lib") / "msp_messages.json")

@dataclass(frozen=True)
class _PayloadSide:
    struct_fmt: Optional[str]          # e.g. "<hhhhhhhhh" or None
    field_names: List[str]             # ordered to match struct fields

    @property
    def size(self) -> Optional[int]:
        return None if self.struct_fmt is None else struct.calcsize(self.struct_fmt)


@dataclass(frozen=True)
class MessageSpec:
    code: MultiWii
    name: str
    mspv: Optional[int]
    direction: Optional[int]
    request: _PayloadSide
    reply: _PayloadSide


class MSPCodec:
    """
    Tiny codec that uses a JSON schema to (un)pack MSP payloads.

    Public API:
      - pack_request(code: MultiWii, values) -> bytes
      - unpack_reply(code: MultiWii, payload: bytes) -> dict

    'values' can be:
      - sequence matching schema order, OR
      - mapping keyed by field names.
    """
    def __init__(self, specs_by_code: Dict[MultiWii, MessageSpec]) -> None:
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

    @classmethod
    def _payload_side_from_dict(cls, side: Optional[Mapping[str, Any]]) -> _PayloadSide:
        if not side:
            return _PayloadSide(struct_fmt=None, field_names=[])
        fmt = cls._normalize_fmt(side.get("struct"))
        fields_desc = side.get("payload") or []
        names = [fd.get("name", f"f{i}") for i, fd in enumerate(fields_desc)]
        return _PayloadSide(struct_fmt=fmt, field_names=names)

    @classmethod
    def _parse_one(cls, name: str, node: Mapping[str, Any]) -> Optional[MessageSpec]:
        code = MultiWii(int(node["code"]))

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
    def _parse_schema(cls, obj: Mapping[str, Any]) -> Dict[MultiWii, MessageSpec]:
        specs: Dict[MultiWii, MessageSpec] = {}
        for name, node in obj.items():
            if not isinstance(node, Mapping):
                continue
            spec = cls._parse_one(name, node)
            if spec:
                specs[spec.code] = spec
        return specs

    # ----- Helpers -----

    def _get_spec(self, code: MultiWii) -> MessageSpec:
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

    def pack_request(self, code: MultiWii, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
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

    def unpack_reply(self, code: MultiWii, payload: bytes) -> Dict[str, Any]:
        spec = self._get_spec(code)
        fmt = spec.reply.struct_fmt
        if fmt is None:
            return {}
        size = struct.calcsize(fmt)
        if len(payload) != size:
            raise ValueError(f"{spec.name}: reply size {len(payload)} != expected {size}")
        try:
            values = struct.unpack(fmt, payload)
        except struct.error as e:
            raise ValueError(f"{spec.name}: unpack_reply error: {e}")
        return dict(zip(spec.reply.field_names, values))

    # Optional extras if you ever need them:
    def unpack_request(self, code: MultiWii, payload: bytes) -> Dict[str, Any]:
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

    def pack_reply(self, code: MultiWii, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
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
