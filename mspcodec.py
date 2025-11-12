# msp_codec.py
# Minimal payload codec for MSP using InavMSP enum + msp_messages.json schema.
# Focus: pack/unpack payloads ONLY (binary <-> Python values), no transport.

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, Iterable, Mapping, Any, Union, Tuple, Sequence
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


class MSPResult(enum.IntEnum):
    MSP_RESULT_ACK = 1
    MSP_RESULT_ERROR = -1
    MSP_RESULT_NO_REPLY = 0


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
    fields: Tuple[Mapping[str, Any], ...]  # raw field descriptors from schema
    repeating: Optional[str]           # identifier of repeating group (if any)

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

        inline_array = False
        inline_length: Optional[int] = None
        if "[" in ctype_val and ctype_val.endswith("]"):
            inline_array = True
            base, size_spec = ctype_val.split("[", 1)
            ctype_val = base.strip()
            size_spec = size_spec[:-1].strip()
            if size_spec.isdigit():
                parsed = int(size_spec)
                if parsed > 0:
                    inline_length = parsed

        def _positive(value: Any) -> Optional[int]:
            if isinstance(value, int):
                return value if value > 0 else None
            if isinstance(value, str) and value.isdigit():
                parsed_val = int(value)
                return parsed_val if parsed_val > 0 else None
            return None

        declared_length = _positive(field.get("array_size"))
        is_array = bool(field.get("array")) or inline_array or declared_length is not None
        array_length = inline_length if inline_length is not None else declared_length

        if is_array:
            base = field.get("array_ctype") or ctype_val
            base = base.replace("[]", "").strip()
            if base == "char":
                if array_length is None:
                    return None
                return f"{array_length}s"
            base_code = bin_type_map.get(base)
            if base_code is None or array_length is None:
                return None
            return f"{array_length}{base_code}"

        direct = bin_type_map.get(ctype_val)
        if direct:
            return direct

        return None

    @classmethod
    def _payload_side_from_dict(cls, side: Optional[Mapping[str, Any]]) -> _PayloadSide:
        if not side:
            return _PayloadSide(struct_fmt=None, field_names=[], fields=(), repeating=None)
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

        repeating = side.get("repeating")

        return _PayloadSide(struct_fmt=fmt, field_names=names, fields=tuple(fields_desc), repeating=repeating)

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

    @staticmethod
    def _field_is_array(field: Mapping[str, Any]) -> bool:
        if not field:
            return False
        if field.get("array"):
            return True
        ctype = str(field.get("ctype", ""))
        return "[]" in ctype or "[" in ctype

    @staticmethod
    def _field_array_length(field: Mapping[str, Any]) -> Optional[int]:
        size = field.get("array_size")
        if isinstance(size, int):
            return size
        if isinstance(size, str) and size.isdigit():
            return int(size)
        return None

    @staticmethod
    def _field_array_base(field: Mapping[str, Any]) -> str:
        base = field.get("array_ctype")
        if base:
            return str(base).replace("[]", "").strip()
        ctype = str(field.get("ctype", ""))
        if "[" in ctype:
            ctype = ctype.split("[", 1)[0]
        return ctype.replace("[]", "").strip()

    @classmethod
    def _map_struct_values(
        cls,
        message_name: str,
        side: _PayloadSide,
        values: Sequence[Any],
    ) -> Dict[str, Any]:
        if not side.field_names:
            return {}

        if not side.fields:
            return dict(zip(side.field_names, values))

        result: Dict[str, Any] = {}
        idx = 0
        total = len(values)

        for name, field in zip(side.field_names, side.fields):
            if not cls._field_is_array(field):
                if idx >= total:
                    raise ValueError(f"{message_name}: insufficient data for field '{name}'")
                result[name] = values[idx]
                idx += 1
                continue

            base = cls._field_array_base(field)
            arr_len = cls._field_array_length(field)

            if base == "char":
                if idx >= total:
                    raise ValueError(f"{message_name}: insufficient data for field '{name}'")
                result[name] = values[idx]
                idx += 1
                continue

            if arr_len is None or arr_len <= 0 or idx + arr_len > total:
                arr_len = total - idx

            if arr_len < 0 or idx + arr_len > total:
                raise ValueError(f"{message_name}: array field '{name}' exceeds payload size")

            chunk = list(values[idx : idx + arr_len])
            result[name] = chunk
            idx += arr_len

        if idx < total:
            last_field = side.fields[-1] if side.fields else None
            last_name = side.field_names[-1]
            if last_field and cls._field_is_array(last_field) and cls._field_array_base(last_field) != "char":
                extra = list(values[idx:])
                existing = result.get(last_name, [])
                if not isinstance(existing, list):
                    existing = list(existing)
                existing.extend(extra)
                result[last_name] = existing
                idx = total

        if idx != total:
            raise ValueError(f"{message_name}: unused payload data ({total - idx} values)")

        return result

    @classmethod
    def _flatten_struct_values(
        cls,
        message_name: str,
        side: _PayloadSide,
        values: Union[Iterable[Any], Mapping[str, Any]],
    ) -> List[Any]:
        if not side.field_names:
            return []

        if not side.fields:
            return cls._values_in_order(values, side.field_names)

        if isinstance(values, Mapping):
            data = values
        else:
            seq = list(values)
            if len(seq) != len(side.field_names):
                raise ValueError(f"{message_name}: expected {len(side.field_names)} values, got {len(seq)}")
            data = dict(zip(side.field_names, seq))

        flat: List[Any] = []
        for name, field in zip(side.field_names, side.fields):
            if name not in data:
                raise KeyError(f"{message_name}: missing value for field '{name}'")
            value = data[name]
            if not cls._field_is_array(field):
                flat.append(value)
                continue

            base = cls._field_array_base(field)
            arr_len = cls._field_array_length(field)

            if base == "char":
                if isinstance(value, (bytes, bytearray)):
                    flat.append(bytes(value))
                elif isinstance(value, str):
                    flat.append(value.encode("latin1"))
                else:
                    raise ValueError(f"{message_name}: expected bytes for char array field '{name}'")
                continue

            if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
                raise ValueError(f"{message_name}: expected sequence for array field '{name}'")

            items = list(value)
            if arr_len is not None and arr_len > 0 and len(items) != arr_len:
                raise ValueError(
                    f"{message_name}: array field '{name}' expects {arr_len} elements, got {len(items)}"
                )
            flat.extend(items)

        return flat

    # ----- Public API -----

    def pack_request(self, code: InavMSP, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
        spec = self._get_spec(code)
        fmt = spec.request.struct_fmt
        if fmt is None:
            if values not in ((), [], {}):
                raise ValueError(f"{spec.name}: request has no payload")
            return b""
        ordered = self._flatten_struct_values(spec.name, spec.request, values)
        try:
            return struct.pack(fmt, *ordered)
        except struct.error as e:
            raise ValueError(f"{spec.name}: pack_request error: {e}")

    def unpack_reply(self, code: InavMSP, payload: bytes) -> Any:
        spec = self._get_spec(code)
        side = spec.reply
        fmt = side.struct_fmt

        if side.repeating:
            return self._decode_repeating_payload(spec.name, side, payload)

        if fmt is None:
            return self._decode_variable_payload(spec.name, side, payload)

        expected_size = side.size
        if expected_size is not None and len(payload) != expected_size:
            raise ValueError(f"{spec.name}: reply size {len(payload)} != expected {expected_size}")

        try:
            values = struct.unpack(fmt, payload)
        except struct.error as e:
            raise ValueError(f"{spec.name}: unpack_reply error: {e}")
        return self._map_struct_values(spec.name, side, values)

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
        values = struct.unpack(fmt, payload)
        return self._map_struct_values(spec.name, spec.request, values)

    def pack_reply(self, code: InavMSP, values: Union[Iterable[Any], Mapping[str, Any]] = ()) -> bytes:
        spec = self._get_spec(code)
        fmt = spec.reply.struct_fmt
        if fmt is None:
            if values not in ((), [], {}):
                raise ValueError(f"{spec.name}: reply has no payload")
            return b""
        ordered = self._flatten_struct_values(spec.name, spec.reply, values)
        try:
            return struct.pack(fmt, *ordered)
        except struct.error as e:
            raise ValueError(f"{spec.name}: pack_reply error: {e}")

    @staticmethod
    def _decode_variable_payload(message_name: str, side: _PayloadSide, payload: bytes) -> Dict[str, Any]:
        fields = side.fields
        if not fields:
            if payload not in (b"",):
                raise ValueError(f"{message_name}: reply expected empty payload, got {len(payload)} bytes")
            return {}

        values, offset = MSPCodec._decode_field_sequence(message_name, fields, payload, 0)
        if offset != len(payload):
            raise ValueError(f"{message_name}: decoded {offset} of {len(payload)} payload bytes")
        return values

    @classmethod
    def _decode_repeating_payload(cls, message_name: str, side: _PayloadSide, payload: bytes) -> List[Any]:
        if not payload:
            return []

        if side.struct_fmt is not None:
            entry_size = struct.calcsize(side.struct_fmt)
            if entry_size <= 0:
                raise ValueError(f"{message_name}: invalid repeating entry size {entry_size}")
            if len(payload) % entry_size != 0:
                raise ValueError(
                    f"{message_name}: payload size {len(payload)} not aligned to repeating entry size {entry_size}"
                )
            entries: List[Dict[str, Any]] = []
            for offset in range(0, len(payload), entry_size):
                raw = struct.unpack_from(side.struct_fmt, payload, offset)
                entries.append(cls._map_struct_values(message_name, side, raw))
            return entries

        fields = side.fields
        if not fields:
            if payload not in (b"",):
                raise ValueError(f"{message_name}: repeating payload has no schema but is non-empty")
            return []

        entries: List[Dict[str, Any]] = []
        offset = 0
        while offset < len(payload):
            values, offset = MSPCodec._decode_field_sequence(message_name, fields, payload, offset)
            entries.append(values)
        if offset != len(payload):
            raise ValueError(f"{message_name}: decoded {offset} of {len(payload)} payload bytes")
        return entries

    @staticmethod
    def _decode_field_sequence(
        message_name: str,
        fields: Tuple[Mapping[str, Any], ...],
        payload: bytes,
        offset: int,
    ) -> Tuple[Dict[str, Any], int]:
        values: Dict[str, Any] = {}
        current_offset = offset
        for idx, field in enumerate(fields):
            name, value, current_offset = MSPCodec._decode_single_field(
                message_name, field, payload, current_offset, values, idx
            )
            values[name] = value
        return values, current_offset

    @staticmethod
    def _decode_single_field(
        message_name: str,
        field: Mapping[str, Any],
        payload: bytes,
        offset: int,
        parsed: Dict[str, Any],
        index: int,
    ) -> Tuple[str, Any, int]:
        name = field.get("name") or f"f{index}"
        total_len = len(payload)
        optional = bool(field.get("optional"))
        if offset > total_len:
            raise ValueError(f"{message_name}: field '{name}' offset {offset} exceeds payload size {total_len}")

        is_array = bool(field.get("array")) or "[]" in str(field.get("ctype", ""))

        if offset == total_len:
            if optional:
                return name, ([] if is_array else None), offset
            raise ValueError(f"{message_name}: field '{name}' offset {offset} exceeds payload size {total_len}")

        if is_array:
            base_ctype = field.get("array_ctype") or field.get("ctype", "")
            if not base_ctype:
                raise ValueError(f"{message_name}: array field '{name}' missing base type")
            base_ctype = base_ctype.replace("[]", "").strip()
            if base_ctype == "char":
                elem_size = 1
                base_code = None
            else:
                base_code = bin_type_map.get(base_ctype)
                if not base_code:
                    raise ValueError(f"{message_name}: unsupported array base type '{base_ctype}' for field '{name}'")
                elem_size = struct.calcsize("<" + base_code)
            count = field.get("array_size")
            if isinstance(count, str) and count.isdigit():
                count = int(count)
            if isinstance(count, int) and count > 0:
                length = count
            else:
                length = MSPCodec._resolve_dynamic_length(field, parsed)
                if length is None:
                    remaining = total_len - offset
                    if elem_size == 0:
                        raise ValueError(f"{message_name}: cannot determine element size for field '{name}'")
                    if remaining % elem_size != 0:
                        raise ValueError(
                            f"{message_name}: remaining bytes {remaining} not aligned to element size {elem_size} for field '{name}'"
                        )
                    length = remaining // elem_size
            total_bytes = elem_size * length
            if offset + total_bytes > total_len:
                if optional:
                    default_val: Any = b"" if base_ctype == "char" else []
                    return name, default_val, total_len
                raise ValueError(
                    f"{message_name}: field '{name}' length {total_bytes} exceeds payload boundary ({offset} + {total_bytes} > {total_len})"
                )
            chunk = payload[offset:offset + total_bytes]
            new_offset = offset + total_bytes
            if base_ctype == "char":
                value: Any = chunk
            elif total_bytes == 0:
                value = []
            else:
                fmt = "<" + base_code * length
                value = list(struct.unpack(fmt, chunk))
            return name, value, new_offset

        struct_code = MSPCodec._struct_code_for_field(field)
        if struct_code is None:
            raise ValueError(f"{message_name}: unsupported field '{name}' without struct code")
        fmt = MSPCodec._normalize_fmt(struct_code)
        if fmt is None:
            raise ValueError(f"{message_name}: unable to normalize format for field '{name}'")
        size = struct.calcsize(fmt)
        if offset + size > total_len:
            if optional:
                return name, None, total_len
            raise ValueError(
                f"{message_name}: field '{name}' size {size} exceeds payload boundary ({offset} + {size} > {total_len})"
            )
        raw = struct.unpack_from(fmt, payload, offset)
        value = raw[0] if len(raw) == 1 else raw
        new_offset = offset + size
        return name, value, new_offset

    @staticmethod
    def _resolve_dynamic_length(field: Mapping[str, Any], parsed: Mapping[str, Any]) -> Optional[int]:
        explicit = field.get("length_field") or field.get("lengthField") or field.get("lengthFrom")
        if explicit:
            length = parsed.get(explicit)
            if length is not None:
                try:
                    return int(length)
                except (TypeError, ValueError):
                    pass

        name = field.get("name") or ""
        candidates: List[str] = []
        if name:
            candidates.extend([
                f"{name}Length",
                f"{name}Count",
            ])
            if name.endswith("s"):
                singular = name[:-1]
                candidates.extend([
                    f"{singular}Length",
                    f"{singular}Count",
                ])
        for candidate in candidates:
            if candidate and candidate in parsed:
                try:
                    return int(parsed[candidate])
                except (TypeError, ValueError):
                    continue
        return None
