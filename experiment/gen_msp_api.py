#!/usr/bin/env python3
# gen_msp_messages.py
#
# Usage:
#   python gen_msp_messages.py <input_json_path> <output_py_path>

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# Map C types to Python struct format and range
CTYPE = {
    "uint8_t":  ("<B", 0, 2**8 - 1, 1),
    "int8_t":   ("<b", -(2**7), 2**7 - 1, 1),
    "uint16_t": ("<H", 0, 2**16 - 1, 2),
    "int16_t":  ("<h", -(2**15), 2**15 - 1, 2),
    "uint32_t": ("<I", 0, 2**32 - 1, 4),
    "int32_t":  ("<i", -(2**31), 2**31 - 1, 4),
    "uint64_t": ("<Q", 0, 2**64 - 1, 8),
    "int64_t":  ("<q", -(2**63), 2**63 - 1, 8),
    "float":    ("<f", None, None, 4),  # range check only for ints
    # "char" handled specially
}

def _assert_int_in_range(name: str, v: int, lo: Optional[int], hi: Optional[int]) -> None:
    if not isinstance(v, int):
        raise TypeError(f"{name} must be int, got {type(v).__name__}")
    if lo is not None and v < lo:
        raise ValueError(f"{name} {v} < {lo}")
    if hi is not None and v > hi:
        raise ValueError(f"{name} {v} > {hi}")

def _pack_scalar(ctype: str, name: str, v: Any) -> bytes:
    if ctype == "char":
        raise RuntimeError("internal misuse: scalar char must be handled by array logic")
    fmt, lo, hi, _ = CTYPE[ctype]
    if lo is not None:
        _assert_int_in_range(name, v, lo, hi)
    elif ctype == "float":
        if not isinstance(v, (int, float)):
            raise TypeError(f"{name} must be float compatible")
        v = float(v)
    return __import__("struct").pack(fmt, v)

def _unpack_scalar(ctype: str, name: str, data: bytes) -> Tuple[Any, bytes]:
    if ctype == "char":
        raise RuntimeError("internal misuse: scalar char must be handled by array logic")
    fmt, _, _, size = CTYPE[ctype]
    if len(data) < size:
        raise ValueError(f"not enough data to unpack {name} ({ctype}) need {size} have {len(data)}")
    v = __import__("struct").unpack(fmt, data[:size])[0]
    return v, data[size:]

def _sanitize_class(name: str) -> str:
    out = []
    for ch in name:
        if ch.isalnum():
            out.append(ch)
        else:
            out.append("_")
    s = "".join(out)
    if s and s[0].isdigit():
        s = "_" + s
    return "".join([part.capitalize() for part in s.split("_") if part])

def _py_type_for(field_def: Dict[str, Any]) -> str:
    ctype = field_def.get("ctype")
    is_array = bool(field_def.get("array"))
    if is_array:
        if ctype == "char":
            if field_def.get("array_size", 0) == 0:
                return "bytes"
            return "str"
        base = "int" if ctype != "float" else "float"
        return f"List[{base}]"
    if ctype == "char":
        return "str"
    if ctype == "float":
        return "float"
    return "int"

def _default_for(field_def: Dict[str, Any]) -> str:
    if field_def.get("optional"):
        return "None"
    t = _py_type_for(field_def)
    if t.startswith("List["):
        return "field(default_factory=list)"
    if t == "str":
        return "''"
    if t == "bytes":
        return "b''"
    if t == "float":
        return "0.0"
    return "0"

def _emit_dataclass(writer, cls_name: str, payload: Dict[str, Any], level_tag: str) -> None:
    fields_list: List[Dict[str, Any]] = payload.get("payload", []) if payload else []
    writer(f"    @dataclass\n")
    writer(f"    class {cls_name}:\n")
    try:

        for fdef in fields_list:
            name = fdef["name"]
            optional = fdef.get("optional", False)
            if "payload" in fdef and fdef.get("repeating"):
                nested_name = _sanitize_class(name)
                writer(f"        {name}: List['{nested_name}'] = field(default_factory=list)\n")
            else:
                py_t = _py_type_for(fdef)
                default = _default_for(fdef)
                optwrap = f"Optional[{py_t}]" if optional else py_t
                writer(f"        {name}: {optwrap} = {default}\n")

        for fdef in fields_list:
            if "payload" in fdef and fdef.get("repeating"):
                nested_name = _sanitize_class(fdef["name"])
                writer("\n")
                _emit_dataclass(writer, nested_name, fdef, level_tag + "_" + nested_name)

        writer("\n")
        writer(f"        def pack(self) -> bytes:\n")
        writer(f"            out = bytearray()\n")
        for fdef in fields_list:
            name = fdef["name"]
            ctype = fdef.get("ctype")
            is_optional = fdef.get("optional", False)
            is_array = fdef.get("array", False)
            if "payload" in fdef and fdef.get("repeating"):
                writer(f"            if not isinstance(self.{name}, list):\n")
                writer(f"                raise TypeError('{name} must be a list of {_sanitize_class(name)}')\n")
                writer(f"            for _item in self.{name}:\n")
                writer(f"                if not hasattr(_item, 'pack'):\n")
                writer(f"                    raise TypeError('{name} items must be {_sanitize_class(name)}')\n")
                writer(f"                out += _item.pack()\n")
                continue

            if is_optional:
                writer(f"            if self.{name} is None:\n")
                writer(f"                pass\n")
                writer(f"            else:\n")
                writer(f"                _v = self.{name}\n")
                indent = "                "
            else:
                writer(f"            _v = self.{name}\n")
                indent = "            "

            if is_array:
                if ctype == "char":
                    size = fdef.get("array_size", 0)
                    if size == 0:
                        writer(f"{indent}if not isinstance(_v, (bytes, bytearray)):\n")
                        writer(f"{indent}    raise TypeError('{name} must be bytes for variable length char array')\n")
                        writer(f"{indent}out += _v\n")
                    else:
                        writer(f"{indent}if not isinstance(_v, str):\n")
                        writer(f"{indent}    raise TypeError('{name} must be str')\n")
                        writer(f"{indent}_b = _v.encode('ascii', errors='ignore')\n")
                        writer(f"{indent}if len(_b) > {size}:\n")
                        writer(f"{indent}    raise ValueError('{name} length {{}} > {size}'.format(len(_b)))\n")
                        writer(f"{indent}out += _b.ljust({size}, b'\\x00')\n")
                else:
                    size = fdef.get("array_size", 0)
                    writer(f"{indent}if not isinstance(_v, list):\n")
                    writer(f"{indent}    raise TypeError('{name} must be list')\n")
                    if size and size > 0:
                        writer(f"{indent}if len(_v) != {size}:\n")
                        writer(f"{indent}    raise ValueError('{name} must have exactly {size} elements')\n")
                    writer(f"{indent}for i, _e in enumerate(_v):\n")
                    writer(f"{indent}    out += _pack_scalar('{ctype}', '{name}[{{}}]'.format(i), _e)\n")
            else:
                if ctype == "char":
                    writer(f"{indent}if not isinstance(_v, str) or len(_v) != 1:\n")
                    writer(f"{indent}    raise TypeError('{name} must be a single character')\n")
                    writer(f"{indent}out += _v.encode('ascii', errors='ignore')\n")
                else:
                    writer(f"{indent}out += _pack_scalar('{ctype}', '{name}', _v)\n")
        writer(f"            return bytes(out)\n")

        writer("\n")
        writer(f"        @classmethod\n")
        writer(f"        def unpack(cls, data: bytes, **hints) -> Tuple['{cls_name}', bytes]:\n")
        writer(f"            _data = memoryview(data).tobytes()\n")
        writer(f"            _kwargs: Dict[str, Any] = {{}}\n")
        for fdef in fields_list:
            name = fdef["name"]
            ctype = fdef.get("ctype")
            is_optional = fdef.get("optional", False)
            is_array = fdef.get("array", False)
            if "payload" in fdef and fdef.get("repeating"):
                nested = _sanitize_class(name)
                writer(f"            _count = hints.get('{name}_count', hints.get('repeating_count'))\n")
                writer(f"            if _count is None:\n")
                writer(f"                raise ValueError('missing hint: {name}_count or repeating_count for repeating block')\n")
                writer(f"            _lst = []\n")
                writer(f"            for _ in range(int(_count)):\n")
                writer(f"                _item, _data = {nested}.unpack(_data, **hints)\n")
                writer(f"                _lst.append(_item)\n")
                writer(f"            _kwargs['{name}'] = _lst\n")
                continue

            if is_optional:
                writer(f"            if len(_data) == 0:\n")
                writer(f"                _kwargs['{name}'] = None\n")
                writer(f"            else:\n")
                indent = "                "
            else:
                indent = "            "

            if is_array:
                if ctype == "char":
                    size = fdef.get("array_size", 0)
                    if size == 0:
                        writer(f"{indent}_len = hints.get('{name}_len', len(_data))\n")
                        writer(f"{indent}if len(_data) < _len:\n")
                        writer(f"{indent}    raise ValueError('not enough data to unpack {name} variable bytes')\n")
                        writer(f"{indent}_kwargs['{name}'] = _data[:_len]\n")
                        writer(f"{indent}_data = _data[_len:]\n")
                    else:
                        writer(f"{indent}if len(_data) < {size}:\n")
                        writer(f"{indent}    raise ValueError('not enough data to unpack {name}')\n")
                        writer(f"{indent}_raw = _data[:{size}]\n")
                        writer(f"{indent}_kwargs['{name}'] = _raw.split(b'\\x00', 1)[0].decode('ascii', errors='ignore')\n")
                        writer(f"{indent}_data = _data[{size}:]\n")
                else:
                    size = fdef.get("array_size", 0)
                    if size and size > 0:
                        elem_size = CTYPE[ctype][3]
                        total = size * elem_size
                        writer(f"{indent}if len(_data) < {total}:\n")
                        writer(f"{indent}    raise ValueError('not enough data to unpack {name}')\n")
                        writer(f"{indent}_tmp_v = []\n")
                        writer(f"{indent}_tmp = _data[:{total}]\n")
                        writer(f"{indent}for _i in range({size}):\n")
                        writer(f"{indent}    _val, _tmp = _unpack_scalar('{ctype}', '{name}[{{}}]'.format(_i), _tmp)\n")
                        writer(f"{indent}    _tmp_v.append(_val)\n")
                        writer(f"{indent}_kwargs['{name}'] = _tmp_v\n")
                        writer(f"{indent}_data = _data[{total}:]\n")
                    else:
                        writer(f"{indent}_alen = hints.get('{name}_len')\n")
                        writer(f"{indent}if _alen is None:\n")
                        writer(f"{indent}    raise ValueError('missing hint: {name}_len for variable length array')\n")
                        writer(f"{indent}elem_size = {CTYPE[ctype][3]}\n")
                        writer(f"{indent}total = int(_alen) * elem_size\n")
                        writer(f"{indent}if len(_data) < total:\n")
                        writer(f"{indent}    raise ValueError('not enough data to unpack {name}')\n")
                        writer(f"{indent}_tmp_v = []\n")
                        writer(f"{indent}_tmp = _data[:total]\n")
                        writer(f"{indent}for _i in range(int(_alen)):\n")
                        writer(f"{indent}    _val, _tmp = _unpack_scalar('{ctype}', '{name}[{{}}]'.format(_i), _tmp)\n")
                        writer(f"{indent}    _tmp_v.append(_val)\n")
                        writer(f"{indent}_kwargs['{name}'] = _tmp_v\n")
                        writer(f"{indent}_data = _data[total:]\n")
            else:
                if ctype == "char":
                    writer(f"{indent}if len(_data) < 1:\n")
                    writer(f"{indent}    raise ValueError('not enough data to unpack {name}')\n")
                    writer(f"{indent}_kwargs['{name}'] = _data[:1].decode('ascii', errors='ignore')\n")
                    writer(f"{indent}_data = _data[1:]\n")
                else:
                    writer(f"{indent}_val, _data = _unpack_scalar('{ctype}', '{name}', _data)\n")
                    writer(f"{indent}_kwargs['{name}'] = _val\n")
        writer(f"            return cls(**_kwargs), _data\n")
    except:
        print("error with",cls_name)

def _emit_message(writer, msg_name: str, msg: Dict[str, Any]) -> None:
    class_name = _sanitize_class(msg_name)
    print(msg)
    code = msg.get("code")
    writer(f"\nclass {class_name}:\n")
    writer(f"    CODE = {int(code)}\n")
    writer(f"    MSPV = {int(msg.get('mspv', 1))}\n")
    # Escape triple quotes in description to keep valid Python source
    raw_desc = (msg.get('description') or '')
    safe_desc = raw_desc.replace('\"\"\"', '\\\"\\\"\\\"')
    writer(f'    """{safe_desc}"""\n')
    req = msg.get("request")
    rep = msg.get("reply")
    if not req and not rep:
        writer(f"    pass\n")
        return
    if req:
        _emit_dataclass(writer, "Request", req, "REQ")
        writer("\n")
    if rep:
        _emit_dataclass(writer, "Reply", rep, "REP")
        writer("\n")

def main() -> None:
    if len(sys.argv) != 3:
        print("Usage: python gen_msp_messages.py <input_json_path> <output_py_path>", file=sys.stderr)
        sys.exit(2)
    in_path = sys.argv[1]
    out_path = sys.argv[2]
    with open(in_path, "r", encoding="utf-8") as f:
        schema = json.load(f)

    lines: List[str] = []
    def w(s: str) -> None:
        lines.append(s)

    # Header of generated module
    w("# Auto-generated by gen_msp_messages.py. Do not edit by hand.\n")
    w("from __future__ import annotations\n")
    w("from dataclasses import dataclass, field\n")
    w("from typing import Any, Dict, List, Tuple, Optional\n")
    w("import struct\n")
    w("\n")
    # Embed helpers in generated file
    w("CTYPE = {\n")
    for k, v in CTYPE.items():
        lo = "None" if v[1] is None else str(v[1])
        hi = "None" if v[2] is None else str(v[2])
        w(f"    '{k}': ('{v[0]}', {lo}, {hi}, {v[3]}),\n")
    w("}\n\n")
    w("def _assert_int_in_range(name: str, v: int, lo, hi):\n")
    w("    if not isinstance(v, int):\n")
    w("        raise TypeError(f\"{name} must be int, got {type(v).__name__}\")\n")
    w("    if lo is not None and v < lo:\n")
    w("        raise ValueError(f\"{name} {v} < {lo}\")\n")
    w("    if hi is not None and v > hi:\n")
    w("        raise ValueError(f\"{name} {v} > {hi}\")\n")
    w("\n")
    w("def _pack_scalar(ctype: str, name: str, v: Any) -> bytes:\n")
    w("    if ctype == 'char':\n")
    w("        raise RuntimeError('scalar char must be handled by array logic')\n")
    w("    fmt, lo, hi, _ = CTYPE[ctype]\n")
    w("    if lo is not None:\n")
    w("        _assert_int_in_range(name, v, lo, hi)\n")
    w("    elif ctype == 'float':\n")
    w("        if not isinstance(v, (int, float)):\n")
    w("            raise TypeError(f\"{name} must be float compatible\")\n")
    w("        v = float(v)\n")
    w("    return struct.pack(fmt, v)\n")
    w("\n")
    w("def _unpack_scalar(ctype: str, name: str, data: bytes):\n")
    w("    if ctype == 'char':\n")
    w("        raise RuntimeError('scalar char must be handled by array logic')\n")
    w("    fmt, _, _, size = CTYPE[ctype]\n")
    w("    if len(data) < size:\n")
    w("        raise ValueError(f'not enough data to unpack {name} ({ctype}) need {size} have {len(data)}')\n")
    w("    v = struct.unpack(fmt, data[:size])[0]\n")
    w("    return v, data[size:]\n")
    w("\n")

    for msg_name, msg in schema.items():
        _emit_message(w, msg_name, msg)

    out = "".join(lines)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(out)

if __name__ == "__main__":
    main()
