import enum
import json
from pathlib import Path

from .inav_defines import InavDefines

ENUMS_JSON_PATH = Path(__file__).with_suffix(".json")

_base_scope = {
    name: value
    for name, value in vars(InavDefines).items()
    if not name.startswith("_") and isinstance(value, int)
}
_base_scope["BIT"] = lambda bit: 1 << bit


def _resolve_raw_value(raw_value, scope):
    if isinstance(raw_value, list):
        combined_value = 0
        for item in raw_value:
            combined_value |= _resolve_raw_value(item, scope)
        return combined_value
    if not isinstance(raw_value, str):
        raise TypeError(f"Unsupported enum literal {raw_value!r}")
    if raw_value == "":
        raise ValueError("Empty enum literal")
    if raw_value.startswith("'") and raw_value.endswith("'") and len(raw_value) == 3:
        return ord(raw_value[1])
    text = raw_value.strip()
    try:
        return int(text, 0)
    except ValueError:
        pass
    return eval(text, {}, scope)


class InavEnums:
    pass


_skipped_enum_members = {}

for enum_name, entries in json.loads(ENUMS_JSON_PATH.read_text()).items():
    scope = dict(_base_scope)
    resolved = {}
    skipped = []
    for member_name, raw_value in entries.items():
        if member_name == "_source":
            continue
        scope.update(resolved)
        try:
            value = _resolve_raw_value(raw_value, scope)
        except (NameError, SyntaxError, TypeError, ValueError):
            skipped.append(member_name)
            continue
        if not isinstance(value, int):
            skipped.append(member_name)
            continue
        resolved[member_name] = value
    setattr(InavEnums, enum_name, enum.IntEnum(enum_name, resolved))
    if skipped:
        _skipped_enum_members[enum_name] = tuple(skipped)

SKIPPED_ENUM_MEMBERS = _skipped_enum_members
