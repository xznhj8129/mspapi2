import enum
import json
from importlib import resources
from typing import List

from .mspcodec import InavMSP

globals().update(InavMSP.__members__)

def msp_override_mask(channels: List[str]):
    result = 0
    for position in channels:
        result |= (1 << position-1)
    print("Enter the following into CLI:")
    print("set msp_override_channels = ", result)

def format_nested_dict(data, *, indent_step: int = 4, start_indent: int = 0) -> str:
    """
    Render nested dicts/lists as indented lines keyed by depth.
    """
    lines = []
    base_pad = start_indent * indent_step

    def render_scalar(value):
        if isinstance(value, enum.Enum):
            return repr(value)
        if isinstance(value, str):
            return value
        return repr(value)

    def walk(node, depth: int) -> None:
        pad = " " * (base_pad + depth * indent_step)
        if isinstance(node, dict):
            for key, value in node.items():
                if isinstance(value, (dict, list, tuple)):
                    lines.append(f"{pad}{key}:")
                    walk(value, depth + 1)
                    continue
                lines.append(f"{pad}{key}: {render_scalar(value)}")
            return
        if isinstance(node, (list, tuple)):
            for item in node:
                if isinstance(item, (dict, list, tuple)):
                    lines.append(f"{pad}-")
                    walk(item, depth + 1)
                    continue
                lines.append(f"{pad}- {render_scalar(item)}")
            return
        lines.append(f"{pad}{render_scalar(node)}")

    walk(data, 0)
    return "\n".join(lines)

class MSPlib:
    def __init__(self):
        schema = resources.files("mspapi2.lib") / "msp_messages.json"
        with schema.open("r", encoding="utf-8") as file:
            self.libfile = json.load(file)
        
