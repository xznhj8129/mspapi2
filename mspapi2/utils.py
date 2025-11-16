import json
from importlib import resources

from .mspcodec import InavMSP

globals().update(InavMSP.__members__)

class MSPlib:
    def __init__(self):
        schema = resources.files("mspapi2.lib") / "msp_messages.json"
        with schema.open("r", encoding="utf-8") as file:
            self.libfile = json.load(file)
        
