import json
from mspcodec import MultiWii

globals().update(MultiWii.__members__)

class MSPlib:
    def __init__(self):
        with open("msp_messages.json","r") as file:
            self.libfile = json.loads(file.read())
        
