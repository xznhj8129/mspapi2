import json
from mspcodec import InavMSP

globals().update(InavMSP.__members__)

class MSPlib:
    def __init__(self):
        with open("msp_messages.json","r") as file:
            self.libfile = json.loads(file.read())
        
