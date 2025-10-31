import json
from lib.msp_enum import *

class MSPlib:
    def __init__(self):
        with open("msp_messages.json","r") as file:
            self.libfile = json.loads(file.read())
        