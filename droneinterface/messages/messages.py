from flightanalysis.base.collection import Collection
from . import mavlink, MesDefs, MesDef, mdefs
from uuid import uuid4

class Message:
    def __init__(self, mess: mavlink.MAVLink_message) -> None:
        self.mess = mess
        self.uid = uuid4()
    

class Messages(Collection):
    VType = Message
    uid = "uid"