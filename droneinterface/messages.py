from pymavlink.dialects.v20.all import *
from flightanalysis.base.collection import Collection



class Messages(Collection):
    VType = type
    uid = "msgname"
    
    
    def id_lookup(self, id):
        for v in self:
            if v.id == id:
                return v

messages = Messages([Mess for Mess in MAVLink_message.__subclasses__() if hasattr(Mess, "msgname")])

    
pass