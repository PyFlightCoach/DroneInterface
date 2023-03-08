
from droneinterface.messages import mavlink
from flightanalysis.base.collection import Collection
from typing import Union


class MesDefs(Collection):
    VType = type
    uid = "msgname"
    
    def __init__(self, data):
        super().__init__(data)
        self.idmap = {m.id: m for m in self}
        self.msgname = [m.msgname for m in self]
            
    def from_id(self, id: Union[int, list]):
        if isinstance(id, int):
            return self.idmap[id]
        elif isinstance(id, list):
            return [self.idmap[i] for i in id]
        else:
            raise AttributeError("expected an int or list")

    def subset(self, *messages):
        return MesDefs([getattr(self, m) for m in messages])
  
     
mdefs = MesDefs(
    [M for M in mavlink.MAVLink_message.__subclasses__() if hasattr(M, "msgname")]
)



if __name__ == "__main__":
    pass    