
from pymavlink.dialects.v20.all import MAVLink_message as v2Message
from pymavlink.dialects.v10.all import MAVLink_message as v1Message

from flightanalysis.base.collection import Collection
from typing import Union
from numbers import Number


class Messages(Collection):
    VType = type
    uid = "msgname"
    
    def __init__(self, data):
        super().__init__(data)
        self.idmap = {m.id: m for m in self}
    
    @staticmethod
    def parse_dialect(dialect="v2"):
        MM = v1Message if dialect == "v1" else v2Message
        return Messages([Mess for Mess in MM.__subclasses__() if hasattr(Mess, "msgname")])

    def from_id(self, id: Union[int, list]):
        if isinstance(id, int):
            return self.idmap[id]
        elif isinstance(id, list):
            return Messages([self.idmap[i] for i in id])
        else:
            raise AttributeError("expected an int or list")

    def subset(self, *messages):
        return Messages([getattr(self, m) for m in messages])

if __name__ == "__main__":
    messages = Messages.parse_dialect()
    pass    