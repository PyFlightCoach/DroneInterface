from __future__ import annotations
from droneinterface.messages import mavlink


wrappers = {}

class ParamLink:
    def __init__(self, name: str, builder: type, *params) -> None:
        if len(params) > 1:
            assert len(params) == len(builder.cols)
        self.name = name
        self.builder = builder
        self.params = params

    def read(self, msg: mavlink.MAVLink_message):
        return self.builder(*[getattr(msg, n) for n in self.params])

    def write(self, value):
        #converts the DroneInterface type into a dict
        val = [value] if len(self.params) == 1 else value.data[0]  # assumes it derives from pfc-geometry Base
        return {name: value for name, value in zip(self.params, val)}
    
    
def wrapper_factory(name:str, msg_id: int, MsgCls: type, links: list):
    links = [ParamLink(*l) for l in links]
    def constructor(self, timestamp, *args, **kwargs):
        super(self).__init__(timestamp)
        for pl, val in zip(links[:len(args)], args):
            kwargs[pl.name] = val
        for k, v in kwargs:
            setattr(self, k, v)
    
    @classmethod
    def parser(cls, msg: MsgCls):
        assert msg.id == msg_id    
        return cls(msg._timestamp, *[pl.read(msg) for pl in links])
    
    def encoder(self) -> MsgCls:
        return MsgCls(**{l.write(getattr(self, l.name)) for l in self.links})
    
    Cls = type(
        name,
        (object,),
        dict(
            __init__ = constructor,
            parse = parser,
            encoder = encoder,
            __str__ = lambda self: f"id:{self.id}, name:{self.__class__.__name__}, time:{self.time}",
            id = msg_id
        )
    )
    
    wrappers[msg_id] = Cls
    
    return Cls


