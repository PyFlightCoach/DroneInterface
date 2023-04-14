from __future__ import annotations
from droneinterface.messages import mavlink
import inspect


wrappers = {}

class ParamLink:
    def __init__(self, name: str, builder: type, params: list, tofunc=None, fromfunc=None) -> None:            
        if len(params) > 1:
            assert len(params) == len(builder.cols)
        self.name = name
        self.builder = builder
        self.params = params
        
        self.tofunc = (lambda v: v) if tofunc is None else tofunc
        self.fromfunc = (lambda v: v) if fromfunc is None else fromfunc

    def read(self, msg: mavlink.MAVLink_message):
        return self.builder(*[self.tofunc(getattr(msg, n)) for n in self.params])

    def write(self, value):
        #converts the DroneInterface type into a dict
        val = [value] if len(self.params) == 1 else value.data[0]  # assumes it derives from pfc-geometry Base
        return {name: self.fromfunc(value) for name, value in zip(self.params, val)}
    
    
def wrapper_factory(name:str, msg_id: int, links: list, props: dict=None, set_id: int=None):
    MsgCls: type = mavlink.mavlink_map[msg_id]
    
    props = {} if props is None else props

    links = [ParamLink(*l) for l in links]

    #create the missing links
    linkedargs = [p for l in links for p in l.params]
    unlinkedargs = [a for a in inspect.getfullargspec(MsgCls.__init__)[0] if not a in linkedargs]
    unlinkedargs.remove("self")
    for ula in unlinkedargs:
        links.append(ParamLink(ula, lambda v: v, [ula]))

    def constructor(self, timestamp, *args, **kwargs):
        self.timestamp = timestamp
        for pl, val in zip(links[:len(args)], args):
            kwargs[pl.name] = val
        for k, v in kwargs.items():
            setattr(self, k, v)
        for l in links:
            if not hasattr(self, l.name):
                raise AttributeError(f"Missing required attribute {l.name} in {self.__class__.__name__}")
    
    @classmethod
    def parser(cls, msg: MsgCls):
        assert msg.id == msg_id    
        return cls(msg._timestamp, *[pl.read(msg) for pl in links])
    
    def encoder(self) -> MsgCls:
        kws = [l.write(getattr(self, l.name)) for l in links]

        return MsgCls(**{k: v for kw in kws for k, v in kw.items()})
    
    def setter(self, target_system, target_component):
        if set_id is not None:
            return mavlink.mavlink_map(set_id)(
                target_system = target_system, 
                target_component = target_component,
                **{l.write(getattr(self, l.name)) for l in links}
            )
        else:
            return None
        
    Cls = type(
        name,
        (object,),
        dict(
            __init__ = constructor,
            parse = parser,
            encoder = encoder,
            setter = setter,
            __str__ = lambda self: f"id:{self.id}, name:{self.__class__.__name__}, time:{self.timestamp}",
            id = msg_id,
            **props
        )
    )
    
    wrappers[msg_id] = Cls
    
    return Cls


