from __future__ import annotations
from droneinterface.messages import mavlink
import inspect
from numbers import Number

from pandas.api.types import is_list_like

wrappers = {}

class ParamLink:
    def __init__(self, name: str, builder: type, params: list, tofuncs=None, fromfuncs=None) -> None:            
        if len(params) > 1:
            assert len(params) == len(builder.cols), f'Error making {name}, params={params}, cols={builder.cols}'
        self.name = name
        self.builder = builder
        self.params = params
        
        self.tofuncs = ParamLink._processfuncs(tofuncs, len(params))
        self.fromfuncs = ParamLink._processfuncs(fromfuncs, len(params))

    @staticmethod
    def _processfuncs(funcs, n):
        if not is_list_like(funcs):
            funcs = [funcs for _ in range(n)]
        
        return [ParamLink._processfunc(func) for func in funcs]

    @staticmethod
    def _processfunc(func):
        if inspect.isfunction(func):
            return func
        elif isinstance(func, Number):
            return lambda v: v * func
        elif func is None:
            return lambda v: v
        else:
            raise TypeError(f"Invalid type {type(func)}")

    def read(self, msg: mavlink.MAVLink_message):
        return self.builder(*[tof(getattr(msg, n)) for tof, n in zip(self.tofuncs, self.params)])

    def write(self, value):
        #converts the DroneInterface type into a dict
        
        val = [value] if len(self.params) == 1 else value.data[0]  # assumes it derives from pfc-geometry Base
        vos = [ff(v) for ff, v in zip(self.tofuncs, val)]
        return {name: value for name, value in zip(self.params, vos)}
    
    
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
        assert msg.__class__.id == msg_id    
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


