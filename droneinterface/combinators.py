from .messages import wrappers, wrappermap, AttitudeQuaternion, LocalPositionNED, ScaledIMU
from flightanalysis import State
from typing import List, Any, Dict
from geometry import Transformation
from flightanalysis.base import Time
from threading import Thread

combinators = {}

class Combinator:
    wrappers = dict()
    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle
        self.ids:List[int] = [wr.id for wr in self.__class__.wrappers.values()]

    def __init_subclass__(cls):
        combinators[cls.output.__name__] = cls  

    def _prepare(self, request: str, *args, **kwargs):       
        return tuple(self.vehicle.async_messages(request, self.ids, *args, **kwargs))

#        return tuple(getattr(self.vehicle, f"{request}_{v.__name__}")() for v in self.wrappers.values())    
   

class StateMaker(Combinator):
    output = State
    wrappers = dict(
        matt=AttitudeQuaternion,
        mpos=LocalPositionNED,
        macc=ScaledIMU
    )

    def generate(self, request: str, *args, **kwargs) -> State:
        matt, mpos, macc=self._prepare(request, *args, **kwargs)
        att = self.vehicle.flightline.transform_to.apply(matt.att)
        to_body = lambda p : att.inverse().transform_point(self.vehicle.flightline.transform_to.apply(p))
        
        return State.from_transform(
            Transformation(
                self.vehicle.flightline.transform_to.apply(mpos.position),
                att   
            ),
            time = Time.now(),
            vel = to_body(mpos.velocity),
            rvel = to_body(matt.rvel),
            acc = to_body(macc.acc),
            #racc = to_body()  # TODO no idea where to get this from
        )
    
def append_combinators(obj, reduced_ids: List[int] = None) -> None:    
    for cname, Combi in combinators.items():
        combi = Combi(vehicle=obj)
        if reduced_ids is None or all([id in reduced_ids for id in combi.ids]):
            setattr(obj, cname.lower(), combi)
            setattr(obj, f"last_{cname.lower()}", lambda *args, **kwargs: combi.generate("last", *args, **kwargs))
            setattr(obj, f"get_{cname.lower()}", lambda *args, **kwargs: combi.generate("get", *args, **kwargs))
            setattr(obj, f"next_{cname.lower()}", lambda *args, **kwargs: combi.generate("next", *args, **kwargs))
