from .messages import wrappers, wrappermap, AttitudeQuaternion, GlobalPositionInt, ScaledIMU
from flightdata import State
from typing import List, Any, Dict
from geometry import Transformation, Time
from threading import Thread
import numpy as np

combinators = {}

class Combinator:
    wrappers = dict()
    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle
        self.ids:List[int] = [wr.id for wr in self.__class__.wrappers.values()]

    def __init_subclass__(cls):
        combinators[cls.output.__name__] = cls  

    def _prepare(self, request: str, *args, **kwargs):
        if request == "last":
            res = tuple(self.vehicle.last_message(id, *args, **kwargs) for id in self.ids)
        else:
            res = tuple(self.vehicle.parallel_messages(request, self.ids, *args, **kwargs))
        if np.any(np.array(res) == None):
            raise Exception(f"got {[None if c is None else c.__name__ for c in res]} for {request} {self.ids}")
        else:
            return res
#        return tuple(getattr(self.vehicle, f"{request}_{v.__name__}")() for v in self.wrappers.values())    


class StateMaker(Combinator):
    output = State
    wrappers = dict(
        matt=AttitudeQuaternion,
        mpos=GlobalPositionInt,
        macc=ScaledIMU
    )

    def generate(self, request: str, *args, **kwargs) -> State:
        matt, mpos, macc=self._prepare(request, *args, **kwargs)
        
        
        to_body = Transformation(
            self.vehicle.origin.rotation.transform_point(mpos.position - self.vehicle.origin.pos),
            self.vehicle.origin.rotation * matt.att
        )
        
        return State.from_transform(
            to_body,
            time = Time.now(),
            vel = to_body.apply(mpos.velocity),
            rvel = to_body.apply(matt.rvel),
            acc = to_body.apply(macc.acc),
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
