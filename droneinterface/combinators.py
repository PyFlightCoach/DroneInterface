from .messages import wrappers, AttitudeQuaternion, LocalPositionNED, ScaledIMU
from flightanalysis import State
from typing import List, Any, Dict
from geometry import Transformation
from flightanalysis.base import Time


combinators = {}

class Combinator:
    wrappers = dict()
    def __init__(self, vehicle) -> None:
        self.vehicle = vehicle

    def __getattribute__(self, name: str):
        return getattr(self.vehicle, name)

    @property
    def ids(self) -> List[int]:
        return [wr.id for wr in self.wrappers.values()]

    def __init_subclass__(cls):
        combinators[cls.output.__name__] = cls  

    def __call__(self) -> Any:
        return None
    
    
class StateMaker(Combinator):
    output = State

    def __call__(self) -> State:
        matt=self.last_AttitudeQuaternion 
        mpos=self.last_LocalPositionNED
        macc=self.last_ScaledIMU

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

            setattr(obj, f"last_{cname.lower()}", combi)
