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
        self.last_values = None

        
    @property
    def ids(self) -> List[int]:
        return [wr.id for wr in self.wrappers.values()]

    def snap(self):
        self.last_values = {k: getattr(self.vehicle, v.__name__) for k, v in self.wrappers.items()} 
    
    def __getattr__(self, name):
        if name in self.wrappers:
            if self.last_values is None:
                self.snap()
            return self.last_values[name]
        raise AttributeError(f"{name} not found in {self.vehicle}")

    def __init_subclass__(cls):
        combinators[cls.output.__name__] = cls  

    def __call__(self) -> Any:
        return None
    
    
class StateMaker(Combinator):
    output = State
    wrappers = dict(
        matt=AttitudeQuaternion, 
        mpos=LocalPositionNED,
        macc=ScaledIMU
    )
       

    def __call__(self) -> State:
        self.snap()
        att = self.vehicle.flightline.transform_to.apply(self.matt.att)
        to_body = lambda p : att.inverse().transform_point(self.vehicle.flightline.transform_to.apply(p))
        
        return State.from_transform(
            Transformation(
                self.vehicle.flightline.transform_to.apply(self.mpos.position),
                att   
            ),
            time = Time.now(),
            vel = to_body(self.mpos.velocity),
            rvel = to_body(self.matt.rvel),
            acc = to_body(self.macc.acc),
            #racc = to_body()  # TODO no idea where to get this from
        )
        

def append_combinators(obj, reduced_ids: List[int] = None) -> None:    
    for cname, Combi in combinators.items():
        combi = Combi(vehicle=obj)
        if reduced_ids is None or all([id in reduced_ids for id in combi.ids]):

            setattr(obj, f"get_{cname.lower()}", combi) # TODO can this be a property?
