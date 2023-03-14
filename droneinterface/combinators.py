from .messages import wrappers, MessageWrapper, AttitudeQuaternion, LocalPositionNED, ScaledIMU
from flightanalysis import State
from typing import List, Any, Dict
from geometry import Transformation


combinators = {}

class Combinator:
    wrappers = dict()
    def __init__(self, vehicle, messages: Dict[str, MessageWrapper]) -> None:
        self.vehicle = vehicle
        self.messages = messages
        self.last_values = None# {k: None for k, v in self.messages.items()}
    
    def snap(self):
        self.last_values = {k: getattr(self.vehicle, v.__name__) for k, v in self.messages.items()} 
    
    def __getattr__(self, name) -> MessageWrapper:
        if name in self.messages:
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
            vel = to_body(self.mpos.velocity),
            rvel = to_body(self.matt.rvel),
            acc = to_body(self.macc.acc),
            #racc = to_body()  # no idea where to get this from
        )
        

def append_combinators(obj):
    
    for cname, Combi in combinators.items():
        setattr(
            obj, 
            f"get_{cname.lower()}", 
            Combi(
                vehicle=obj, 
                messages = {k: v for k, v in Combi.wrappers.items()}
            )
        )
        