from .messages import wrappers, MessageWrapper, AttitudeQuaternion, LocalPositionNEDCov
from flightanalysis import State, Box, FlightLine
from typing import List, Any, Dict
from .vehicle import Vehicle

combinators = {}

class Combinator:
    wrappers = dict()
    def __init__(self, vehicle: Vehicle, messages: Dict(str, MessageWrapper)) -> None:
        self.vehicle = vehicle
        self.messages = messages
        
    def __getattr__(self, name) -> MessageWrapper:
        if name in self.messages:
            return self.messages[name]

    def __init_subclass__(cls):
        combinators[cls.output] = cls  

    def __call__(self) -> Any:
        return None
    
    
class StateMaker(Combinator):
    output = State
    wrappers = dict(
        matt=AttitudeQuaternion, 
        mpos=LocalPositionNEDCov
    )
    
    def __call__(self) -> State:
        att = self.vehicle.flightline.transform_to(self.matt.att)
        to_body = lambda p : att.inverse().transform_point(self.vehicle.flightline.transform_to(p))
        
        return State.from_constructs(
            pos = self.vehicle.flightline.transform_to(self.mpos.position),
            att = self.vehicle.flightline.transform_to(self.matt.att),
            vel = to_body(self.mpos.velocity),
            rvel = to_body(self.matt.rvel),
            acc = to_body(self.mpos.acceleration),
            #racc = to_body()
        )
        
    