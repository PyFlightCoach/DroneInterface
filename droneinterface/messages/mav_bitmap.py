from droneinterface.messages import mavlink
from functools import partial
enums = {}

def mav_bitmap(name):
    if name not in enums:
        enum = mavlink.enums[name]
        lookup = {v.name.lower(): k for k, v in enum.items()}

        def constructor(self, value):
            self.value = value

        def getattr(self, name):
            return lookup[name] & self.value == lookup[name]

        def repr(self):
            return f"{self.__class__.__name__}({self.value}:{[k for k, v in lookup.items() if v & self.value == v]})"

        attrs = {name: property(partial(getattr, name=name)) for name in lookup.keys()}

        enums[name] = type(
            name,
            (object,),
            dict(
                lookup = lookup,
                __init__ = constructor,
                __repr__ = repr,
                data = property(lambda self: [self.value]),
                **attrs
            )
        )
    return enums[name]