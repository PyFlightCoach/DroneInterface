
from pytest import fixture
from droneinterface.vehicle import Vehicle
from geometry import GPS, Quaternion


    
@fixture(scope="session")
def veh():
    return Vehicle.connect('tcp:127.0.0.1:5760', 1)


def test_attr(veh):
    att = veh.attitudequaternion
    
    assert isinstance(att.attitude, Quaternion)


