
from pytest import fixture
from droneinterface.vehicle import Vehicle
from geometry import GPS, Quaternion
from geometry.testing import assert_almost_equal
from time import sleep
from droneinterface.messages import mavlink
from droneinterface.messages import wrappers



@fixture(scope="session")
def conn() -> Vehicle:
    return Vehicle.connect('tcp:127.0.0.1:5760', 1)

def test_Vehicle(conn: Vehicle):
    msg=conn.get_message(33)
    assert msg.id == 33
    
def test_get_message(conn: Vehicle):
    home = conn.get_message(mavlink.MAVLINK_MSG_ID_HOME_POSITION)
    assert_almost_equal(
        home.home, 
        GPS(-35.363262, 149.165237)
    ) 
    

def test_subscribe(conn: Vehicle):
    observer = conn.subscribe([33])
    sleep(0.5)
    assert isinstance(observer.data[33], wrappers[33])
    observer.stop()
    

def test_arm_disarm(conn: Vehicle):
    conn.arm()
    sleep(1.0)
    assert conn.Heartbeat.armed

    conn.disarm()

    sleep(1.0)
    assert not conn.Heartbeat.armed




