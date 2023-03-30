from pytest import fixture
from droneinterface.connection import Connection
from droneinterface.messages import mavlink
from droneinterface.messages import wrappers
from geometry import GPS
from geometry.testing import assert_almost_equal

@fixture(scope="session")
def conn() -> Connection:
    return Connection.connect('tcp:127.0.0.1:5760', 1)

def test_connection(conn: Connection):
    msg=conn.get_message(33)
    assert msg.id == 33
    
def test_get_message(conn: Connection):
    home = conn.get_message(mavlink.MAVLINK_MSG_ID_HOME_POSITION)
    assert_almost_equal(
        home.home, 
        GPS(-35.363262, 149.165237)
    ) 
    

def test_subscribe(conn):
    observer = conn.subscribe([33])
    
    assert isinstance(observer.data[33], wrappers[33])
    observer.stop()
    