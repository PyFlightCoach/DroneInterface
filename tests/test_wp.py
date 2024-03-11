from pytest import fixture
from pymavlink.mavwp import MAVWPLoader



@fixture
def wps():
    wp = MAVWPLoader(1,1)
    wp.load('tests/test_data/way.txt')
    return wp


def test_loaded_points(wps):
    pass