from droneinterface import Connection
from pytest import fixture
from pathlib import Path



@fixture
def conn_csv():
    return Connection(outdir = Path("tests/test_data/Conn"))


def test_csv_msgs_parsed(conn_csv):
    assert 4 in conn_csv.msgs
    assert 0 in conn_csv.msgs[4]


