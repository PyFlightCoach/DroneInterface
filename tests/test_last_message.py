from droneinterface.last_message import LastMessage
from pathlib import Path
import numpy as np
import pandas as pd
from pytest import fixture
from droneinterface.messages import mavlink
from tempfile import TemporaryFile

@fixture
def lm_csv():
    return LastMessage.build_csv(Path("tests/test_data/Conn/4_32.csv"))


def test_build_csv(lm_csv):
    assert lm_csv.id == 32
    df = lm_csv.all_messages()
    assert isinstance(df, pd.DataFrame)

    assert len(df) > 0

    assert not lm_csv.last_message is None
    assert lm_csv.rate > 0

def test_create_message(lm_csv):
    df = lm_csv.all_messages()
    msg = lm_csv.create_message(df.iloc[0])
    assert isinstance(msg, mavlink.mavlink_map[lm_csv.id])
    assert msg._timestamp == df.index[0]


@fixture
def temp_csv():
    tf = Path("tests/test_data/testcsv.csv")
    yield tf
    tf.unlink()


def test_store_hb(temp_csv):
    
    lm = LastMessage._build_mavlink(mavlink.mavlink_map[0], temp_csv)
    hb = mavlink.MAVLink_heartbeat_message(1,0,1,1,1,2)
    hb._timestamp = 0
    lm.receive_message(hb)

    assert len(lm.all_messages()) == 1

    