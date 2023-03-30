from droneinterface.messages.wrappers import *


def test_wrappers_dict_populated():
    assert mavlink.MAVLINK_MSG_ID_HOME_POSITION in wrappers