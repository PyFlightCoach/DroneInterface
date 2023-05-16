from droneinterface.messages import *


def test_wrappers_dict_populated():
    assert mavlink.MAVLINK_MSG_ID_HOME_POSITION in wrappers


def test_set_channel():
    set_channel_1 = RCOverride.set_channel(1,1,1,1500, "ignore")
    set_channel_1_release = RCOverride.set_channel(1,1,1,1500, "release")
    assert set_channel_1_release.chan9_raw == 2**16 -1 #A value of UINT16_MAX-1 means to release this channel back to the RC radio. [us] (type:uint16_t) 
    assert set_channel_1_release.chan8_raw == 0 # A value of 0 means to release this channel back to the RC radio.
    assert set_channel_1.chan9_raw == 2**16 #A value of UINT16_MAX means to ignore this field.
    