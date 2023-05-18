from droneinterface.last_message import LastMessage
from pathlib import Path
import numpy as np
import pandas as pd

def test_build_csv():
    lm = LastMessage.build_csv(Path("tests/4_32.csv"))
    assert lm.id == 32
    df = lm.all_messages()
    assert isinstance(df, pd.DataFrame)

    assert len(df) > 0