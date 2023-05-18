import numpy as np
import pandas as pd
from pathlib import Path
from droneinterface import Connection
import plotly.express as px
from time import sleep

folder = Path("log_tmp/flight_1_shakedown")

def last_run():
    dates = {Connection.parse_date(run): run for run in folder.glob("Conn_*")}
    if len(dates) ==0:
        return None
    return dates[max(dates.keys())]

def last_recording(path):
    recordings = {int(p.name.split("_")[-1]): p for p in path.glob("sweep_*") if (p / "summary.csv").exists()}
    if len(recordings) ==0:
        return None
    return recordings[max(recordings.keys())]


def plotsummary(name, df):
    px.scatter(x=df.index, y=df.current * df.voltage, title=name).show()


if __name__ == "__main__":
    plotted_run = None
    plotted_recording = None

    while True:
        runtoplot = last_run()
        recording_to_plot = last_recording(runtoplot)
        if not recording_to_plot==plotted_recording or not runtoplot==plotted_run:
            if recording_to_plot is None or runtoplot is None:
                continue
            summarycsv = recording_to_plot / "summary.csv"
            if summarycsv.exists():
                summarydf = pd.read_csv(recording_to_plot / "summary.csv")
                plotsummary(f"{runtoplot.name} {recording_to_plot.name}", summarydf)
                plotted_run = runtoplot
                plotted_recording = recording_to_plot
        sleep(5)
