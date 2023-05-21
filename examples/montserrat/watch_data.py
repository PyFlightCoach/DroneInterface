import numpy as np
import pandas as pd
from pathlib import Path
from droneinterface import Connection
import plotly.express as px
import plotly.graph_objects as go
from flightplotting import titlerenderer
from time import sleep




def get_runs_in_folder(folder):
    return {Connection.parse_date(run): run for run in sorted(folder.glob("Conn_*"))}

def last_run_in_folder(folder):
    dates = get_runs_in_folder(folder)
    if len(dates) ==0:
        return None
    return dates[max(dates.keys())]


def get_recordings_in_folder(runpath, require_summary=True):
    def check_summary(p):
        if not require_summary: 
            return True
        else:
            return (p / "summary.csv").exists()

    return {int(p.name.split("_")[-1]): p for p in sorted(runpath.glob("sweep_*")) if check_summary(p)}

def last_recording(runpath):
    recordings = get_recordings_in_folder(runpath)
    if len(recordings) ==0:
        return None
    return recordings[max(recordings.keys())]


def plotsummary(name, df):
    fig = go.Figure(layout=go.Layout(title=name))
    fig.update_layout(
        xaxis=dict(
            title="flap setting",
            range=(1200,1800)
        )
    )
    def tr(i, name, col, range=None):
        id = '' if i==0 else i+1
        fig.add_trace(go.Scatter(x=df.index, y=col, name=name, yaxis=f"y{id}"))  
        if i == 0: 
            fig.update_layout(
                yaxis=dict(title=name)
            )
        else: 
            fig.update_layout(
                **{
                    f"yaxis{id}": dict(
                    title=name,
                    anchor="free",
                    side="left",
                    overlaying="y",
                    position=i*0.1
                    )
                }
            )    
        if not range is None:
            fig.update_layout(**{f"yaxis{id}": dict(range=range)})
        return i+1
    i=0
    i=tr(i, "Power", df.current * df.voltage, (0, 1000))
    i=tr(i, "airspeed", df.airspeed, (0,30))
    i=tr(i, "ground_speed", df.ground_speed, (0,30))
    i=tr(i, "wind_speed", df.wind_speed, (0,30))
    i=tr(i, "wind_direction", df.wind_direction, (-180,180))
    i=tr(i, "servo_9", df.servo_9, (1000,2000))
    i=tr(i, "servo_10", df.servo_10, (1000,2000))
    return fig
    #fig.show(renderer="titleBrowser", browser_tab_title=name.split(" ")[1])




if __name__ == "__main__":
    folder = Path("/home/td6834/projects/montserrat/DroneInterfaceOutput/flight_6")
    plotted_run = None
    plotted_recording = None

    while True:
        runtoplot = last_run_in_folder(folder)
        recording_to_plot = last_recording(runtoplot)
        if not recording_to_plot==plotted_recording or not runtoplot==plotted_run:
            if recording_to_plot is None or runtoplot is None:
                continue
            summarycsv = recording_to_plot / "summary.csv"
            if summarycsv.exists():
                summarydf = pd.read_csv(recording_to_plot / "summary.csv", index_col=0)
                fig = plotsummary(f"{runtoplot.name} {recording_to_plot.name}", summarydf)
                fig.show(
                    renderer="titleBrowser", 
                    browser_tab_title=recording_to_plot.name
                )
                plotted_run = runtoplot
                plotted_recording = recording_to_plot
        sleep(5)
