import plotly.graph_objects as go
from collections import deque
from droneinterface import Vehicle
from time import time, sleep
from random import random

import dash

import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import multiprocessing
from threading import Thread
from json import dumps
import logging
logging.basicConfig(level=logging.DEBUG)


app = dash.Dash()

x = deque(maxlen = 20)

msgkeys = ["1_1_0", "1_1_26", "1_1_31", "1_1_32"]

ys = {k: deque(maxlen = 20) for k in msgkeys}


app.layout = html.Div([
    dcc.Graph(id='live-update-graph'),
    dcc.Interval(
		id='interval-component',
		interval=1*1000, # in milliseconds
		n_intervals=0
	)
])

fig = go.FigureWidget()
for _ in msgkeys:
	fig.add_scatter()



vehicle = Vehicle.connect('tcp:127.0.0.1:5760', 1, 1, "log_tmp")

start = time()

@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_fig(n):
	x.append(time() - start)
	data = vehicle.conn.rates(msgkeys)

	for k, v in zip(msgkeys, data):
		ys[k].append(v)
#	print(dumps({k: list(v) for k, v in ys.items()}, indent=2))
	with fig.batch_update():
		for d, y in zip(fig.data, ys.values()):
			d.x = list(x)
			d.y = list(y)
			#print(list(y))

	return fig

def start_server(app: dash.Dash, **kwargs):
    def run():
        app.run_server(**kwargs)
    server_process = multiprocessing.Process(target=run)
    server_process.start()


def cycle_rates():
	while True:
		with vehicle.subscribe(vehicle.state.ids, 10):
			sleep(5)
		sleep(5)


if __name__ == '__main__':

	th = Thread(target=cycle_rates, daemon=True)
	th.start()

	start_server(app)

	while True:
		print(vehicle.conn.rates(msgkeys))
		sleep(1)
#	input("change rates running")
pass
