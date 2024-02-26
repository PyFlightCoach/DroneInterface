import plotly.graph_objects as go
from collections import deque
from droneinterface import Vehicle
from time import time, sleep

import dash

import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
from multiprocessing import Process, Array
from threading import Thread
from json import dumps
from droneinterface import enable_logging

enable_logging('DEBUG')

msgkeys = [0, 26, 31, 32]


start = time()


def run(x, *ys):
	app = dash.Dash()
	
	fig = go.FigureWidget()
	for k in msgkeys:
		fig.add_scatter(name=k)
	
	app.layout = html.Div([
		dcc.Graph(id='live-update-graph'),
		html.Br(),
		html.Div(id='info-text'),
		dcc.Interval(
			id='interval-component',
			interval=1*1000, # in milliseconds
			n_intervals=0
		)
	])

	@app.callback(Output('live-update-graph', 'figure'),
					Input('interval-component', 'n_intervals'))
	def update_fig(n):
		with fig.batch_update():
			for d, y in zip(fig.data, ys):
				d.x = list(x)
				d.y = list(y)
		return fig

	@app.callback(Output('info-text', 'children'),
				Input('interval-component', 'n_intervals'))
	def update_text(n):
		return dumps({k: list(v) for k, v in zip(msgkeys, ys)}, indent=2)
	
	app.run_server()
	

x = deque(maxlen = 20)
ys = {k: deque(maxlen = 20) for k in msgkeys}


_x = Array("f", 20)
_ys = [Array("f", 20) for _ in msgkeys]

def update_arrs():
	_x[:len(x)] = list(x)[:]
	for i, y in enumerate(ys.values()):
		_ys[i][:len(y)] = list(y)[:]

server_process = Process(target=run, args=(_x,*_ys))
server_process.start()


vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, 1, "log_tmp")

def update_traces():
	while True:
		x.append(time() - start)
		data = vehicle.conn.rates(msgkeys)

		for k, v in zip(msgkeys, data):
			ys[k].append(v)
		update_arrs()
		sleep(1)

def cycle_rates():
	while True:
		sleep(5)
		with vehicle.subscribe(vehicle.state.ids, 10):
			sleep(5)
		


if __name__ == '__main__':

	th = Thread(target=update_traces, daemon=True)
	th.start()


	th1 = Thread(target=cycle_rates, daemon=True)
	th1.start()


	input("end of script")