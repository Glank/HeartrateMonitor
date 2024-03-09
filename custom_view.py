import serial
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread, Lock
import signal
import sys
import math
import time
from textwrap import dedent

LOG_PREFIX = 'exercise'
LOG_FILE = '{}_{}.txt'.format(
  'cv_log' if LOG_PREFIX is None else LOG_PREFIX,
  datetime.now().strftime("%Y%m%d_%H%M%S")
)
SAVE_TO_LOG = False
READ_FROM_LOG = None #'exercise_20240226.txt'
COM_PORT = 'COM6'
BAUD_RATE = 460800

MAX_HR_RECS = 15

TBL_FORMATTING = {
  'p': {
    'cols' : [
      ["time", lambda x:int(x)],
      ["amp", lambda x:int(x)],
    ],
    'max_recs': 2**9,
  },
  'hr': {
    'cols' : [
      ["time", lambda x:int(x)],
      ["hr", lambda x:float(x)],
      ["hr_lb", lambda x:float(x)],
      ["hr_lb", lambda x:float(x)],
      ["err", lambda x:x],
    ],
    'max_recs': 15,
  },
}

GRAPHS = {
  'HR' : {
    'screen_pos': [0.1,0.65,0.7,0.25],
    'lines': [
      {
        'name': 'valid_hr',
        'x': ['hr', 'time'],
        'y': ['hr', 'hr'],
        'fmt': 'go',
        'where': lambda r: not r[4],
        'annotate': True
      },
      {
        'name': 'invalid_hr',
        'x': ['hr', 'time'],
        'y': ['hr', 'hr'],
        'fmt': 'ro',
        'where': lambda r: r[4],
      },
    ],
  },
  'Pulse' : {
    'screen_pos': [0,0,1,0.5],
    'show_axes': False,
    'lines': [
      {
        'name': 'pulse',
        'x': ['p', 'time'],
        'y': ['p', 'amp'],
        'fmt': 'b-',
      },
    ],
  },
}

def _init_data():
  _data = {}
  for key in TBL_FORMATTING:
    _data[key] = [] 
  return _data
_data = _init_data()
_data_lock = Lock()

def add_row(tbl, new_row):
  timestamp = None
  table = _data[tbl]
  col_specs = TBL_FORMATTING[tbl]['cols']
  max_recs = TBL_FORMATTING[tbl]['max_recs']
  row_fmted = []
  for i, cell in enumerate(new_row):
    row_fmted.append(col_specs[i][1](cell))
    if col_specs[i][0] == 'time':
      timestamp = row_fmted[-1]
  with _data_lock:
    _data[tbl].append(row_fmted)
    if len(_data[tbl]) > max_recs:
      _data[tbl].pop(0)
  return timestamp

def get_col(tbl, col, where=None):
  col_specs = TBL_FORMATTING[tbl]['cols']
  try:
    idx = next(i for i,c in enumerate(col_specs) if c[0] == col)
  except StopIteration:
    raise KeyError("No column {}.{}".format(tbl, col))
  with _data_lock:
    if where is None:
      return [r[idx] if idx < len(r) else None for r in _data[tbl]]
    else:
      return [r[idx] if idx < len(r) else None for r in _data[tbl] if where(r)]

stop_program = False
stop_stream = False

def read_arduino_stream():

  # set up input stream
  readline = None
  in_s = None
  if READ_FROM_LOG is None:
    in_s = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    def r():
      try:
        return in_s.readline().decode().strip()
      except UnicodeDecodeError:
        pass
      return "" 
    readline = r
  else:
    in_s = open(READ_FROM_LOG, 'r')
    def r():
      global stop_stream
      line = in_s.readline()
      if line == "":
        stop_stream = True
      return line.strip()
    readline = lambda:in_s.readline()

  #set up output stream
  out_f = None
  if SAVE_TO_LOG:
    out_f = open(LOG_FILE, 'w')
    print("Logging to: {}".format(LOG_FILE))

  #read until end of stream or sigint
  last_timestamp = None
  try:
    while not stop_program and not stop_stream:
      line = readline()
      if not line:
        continue
      if out_f is not None:
        out_f.write(line)
        out_f.write("\n")
      parts = line.strip().split(',')
      try:
        timestamp = add_row(parts[0], parts[1:])
      finally:
        if READ_FROM_LOG:
          #simulate real-time input
          if last_timestamp:
            time.sleep((timestamp-last_timestamp)/1000.0)
          last_timestamp = timestamp
  finally:
    if out_f is not None:
      out_f.close()
    in_s.close()

thread = Thread(target=read_arduino_stream)
def signal_handler(sig, frame):
  global stop_program
  print("Stopping...")
  stop_program = True

def main():
  signal.signal(signal.SIGINT, signal_handler)
  thread.start()

  plt.ion()
  fig = plt.figure()
  def on_close(_):
    global stop_program
    stop_program = True
  fig.canvas.mpl_connect('close_event', on_close)
  fig.suptitle('HR & Pulse')
  fig.set_size_inches(16*0.8,9*0.8) 

  axes = {}
  for g in GRAPHS:
    axes[g] = fig.add_axes(GRAPHS[g]['screen_pos'])
    if not GRAPHS[g].get('show_axes', True):
      axes[g].set_axis_off()
  
  lines = {}
  annotations = {}
  for g in GRAPHS:
    lines[g] = {}
    annotations[g] = []
    line_specs = GRAPHS[g].get('lines', [])
    for line_spec in line_specs:
      lines[g][line_spec['name']] = None


  while not stop_program:
    for g, graph_spec in GRAPHS.items():
      while annotations[g]:
        ann = annotations[g].pop(-1)
        ann.remove()

      min_x = float('inf')
      max_x = float('-inf')
      min_y = float('inf')
      max_y = float('-inf')
      for line_spec in graph_spec.get('lines', []):
        l = line_spec['name']
        x_tbl, x_col = tuple(line_spec['x'])
        y_tbl, y_col = tuple(line_spec['y'])
        x = get_col(x_tbl, x_col, line_spec.get('where', None))
        y = get_col(y_tbl, y_col, line_spec.get('where', None))
        if x:
          min_x = min(min(x), min_x)
          max_x = max(max(x), max_x)
          min_y = min(min(y), min_y)
          max_y = max(max(y), max_y)
          line = lines[g][l]
          if line is None:
            lines[g][l], = axes[g].plot(x,y,line_spec['fmt'])
          else:
            lines[g][l].set_data(x,y)
          if line_spec.get('annotate', False):
            for xi,yi in zip(x,y):
              annotations[g].append(axes[g].annotate(str(yi), (xi,yi), fontsize=10, rotation='vertical'))

      if min_x != float('inf'):
        axes[g].set_xlim(min_x, max_x)
        axes[g].set_ylim(min_y, max_y)
    
    plt.draw()
    plt.pause(0.1)

if __name__ == '__main__':
  try:
    main()
  finally:
    stop_program = True
