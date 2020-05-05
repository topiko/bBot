
import sys
import os
import subprocess
import datetime

from plot_run import plot_dynamics

import numpy as np

if len(sys.argv) == 2:
    MODE = sys.argv[1]
else:
    MODE = 'read_rpi'


if MODE == 'read_rpi':
    subprocess.call(['scp', 'sexybot:bBot/Raspberry/orient.npy', '.'])
    orient_arr = np.load('orient.npy')
elif MODE == 'latest':
    orient_arr = np.load('orient.npy')
else:
    files = os.listdir('./datas')
    try:
        orient_arr = np.load('datas/' + MODE)
    except FileNotFoundError:
        print('Invalid fname: {}'.format(MODE))
        orient_arr = np.load('datas/' + files[0])


plot_dynamics(orient_arr)

if MODE == 'read_rpi':
    str_time = datetime.datetime.now().strftime("%d-%m_%H:%M")
    np.save('datas/run_date-{}.npy'.format(str_time), orient_arr)



