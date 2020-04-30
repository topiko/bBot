"""
This module is for reporting the run.
And storing the state and commands.
"""
import time
import numpy as np
from state import update_array
from params import PI

def report(i, n_report, t_init, run_time, wait_sum, run_time_max, state_dict, cmd_dict):
    #    if i < n_report*2:
    #            reset_location(state_dict)
    #            cmd_dict['phidot'] = 0/180*PI
    t_report = time.time()
    print('Freq: ', n_report/(t_report-t_init))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t_report-t_init)))
    print('i = ', i)
    print('diff times = ', np.diff(state_dict['times']))
    print('theta = ', state_dict['theta'][0])
    print('target_theta = ', cmd_dict['target_theta'])
    print('v = ', cmd_dict['v'])
    print('a = ', state_dict['a'][0])
    print('a = ', state_dict['a'])
    print('v = ', state_dict['v'])
    print('cmd = ', cmd_dict['cmd'])
    print('x,y = [{:.0f}mm, {:.0f}mm], phi = {:.1f}deg'.format(state_dict['x'][0]*1000,
                                                               state_dict['y'][0]*1000,
                                                               state_dict['phi'][0]/(2*PI)*360))
    print('run_time = {:.1f} / {:.0f}'.format(run_time, run_time_max))
    print()
    wait_sum = 0
    t_init = time.time()
    print('times', state_dict['times'])
    state_dict['dt'] = np.diff(state_dict['times'][::-1]).mean()

def store(i, store_arr, report_dict, state_dict):
    """
    Store the run results - these are used fo analysis later.
    """
    report_dict['predict_thetas'] = update_array(report_dict['predict_thetas'],
                                                 state_dict['theta_predict'])
    report_dict['predict_times'] = update_array(report_dict['predict_times'],
                                                state_dict['times'][1] + state_dict['dt'])
    for j, key in enumerate(['times', 'theta',
                             'thetadot', 'thetadotdot',
                             'theta_measured', 'thetadot_measured',
                             'thetadotdot_measured',
                             'target_theta', 'target_thetadot',
                             'target_x', 'target_y', 'target_v',
                             'target_l',
                             'run_l',
                             'v', 'a', 'x', 'y', 'phi']):
        store_arr[i, j] = state_dict[key][1]
    store_arr[i, j+1] = report_dict['predict_times'][1]
    store_arr[i, j+2] = report_dict['predict_thetas'][1]



