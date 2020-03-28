
import sys
import time
import serial

import numpy as np
from communication import talk, listen, \
        enable_legs, disable_all
from state import update_state, predict_theta, \
        check_status, update_array, update_location, \
        update_cmd_vars
from control import update_cmd
from params import UPRIGHT_THETA

if len(sys.argv) == 2:
    MODE = sys.argv[1]
else:
    MODE = 'test_mpu'

SER = serial.Serial(
    port='/dev/serial0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=.001
)


REPORT = True

def balance_loop():
    """
    The main loop where patric is balancing.
    """
    i = 0
    wait_sum = 0
    n_report = 50
    t_init = 0
    t_add = .015
    status = 'upright'
    imax = 1000

    # This dictionary handles the command
    cmd_dict = {'cmd_to':'wheels',
                'v':0,
                'phidot':0,
                'target_theta':UPRIGHT_THETA,
                'target_R':np.zeros(2),
                'cmd':[0, 0, 0]}

    # State dictionary handles the state of the robot
    state_dict = {'times':np.zeros(3),
                  'time_next':0,
                  'theta':np.zeros(3),
                  'thetadot':np.zeros(3),
                  'thetadotdot':np.zeros(3),
                  'theta_predict':0,
                  'v':np.zeros(3),
                  'a':np.zeros(3),
                  'phi':np.zeros(3),
                  'phidot':np.zeros(3),
                  'x':np.zeros(3),
                  'y':np.zeros(3)}

    # Report dict is for debugging and performance evaluation
    report_dict = {'predict_times':np.zeros(3),
                   'predict_thetas':np.zeros(3)}

    # store state:
    store_arr = np.zeros((imax, 11))

    # enable the legs:
    if MODE == 'test_mpu':
        pass
    else:
        enable_legs(SER)


    while (i < imax) and (status != 'fell'): # True:

        # Send the latest command to arduino
        talk(SER, cmd_dict['cmd'])

        # Update the history of the command variables 
        # before obtaining new ones: 
        update_cmd_vars(state_dict, cmd_dict)

        # Get new command to be sent at next iteration:
        update_cmd(state_dict, cmd_dict)

        # Listen to serial as a response to above talk:
        theta, cur_time, wait = listen(SER)

        # Update the state of the system with the input from serial:
        update_state(state_dict, theta, cmd_dict, cur_time, t_add)
        update_location(state_dict)

        # To see how much time has been spent 
        # waiting for the arduino to respond:
        wait_sum += wait

        if i > 20:
            status = check_status(state_dict)

        if i%n_report == 0:
            t_report = time.time()
            print('Freq: ', n_report/(t_report-t_init))
            print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t_report-t_init)))
            if REPORT:
                print(state_dict['times'])
                print(state_dict['theta'])
                print('v = ', state_dict['v'])
                print('x = ', state_dict['x'])
                print('cmd = ', cmd_dict['cmd'])
                print()
                wait_sum = 0
                t_init = time.time()
            if i != 0:
                t_add = np.diff(state_dict['times'][::-1]).mean()

        if REPORT:
            report_dict['predict_thetas'] = update_array(report_dict['predict_thetas'],
                                                         state_dict['theta_predict'])
            report_dict['predict_times'] = update_array(report_dict['predict_times'],
                                                        state_dict['times'][1] + t_add)
            for j, key in enumerate(['times', 'theta',
                                     'thetadot', 'thetadotdot',
                                     'v', 'a', 'x', 'y', 'phi']):
                store_arr[i, j] = state_dict[key][1]
            store_arr[i, j+1] = report_dict['predict_times'][1]
            store_arr[i, j+2] = report_dict['predict_thetas'][1]

        predict_theta(state_dict)
        i += 1
    print(status)

    np.save('orient.npy', store_arr[3:i])
    disable_all(SER)



if __name__ == '__main__':
    balance_loop()
