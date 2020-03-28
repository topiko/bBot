
import sys
import time
import serial

import numpy as np
from communication import talk, listen, \
        enable_legs, disable_all, parse_cmd_dict_to_cmd #get_talk_bytes_from_cmd
from state import update_state, predict_theta, \
        check_status, update_array
from control import act, UPRIGHT_THETA

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
    cmd_dict = {'cmd_to':'wheels', 'v':0, 'phidot':0, 'cmd':[0,0,0]}

    # State dictionary handles the state of the robot
    state_dict = {'times':np.zeros(3),
                  'time_next':0,
                  'theta':np.zeros(3),
                  'thetadot':np.zeros(3),
                  'thetadotdot':np.zeros(3),
                  'theta_predict':0,
                  'v':np.zeros(3),
                  'a':np.zeros(3),
                  'phidot':np.zeros(3),
                  'target_theta':UPRIGHT_THETA}

    # Report dict is for debugging and performance evaluation
    report_dict = {'predict_times':np.zeros(3),
                   'predict_thetas':np.zeros(3)}

    # store state:
    store_arr = np.zeros((imax, 8))

    # enable the legs:
    if MODE == 'test_mpu':
        pass
    else:
        enable_legs(SER)


    while (i < imax) and (status != 'fell'): # True:

        talk(SER, parse_cmd_dict_to_cmd(cmd_dict))

        cmd_dict['cmd'] = act(state_dict)
        theta, cur_time, wait = listen(SER)

        update_state(state_dict, theta, cmd_dict, cur_time, t_add)
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
                print(report_dict['predict_times'])
                print(report_dict['predict_thetas'])

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
                                     'v', 'a']):
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
