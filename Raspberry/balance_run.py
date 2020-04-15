"""
This is the main balancing loop.
"""
import sys
import time

import numpy as np
from communication import talk, listen, \
        enable_legs, disable_all, initialize_state_dict
from state import update_state, predict_theta, \
        check_status, update_array, \
        update_cmd_vars, reset_location
from control import react
from params import UPRIGHT_THETA, PI, DT
from kalman import get_patric_kalman

if len(sys.argv) == 2:
    MODE = sys.argv[1]
else:
    MODE = 'test_mpu'

if MODE == 'simulate':
    from simulate import display_simulation, simulate_patric
    SER = simulate_patric(dt=DT)
else:
    import serial
    SER = serial.Serial(
        port='/dev/serial0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=.001
    )

REPORT = True

def balance_loop(ser, run_time_max=10, cmd_dict=None, state_dict=None, kl=None):
    """
    The main loop where patric is balancing.
    """
    i = 0
    wait_sum = 0
    n_report = 1 if MODE == 'simulate' else 1000
    dt = DT if MODE == 'simulate' else 0.016 ##.01
    t_init = 0
    status = 'upright'
    imax = 10000

    # This dictionary handles the command
    if cmd_dict is None:
        cmd_dict = {'cmd_to':'wheels',
                    'v':0,
                    'a':0,
                    'phidot':0,
                    'target_theta':UPRIGHT_THETA,
                    'target_R':np.zeros(2),
                    'cmd':[0, 0, 0]}

    # State dictionary handles the state of the robot
    if state_dict is None:
        state_dict = {'times':np.zeros(3),
                      'time_next':0,
                      'theta_measured':np.zeros(3),
                      'theta':np.zeros(3),
                      'thetadot':np.zeros(3),
                      'thetadot_measured':np.zeros(3),
                      'thetadotdot':np.zeros(3),
                      'thetadotdot_measured':np.zeros(3),
                      'theta_predict':0,
                      'run_l': np.zeros(3),
                      'abs_run_l': np.zeros(3),
                      'v':np.zeros(3),
                      'a':np.zeros(3),
                      'phi':np.zeros(3),
                      'phidot':np.zeros(3),
                      'x':np.zeros(3),
                      'y':np.zeros(3),
                      'dt':dt,
                      'mode':MODE}

        initialize_state_dict(ser, state_dict)

    # Report dict is for debugging and performance evaluation
    report_dict = {'predict_times':np.zeros(3),
                   'predict_thetas':np.zeros(3)}

    # store state:
    store_arr = np.zeros((imax, 14))

    # Request the Kalman filter:
    if kl is None:
        kl = get_patric_kalman(np.array([state_dict['theta'][1],
                                         state_dict['thetadot'][1]]), dt)

    # enable the legs:
    if MODE in ['test_mpu', 'simulate']:
        pass
    else:
        enable_legs(ser)

    run_time = 0
    init_time = state_dict['times'][0]
    while (run_time < run_time_max) and (i < imax) and (status != 'fell'): # True:

        # Send the latest command to arduino
        talk(ser, state_dict, cmd_dict)

        # Update the history of the command variables
        # before obtaining new ones:
        update_cmd_vars(state_dict, cmd_dict)

        # Get new command to be sent at next iteration:
        react(state_dict, cmd_dict)

        # Listen to serial as a response to above talk:
        theta, cur_time, wait = listen(ser, mode=MODE)

        # Update the state of the system with the input from serial:
        update_state(state_dict, kl, theta, cur_time, dt)

        # To see how much time has been spent
        # waiting for the arduino to respond:
        wait_sum += wait

        if i > 5:
            status = check_status(state_dict)

        if i%n_report == 0:
            if i < n_report*2:
                reset_location(state_dict)
                cmd_dict['phidot'] = 0/180*PI
            t_report = time.time()
            print('Freq: ', n_report/(t_report-t_init))
            print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t_report-t_init)))
            if REPORT:
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
            if i != 0:
                dt = np.diff(state_dict['times'][::-1]).mean()

        if REPORT:
            report_dict['predict_thetas'] = update_array(report_dict['predict_thetas'],
                                                         state_dict['theta_predict'])
            report_dict['predict_times'] = update_array(report_dict['predict_times'],
                                                        state_dict['times'][1] + dt)
            for j, key in enumerate(['times', 'theta',
                                     'thetadot', 'thetadotdot',
                                     'theta_measured', 'thetadot_measured',
                                     'thetadotdot_measured',
                                     'v', 'a', 'x', 'y', 'phi']):
                store_arr[i, j] = state_dict[key][1]
            store_arr[i, j+1] = report_dict['predict_times'][1]
            store_arr[i, j+2] = report_dict['predict_thetas'][1]

        predict_theta(state_dict, cmd_dict, kl)
        run_time = cur_time - init_time
        i += 1

    print(status)
    if MODE not in ['simulate']:
        disable_all(ser, state_dict)

    return store_arr[3:i], ser, status, cmd_dict, state_dict, kl




def run_balancing():
    """
    Starts the balancing whenever robot is
    at UPRIGHT angle.
    """
    while True:
        try:
            talk(SER, {'mode':MODE}, {'cmd': [0, 0, 0]})
            theta, _, _ = listen(SER)
            print('theta = {:.2f}'.format(theta))
            if abs(theta - UPRIGHT_THETA) < 1:
                run_data = balance_loop(SER, run_time_max=30)[0]
                np.save('orient.npy', run_data)
        except KeyboardInterrupt:
            print('Disabling.')
            break

    disable_all(SER, {'mode':''})

if __name__ == '__main__':
    if MODE == 'simulate':
        display_simulation(SER, balance_loop)
    else:
        run_balancing()
