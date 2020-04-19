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
from params import UPRIGHT_THETA, PI, DT, CTRL_PARAMS_DICT, OPM_LOOP_TIME
from kalman import get_patric_kalman
from score import score_run
from scipy.optimize import minimize

if len(sys.argv) == 2:
    MODE = sys.argv[1]
else:
    MODE = 'test_mpu'

if MODE.startswith('simul'):
    from simulate import display_simulation, \
            simulate_patric, plot_dynamics
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
PRINT_REPORT = False

def balance_loop(ser, run_time_max=10,
                 cmd_dict=None,
                 state_dict=None,
                 ctrl_params_dict=None,
                 kl=None):
    """
    The main loop where patric is balancing.
    """
    i = 0
    wait_sum = 0
    n_report = 1 if MODE.startswith('simul') else 500
    dt = DT if MODE.startswith('simul') else 0.016 ##.01
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
                    'target_x':0,
                    'target_y':0,
                    'target_v':0,
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
                      'theta_predict':UPRIGHT_THETA,
                      'thetadot_predict':0,
                      'target_theta':np.zeros(3),
                      'target_thetadot':np.zeros(3),
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

    if ctrl_params_dict is None:
        ctrl_params_dict = CTRL_PARAMS_DICT

    # Report dict is for debugging and performance evaluation
    report_dict = {'predict_times':np.zeros(3),
                   'predict_thetas':np.zeros(3)}

    # store state:
    store_arr = np.zeros((imax, 16))

    # Request the Kalman filter:
    if kl is None:
        kl = get_patric_kalman(np.array([state_dict['theta'][1],
                                         state_dict['thetadot'][1]]), dt)

    # enable the legs:
    enable_legs(ser, MODE)

    run_time = 0
    init_time = state_dict['times'][0]
    while (run_time < run_time_max) and (i < imax) and (status != 'fell'): # True:

        # Send the latest command to arduino
        talk(ser, state_dict, cmd_dict)

        # Update the history of the command variables
        # before obtaining new ones:
        update_cmd_vars(state_dict, cmd_dict)

        # Get new command to be sent at next iteration:
        react(state_dict, cmd_dict, ctrl_params_dict)

        # Listen to serial as a response to above talk:
        theta, cur_time, wait = listen(ser, mode=MODE)

        # Update the state of the system with the input from serial:
        update_state(state_dict, kl, theta, cur_time, dt)

        # To see how much time has been spent
        # waiting for the arduino to respond:
        wait_sum += wait

        # Check the patric status:
        status = check_status(state_dict)

        if i%n_report == 0:
            if i < n_report*2:
                reset_location(state_dict)
                cmd_dict['phidot'] = 0/180*PI
            t_report = time.time()
            if PRINT_REPORT:
                print('Freq: ', n_report/(t_report-t_init))
                print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t_report-t_init)))
                print('i = ', i)
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
                if not MODE.startswith('simul'):
                    state_dict['dt'] = dt

        if REPORT:
            report_dict['predict_thetas'] = update_array(report_dict['predict_thetas'],
                                                         state_dict['theta_predict'])
            report_dict['predict_times'] = update_array(report_dict['predict_times'],
                                                        state_dict['times'][1] + dt)
            for j, key in enumerate(['times', 'theta',
                                     'thetadot', 'thetadotdot',
                                     'theta_measured', 'thetadot_measured',
                                     'thetadotdot_measured',
                                     'target_theta', 'target_thetadot',
                                     'v', 'a', 'x', 'y', 'phi']):
                store_arr[i, j] = state_dict[key][1]
            store_arr[i, j+1] = report_dict['predict_times'][1]
            store_arr[i, j+2] = report_dict['predict_thetas'][1]

        predict_theta(state_dict, cmd_dict, kl)
        run_time = cur_time - init_time
        i += 1

    print(status, i)
    disable_all(ser, state_dict)

    return store_arr[3:i], ser, status, cmd_dict, state_dict, kl


def optimize_params():
    """
    Optimizes various parameters using some optimization...
    """
    ctrl_params_dict = CTRL_PARAMS_DICT
    def params_to_dict_(params):
        ctrl_params_dict['kappa_theta'] = params[0]
        ctrl_params_dict['gamma_theta'] = params[1]*10
        return ctrl_params_dict
    def dict_to_params_():
        return [ctrl_params_dict['kappa_theta'],
                ctrl_params_dict['gamma_theta']/10]

    def opm_callback_(params): #, opm_state):
        print('Current params: ', params)
        np.save('ctrl_params.npy', params_to_dict_(params))


    def get_balance_score_from_params_(params):
        """
        Unpact the params to ctrl_params dict and
        run the balance loop.
        Return:
            score: balance run score
        """
        ser = simulate_patric(dt=DT)
        ctrl_params_dict = params_to_dict_(params)
        run_array = None
        while run_array is None:
            run_array = run_balancing(ser,
                                      ctrl_params_dict=ctrl_params_dict,
                                      run_time_max=OPM_LOOP_TIME)

        #plot_dynamics(run_array)
        return score_run(run_array)

    init_params = dict_to_params_()
    minimize(get_balance_score_from_params_, init_params,
             options={'eps':.1},
             bounds=[(0, 3), (10, 200)],
             method='L-BFGS-B', callback=opm_callback_)
    return

def run_balancing(ser, ctrl_params_dict=None, run_time_max=10):
    """
    Starts the balancing whenever robot is
    at UPRIGHT angle.
    """
    if MODE.startswith('simul'):
        theta = UPRIGHT_THETA
    else:
        talk(SER, {'mode':MODE}, {'cmd': [0, 0, 0]})
        theta, _, _ = listen(SER)

    if (np.round(time.time(), 1) - int(time.time())) % .5 == 0:
        print('theta = {:.2f}'.format(theta))
    if abs(theta - UPRIGHT_THETA) < 1:
        print('Run Loop')
        run_data = balance_loop(ser,
                                run_time_max=run_time_max,
                                ctrl_params_dict=ctrl_params_dict)[0]
        np.save('orient.npy', run_data)
        disable_all(SER, {'mode':MODE})
        return run_data
    return None

if __name__ == '__main__':
    if MODE == 'simulate':
        display_simulation(SER, balance_loop)
    elif MODE == 'simuloptimize':
        optimize_params()
    elif MODE == 'optimize':
        optimize_params()
    else:
        try:
            while True:
                if run_balancing(SER) is not None:
                    print('Sleeping')
                    time.sleep(5)
        except KeyboardInterrupt:
            print('Disabling')
            disable_all(SER, {'mode':''})

