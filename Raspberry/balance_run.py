"""
This is the main balancing loop.
"""
import sys
import time

import numpy as np
from communication import talk, listen, \
        enable_legs, disable_all, initialize_state_dict
from state import update_state, predict_theta, \
        check_status, \
        update_cmd_vars, reset_location
from control import react
from params import UPRIGHT_THETA, DT, CTRL_PARAMS_DICT, \
        OPM_LOOP_TIME, AMPLITUDE, OPM_METHOD, RUN_LOOP_TIME
from kalman import get_patric_kalman
from score import score_run
from scipy.optimize import minimize, brute
from report import report, store
from goto import get_x_v_a

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
        timeout=.05
    )

STORE_RUN = True
PRINT_REPORT = True #False
N_REPORT = 1 if MODE.startswith('simulate') else 200
if MODE == 'simuloptimize':
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
                    'target_l':0,
                    'target_x':0,
                    'target_y':0,
                    'target_v':0,
                    'target_a':0,
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
                      'target_l':np.zeros(3),
                      'target_x':np.zeros(3),
                      'target_y':np.zeros(3),
                      'target_v':np.zeros(3),
                      'target_a':np.zeros(3),
                      'run_l': np.ones(3)*AMPLITUDE,
                      'abs_run_l': np.zeros(3),
                      'v':np.zeros(3),
                      'a':np.zeros(3),
                      'phi':np.zeros(3),
                      'phidot':np.zeros(3),
                      'x':np.ones(3)*AMPLITUDE,
                      'y':np.zeros(3),
                      'dt':DT,
                      'mode':MODE}

        initialize_state_dict(ser, state_dict)

    if ctrl_params_dict is None:
        ctrl_params_dict = CTRL_PARAMS_DICT

    # store state:
    if STORE_RUN:
        # Report dict is for debugging and performance evaluation
        report_dict = {'predict_times':np.zeros(3),
                       'predict_thetas':np.zeros(3)}
        store_arr = np.zeros((imax, 22))

    # Request the Kalman filter:
    if kl is None:
        kl = get_patric_kalman(np.array([state_dict['theta'][1],
                                         state_dict['thetadot'][1]]), DT)

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
        update_state(state_dict, kl, theta, cur_time)

        # To see how much time has been spent
        # waiting for the arduino to respond:
        wait_sum += wait

        # Check the patric status:
        if run_time > .5:
            status = check_status(state_dict)

        # report
        if PRINT_REPORT and (i%N_REPORT == 0):
            report(i, N_REPORT, t_init,
                   run_time, wait_sum,
                   run_time_max,
                   state_dict,
                   cmd_dict)
        # store
        if STORE_RUN:
            store(i, store_arr, report_dict, state_dict)

        # Predict the theta and thetadot at next time instance
        predict_theta(state_dict, cmd_dict, kl)

        # Extract current runtime
        run_time = cur_time - init_time

        # Quick test of location updates
        cmd_dict['target_l'], cmd_dict['target_v'], cmd_dict['target_a'] \
                = get_x_v_a(run_time, AMPLITUDE, state_dict)

        # Update i
        i += 1

    print(status)
    disable_all(ser, state_dict)

    return store_arr[3:i], ser, status, cmd_dict, state_dict, kl


def optimize_params():
    """
    Optimizes various parameters using some optimization...
    """
    ctrl_params_dict = CTRL_PARAMS_DICT
    def params_to_dict_(params):
        ctrl_params_dict['damp_pos'] = params[0]
        ctrl_params_dict['omega_pos'] = params[1]
        ctrl_params_dict['damp_theta'] = params[2]
        ctrl_params_dict['omega_theta'] = params[3]

        return ctrl_params_dict
    def dict_to_params_():
        return [ctrl_params_dict['damp_pos'],
                ctrl_params_dict['omega_pos'],
                ctrl_params_dict['damp_theta'],
                ctrl_params_dict['omega_theta']]

    def opm_callback_(params): #, opm_state):
        np.save('ctrl_params.npy', params_to_dict_(params))
        if MODE.startswith('simul'):
            ser = simulate_patric(dt=DT)
            run_array = run_balancing(ser,
                                      ctrl_params_dict=params_to_dict_(params),
                                      run_time_max=OPM_LOOP_TIME)


            plot_dynamics(run_array)

        print('Current params: ', params)

    def get_balance_score_from_params_(params):
        """
        Unpact the params to ctrl_params dict and
        run the balance loop.
        Return:
            score: balance run score
        """
        if MODE.startswith('simul'):
            ser = simulate_patric(dt=DT)
        else:
            ser = SER

        ctrl_params_dict = params_to_dict_(params)

        #run_array = None
        #status = 'upright'
        #while (run_array is None): # and (status != 'fell'):
        run_array = run_balancing(ser,
                                  ctrl_params_dict=ctrl_params_dict,
                                  run_time_max=OPM_LOOP_TIME)

        #print(ctrl_params_dict)
        score = score_run(run_array)
        print(params, score)
        return score

    init_params = dict_to_params_()
    if OPM_METHOD == 'L-BFGS-B':
        bounds = ((0, 10), (0, 20), (0, 10), (0, 50))
        res = minimize(get_balance_score_from_params_, init_params,
                       options={'eps':.010, 'ftol':1e-12, 'gtol':1e-12},
                       bounds=bounds,
                       method='L-BFGS-B', callback=opm_callback_)

    elif OPM_METHOD == 'brute':
        step = 1
        bounds = (slice(0.5, 3.5, .5),
                  slice(0, 4, .5),
                  slice(0.5, 3.5, .5),
                  slice(0, 21, 2))
        res = brute(get_balance_score_from_params_,
                    ranges=bounds, finish=None) #, Ns=20)
        opm_callback_(res)
    print(res)
    return

def run_balancing(ser,
                  ctrl_params_dict=None,
                  run_time_max=10,
                  max_diff_theta=5):
    """
    Starts the balancing whenever robot is
    at UPRIGHT angle.
    """
    while True:
        if MODE.startswith('simul'):
            theta = UPRIGHT_THETA
        else:
            talk(SER, {'mode':MODE}, {'cmd': [0, 0, 0]})
            theta, _, _ = listen(SER, mode=MODE)

        if (np.round(time.time(), 1) - int(time.time())) % .5 == 0:
            print('theta = {:.2f}'.format(theta))
        if abs(theta - UPRIGHT_THETA) < max_diff_theta:
            break

    print('Run Loop')
    run_data, _, status = balance_loop(ser,
                                       run_time_max=run_time_max,
                                       ctrl_params_dict=ctrl_params_dict)[:3]
    np.save('orient.npy', run_data)
    disable_all(SER, {'mode':MODE})
    if status != 'fell':
        return run_data
    else:
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
                run_balancing(SER,
                              run_time_max=RUN_LOOP_TIME,
                              max_diff_theta=1) # is not None:
                print('Sleeping')
                time.sleep(5)
        except KeyboardInterrupt:
            print('Disabling')
            disable_all(SER, {'mode':''})

