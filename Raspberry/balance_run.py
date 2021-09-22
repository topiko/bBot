"""
This is the main balancing loop.
"""
import sys
import time
import socket

import numpy as np

from communication import talk, listen, \
        enable_legs, disable_all, initialize_state_dict
from state import update_state, predict_theta, \
        check_status, \
        update_cmd_vars, reset_location
from control import react, relocate
from params import UPRIGHT_THETA, DT, CTRL_PARAMS_DICT, \
        OPM_LOOP_TIME, AMPLITUDE, OPM_METHOD, RUN_LOOP_TIME, \
        COLLECT_DATA, REMOTE_DX
from kalman import get_patric_kalman
from score import score_run
from scipy.optimize import minimize, brute, differential_evolution
from report import report, store
from goto import get_x_v_a

REMOTE = True

if len(sys.argv) == 2:
    MODE = sys.argv[1]
    REMOTE = True
else:
    REMOTE = False
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

    if REMOTE:
        # Open connection for remote
        # TODO: transfer this to paho-mqtt based. Also
        # on use ps4 remote.
        from utils import makemqttclient
        q, client = makemqttclient(["patric/control"], host="192.168.0.13")
        '''
        SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SOCKET.bind(('192.168.43.32', 8000))
        SOCKET.listen(5)
        CONN, addr = SOCKET.accept()
        print ('connection from', addr)
        CONN.setblocking(0)
        '''


STORE_RUN = True
PRINT_REPORT = True #False
N_REPORT = 1 if MODE.startswith('simulate') else 20000
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
        print('New cmd')
        cmd_dict = {'cmd_to':'wheels',
                    'v':0,
                    'a':0,
                    'phidot':0,
                    'phi':0,
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
        print('New state')
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
                      'target_thetadotdot':np.zeros(3),
                      'target_l':np.zeros(3),
                      'target_x':np.zeros(3),
                      'target_y':np.zeros(3),
                      'target_v':np.zeros(3),
                      'target_a':np.zeros(3),
                      'run_l': np.ones(3)*AMPLITUDE,
                      'abs_run_l': np.zeros(3),
                      'I_pos':0,
                      'I_theta':0,
                      'loop_idx':0,
                      'v':np.zeros(3),
                      'a':np.zeros(3),
                      'a1':np.zeros(3),
                      'a2':np.zeros(3),
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
        dtypes = [(c, np.float32) for c in COLLECT_DATA]
        store_arr = np.zeros(imax).astype(dtypes)
    # Request the Kalman filter:
    if kl is None:
        if not MODE.startswith('simul'):
            print('New KL')
            kl = get_patric_kalman(np.array([state_dict['theta'][1],
                                             state_dict['thetadot'][1]]),
                                             state_dict['dt'])
        else:
            print('WARNING: kl-filter modified since simulation.')
            kl = get_patric_kalman(np.array([state_dict['theta'][1],
                                             state_dict['thetadot'][1]]),
                                             state_dict['dt'],
                                             sigma_thetadotdot=1.00)

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
        if PRINT_REPORT and (i%N_REPORT == 0) and (i != 0):
            report(i, N_REPORT, t_init,
                   run_time, wait_sum,
                   run_time_max,
                   state_dict,
                   cmd_dict)
        # store
        if STORE_RUN:
            store(i, store_arr, state_dict)

        # Predict the theta and thetadot at next time instance
        predict_theta(state_dict, cmd_dict, kl)

        # Extract current runtime
        run_time = cur_time - init_time

        # Quick test of location updates
        #cmd_dict['target_l'], cmd_dict['target_v'], cmd_dict['target_a'], _ \
        #        = get_x_v_a(run_time, AMPLITUDE, state_dict)

        if REMOTE:
            #input from mqtt server.
            a = client.loop(timeout=0.01)
            add_x = 0
            add_phi = 0
            while not q.empty():
                if q.qsize()!=1: print("LONG QUEUE")
                message = q.get()

                msg_topic = message[0]
                message = message[1]
                message = message.decode("utf-8")


                if msg_topic=="patric/control":

                    add_phi, add_x = message.split(',')
                    add_phi = float(add_phi)*10
                    add_x = float(add_x)/100
                    print("mqtt input")
            '''
            add_x = 0
            add_phi = 0
            try:
                data=CONN.recv(2**5)
                add_phi, add_x = data.decode('utf-8').split(',')
                print(add_x, phidot)
            except Exception as e:
                pass
            '''
            cmd_dict['target_l'] += add_x
            cmd_dict['phi'] += add_phi


            # This should be handled by PID?
            cmd_dict['phidot'] = (cmd_dict['phi'] - state_dict['phi'][1])
            cmd_dict['target_v'] = (cmd_dict['target_l'] - state_dict['run_l'][1])

        #Debug:
        state_dict['loop_idx'] = i

        # Update i
        i += 1

    print(status)
    disable_all(ser, state_dict)

    return store_arr[3:i], ser, status, cmd_dict, state_dict, kl


def optimize_params():
    """
    Optimizes various parameters using some optimization...
    """
    show = False
    ctrl_params_dict = CTRL_PARAMS_DICT
    def params_to_dict_(params):
        ctrl_params_dict['P_pos'] = params[0]
        ctrl_params_dict['I_pos'] = params[1]
        ctrl_params_dict['D_pos'] = params[2]
        ctrl_params_dict['P_theta'] = params[3]
        ctrl_params_dict['I_theta'] = params[4]
        ctrl_params_dict['D_theta'] = params[5]

        return ctrl_params_dict
    def dict_to_params_():
        return [ctrl_params_dict['P_pos'],
                ctrl_params_dict['I_pos'],
                ctrl_params_dict['D_pos'],
                ctrl_params_dict['P_theta'],
                ctrl_params_dict['I_theta'],
                ctrl_params_dict['D_theta']]


    def opm_callback_(params, convergence=None): #, opm_state):
        np.save('ctrl_params.npy', params_to_dict_(params))
        if MODE.startswith('simul') and show:
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

        if any(np.isnan(params)):
            print('captured param nan')
            return np.inf

        run_array = run_balancing(ser,
                                  ctrl_params_dict=ctrl_params_dict,
                                  run_time_max=OPM_LOOP_TIME)

        score = score_run(run_array)
        print(params, score)
        return score

    init_params = dict_to_params_()
    if OPM_METHOD == 'L-BFGS-B':
        res = minimize(get_balance_score_from_params_, init_params,
                       options={'eps':1.00, 'ftol':1e-18, 'gtol':1e-18},
                       #bounds=bounds,
                       method='L-BFGS-B', callback=opm_callback_)
    elif OPM_METHOD == 'diff_evo':
        bounds = [(0, 10)]*3 + [(0, 100)]*3
        differential_evolution(get_balance_score_from_params_, bounds,
                               strategy='best1bin',
                               maxiter=1000,
                               popsize=15, tol=0.01,
                               mutation=(0.5, 1),
                               recombination=0.5,
                               seed=None,
                               callback=opm_callback_,
                               disp=False,
                               polish=False,
                               init='latinhypercube',
                               atol=0,
                               updating='immediate',
                               workers=1)
    elif OPM_METHOD == 'brute':
        step = 1
        bounds = (slice(0.5, 3.5, .5),
                  slice(0, 4, .5),
                  slice(0.5, 3.5, .5),
                  slice(0, 21, 2))
        res = brute(get_balance_score_from_params_,
                    ranges=bounds, finish=None) #, Ns=20)
        opm_callback_(res)
    return

def run_balancing(ser,
                  ctrl_params_dict=None,
                  run_time_max=10,
                  max_diff_theta=5):
    """
    Starts the balancing whenever robot is
    at UPRIGHT angle.
    """

    print('Waiting for theta to be close to upright.')
    while True:
        if MODE.startswith('simul'):
            theta = UPRIGHT_THETA
        else:
            talk(ser,
                 {'mode':MODE},
                 {'cmd': [0, 0, 0]})
            time.sleep(.01)
            theta, _, _ = listen(ser, mode=MODE)
        if (np.round(time.time(), 1) - int(time.time())) % .5 == 0:
            print('theta = {:.2f}'.format(theta))
        if abs(theta - UPRIGHT_THETA) < max_diff_theta:
            print('Exiting wait: theta={:.2f}'.format(theta))
            break

    print('Run Loop')
    run_data, _, status, _, state_dict, kl = \
            balance_loop(ser,
                         run_time_max=run_time_max,
                         ctrl_params_dict=ctrl_params_dict)

    if (status == 'upright') and (kl is not None):
        kl.store()

    np.save('orient.npy', run_data)
    disable_all(ser, {'mode':MODE})

    #if not MODE.startswith('simul'):
    #    relocate(ser, state_dict['run_l'][0])

    if status != 'fell':
        return run_data
    else:
        return None

def move_distance(ser):

    from params import ARDUINO_STEP_MULTIP, STEPS_PER_REV, DT

    enable_legs(ser, MODE)
    DT = 5. #0

    STEPS_PER_REV = 2400 #2201
    steps_per_sec = STEPS_PER_REV / DT
    vint = int(round(steps_per_sec / ARDUINO_STEP_MULTIP))
    print('vint = ', vint)
    print('steps per sec = ', vint * ARDUINO_STEP_MULTIP)
    print('tot rounds = ', vint*ARDUINO_STEP_MULTIP / STEPS_PER_REV*DT)
    T0 = time.time()
    run_t = 0

    while run_t <= DT:
        talk(ser, {'mode':MODE}, {'cmd':[0, vint, vint]})
        time.sleep(.10)
        listen(ser, mode=MODE)
        run_t = time.time() - T0
    print('run time = ', run_t)
    disable_all(ser, {'mode':MODE})

if __name__ == '__main__':
    if MODE == 'simulate':
        display_simulation(SER, balance_loop)
    elif MODE == 'simuloptimize':
        optimize_params()
    elif MODE == 'optimize':
        optimize_params()
    elif MODE == 'move':
        move_distance(SER)
    else:
        try:
            while True:
                run_balancing(SER,
                              run_time_max=RUN_LOOP_TIME,
                              max_diff_theta=.10) # is not None:
                print('Sleeping')
                time.sleep(1)
        except KeyboardInterrupt:
            print('Disabling')
            disable_all(SER, {'mode':''})

