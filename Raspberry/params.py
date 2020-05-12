"""
Various physical parameters
"""
import numpy as np

UPRIGHT_THETA = 15.
STEPS_PER_REV = 400
ARDUINO_STEP_MULTIP = 2
WHEEL_DIA = .09
RAIL_W = .078
PI = 3.14159267

DT = .016

AMPLITUDE = .0 #12
INIT_THETA_DEV = 0

# Method for optimizing the ctrl parameters: ('L-BFGS-B', 'brute')
OPM_METHOD = 'L-BFGS-B' #'diff_evo' #'L-BFGS-B' #'brute'

# maximum targeted accel:
MAX_V = 1.0
MAX_A = 1.0
MAX_JERK = 4.0

# Dyn params
ALPHA = 270 #400 #736
BETA = -90

ALPHA_SIMUL = ALPHA # - 30
BETA_SIMUL = BETA #- 20

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .02**2 # .015**2
SIGMA_THETADOTDOT = .2


SIMUL_LOOP_TIME = 22.5
OPM_LOOP_TIME = 7 #22.5
RUN_LOOP_TIME = 20
try:
    CTRL_PARAMS_DICT = np.load('ctrl_params.npy', allow_pickle=True).item()
except FileNotFoundError:
    CTRL_PARAMS_DICT = {'P_pos':4,
                        'D_pos':3,
                        'I_pos':0.5,
                        'P_theta':3,
                        'D_theta':5,
                        'I_theta':1.5,
                        'accel_mltp':4}

#                        'damp_theta':1.0,
#                        'omega_theta':1/(.015*2*np.pi)}
COLLECT_DATA = ['times', 'theta',
                'thetadot', 'thetadotdot',
                'theta_measured', 'thetadot_measured',
                'thetadotdot_measured',
                'target_theta', 'target_thetadot',
                'target_thetadotdot',
                'target_x', 'target_y', 'target_v',
                'target_l',
                'run_l',
                'v', 'a', 'x', 'y', 'phi',
                'target_a'] #, 'theta_predict']

