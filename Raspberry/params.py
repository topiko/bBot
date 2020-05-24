"""
Various physical parameters
"""
import numpy as np

UPRIGHT_THETA = 15.
STEPS_PER_REV = 400 # This would then be 200*8...
ARDUINO_STEP_MULTIP = 2 # Maybe 3 is better...
WHEEL_DIA = .09
RAIL_W = .078
PI = 3.14159267

DT = .016

AMPLITUDE = .1 #12
INIT_THETA_DEV = 1

# Method for optimizing the ctrl parameters: ('L-BFGS-B', 'brute')
OPM_METHOD = 'diff_evo' #'L-BFGS-B' #'diff_evo' #'L-BFGS-B' #'brute'

# maximum targeted accel:
MAX_V = 0.2
MAX_A = .5
MAX_A_CTRL = 3 #3
MAX_JERK = 20.0 #20

# Dyn params
ALPHA = 350 #736
BETA = -90

ALPHA_SIMUL = ALPHA # - 30
BETA_SIMUL = BETA #- 20

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .02**2 # .015**2
SIGMA_THETADOTDOT = .005 #.005 #.2


SIMUL_LOOP_TIME = 22.5
OPM_LOOP_TIME = 7 #22.5
RUN_LOOP_TIME = 20

try:
    CTRL_PARAMS_DICT = np.load('ctrl_params.npy', allow_pickle=True).item()
except FileNotFoundError:
    CTRL_PARAMS_DICT = {'P_pos':0,
                        'D_pos':0,
                        'I_pos':0.0,
                        'P_theta':2.00,
                        'D_theta':4,
                        'I_theta':0.05, #1.5,
                        'accel_mltp':1.2}

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

