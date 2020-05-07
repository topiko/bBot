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

AMPLITUDE = .12
#TILT_MLTP = 10.
#A_MLTP1 = 0.2 # 00 #.2
#A_MLTP2 = 0.010 #30.5

# Method for optimizing the ctrl parameters: ('L-BFGS-B', 'brute')
OPM_METHOD = 'diff_evo' #'L-BFGS-B' #'brute'

# maximum targeted accel:
MAX_A = .5

# Dyn params
ALPHA = 450 #400 #736
BETA = -90

ALPHA_SIMUL = ALPHA # - 30
BETA_SIMUL = BETA #- 20

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .02**2 # .015**2
SIGMA_THETADOTDOT = .2


SIMUL_LOOP_TIME = 22.5
OPM_LOOP_TIME = 7 #22.5
RUN_LOOP_TIME = 70
try:
    CTRL_PARAMS_DICT = np.load('ctrl_params.npy', allow_pickle=True).item()
except FileNotFoundError:
    CTRL_PARAMS_DICT = {'P_pos':6,
                        'D_pos':5,
                        'I_pos':3,
                        'P_theta':30,
                        'D_theta':20,
                        'I_theta':2}

#                        'damp_theta':1.0,
#                        'omega_theta':1/(.015*2*np.pi)}
