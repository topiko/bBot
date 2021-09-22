"""
Various physical parameters
"""
import numpy as np

UPRIGHT_THETA = 15.5 #15.2 Usuakky
STEPS_PER_REV = 1600 #1600 # used to be 400 This would then be 200*8...
ARDUINO_STEP_MULTIP = 8 #
WHEEL_DIA = .092
RAIL_W = .078
PI = 3.14159267

DT = .016

AMPLITUDE = .00 ##12 #12
INIT_THETA_DEV = 1
PHIDOT = 90

# Method for optimizing the ctrl parameters: ('L-BFGS-B', 'brute')
OPM_METHOD = 'diff_evo' #'L-BFGS-B' #'diff_evo' #'L-BFGS-B' #'brute'

# maximum targeted accel:
MAX_V = 1.0
MAX_A = 2.0
MAX_A_CTRL = 30
MAX_V_CTRL = (1024 * ARDUINO_STEP_MULTIP / STEPS_PER_REV) * PI*WHEEL_DIA

MAX_JERK = 500 #20

# Dyn params
LCM = 0.05 # Distance of center of mass from wheel axle
MASS = 1.0 # mass of the robot
J = 0.14**2 #MASS * LCM**2  # moment of inertia

ALPHA = 4.7 # (MASS * LCM / J) #40 #736
print('ALPHA = {:.2f}'.format(ALPHA))

ALPHA_SIMUL = ALPHA # - 30

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .0015**2 # .015**2
SIGMA_THETADOTDOT = .002 # 0.15 is a good value
KL_USE_INPUT = True #False #True #False

SIMUL_LOOP_TIME = 22.5
OPM_LOOP_TIME = 7 #22.5
RUN_LOOP_TIME = 120

KAPPA_D_THETA = None #1/2
REMOTE_DX = 0.005
try:
    CTRL_PARAMS_DICT = np.load('ctrl_params.npy', allow_pickle=True).item()
except FileNotFoundError:
    CTRL_PARAMS_DICT = {'P_pos':4,
                        'D_pos':2,
                        'I_pos':.0,
                        'P_theta':350, #150
                        'D_theta':55, # 20
                        'I_theta':0, #1.5,
                        'accel_mltp':1.0}

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
                'v', 'a', 'x', 'y', 'phi', 'a1', 'a2',
                'target_a'] #, 'theta_predict']

