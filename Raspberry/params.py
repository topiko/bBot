"""
Various physical parameters
"""
import numpy as np

UPRIGHT_THETA = 15.2
STEPS_PER_REV = 400
ARDUINO_STEP_MULTIP = 2
WHEEL_DIA = .09
RAIL_W = .078
PI = 3.14159267

DT = .016

#TILT_MLTP = 10.
#A_MLTP1 = 0.2 # 00 #.2
#A_MLTP2 = 0.010 #30.5

# Dyn params
ALPHA = 560 #400 #736
BETA = -160

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .02**2 # .015**2
SIGMA_THETADOTDOT = .2


SIMUL_LOOP_TIME = 20
OPM_LOOP_TIME = 5
try:
    CTRL_PARAMS_DICT = np.load('ctrl_params.npy', allow_pickle=True).item()
except FileNotFoundError:
    CTRL_PARAMS_DICT = {'kappa_v':2, # 11.2 #5 D x * kappa = v_target
                        'kappa_v2':.250, # 0.9 #2
                        #'kappa_tilt_theta':.2*GRAVITY_ACCEL, #1000, # D v * kappa = theta tilt
                        'kappa_theta':4, # D theta = exp(-kappa * t)
                        'gamma_theta':2000} # D thetadot * gamma = thetadotdot
