"""Various physical parameters"""


UPRIGHT_THETA = 15.
STEPS_PER_REV = 400
ARDUINO_STEP_MULTIP = 2
WHEEL_DIA = .09
RAIL_W = .078
PI = 3.14159267

#PID:
PID_P = -.05 #-.08
PID_I = -0.20 #-.20
PID_D = -0.0001


TILT_MLTP = .02
A_MLTP1 = .2
A_MLTP2 = 0 #30.5


# Dyn params 
ALPHA = 736 #-225

GRAVITY_ACCEL = 9.81

# Uncertainty in theta measurement
SIGMA_THETA = .05**2
