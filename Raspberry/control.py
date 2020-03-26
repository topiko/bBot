# This is the control packag
import numpy as np
from simple_pid import PID

UPRIGHT_THETA = 15

CTRL_PID = PID(25, 200, -0.1, setpoint=UPRIGHT_THETA)



def act(state_dict):

    theta = state_dict['theta'][0]
    target_theta = state_dict['target_theta']

    if np.isnan(theta):
        theta = UPRIGHT_THETA


    cmd = int(CTRL_PID(theta))

    if 1023 < cmd:
        cmd = 1023
    elif cmd < -1024:
        cmd = -1024


    return [0, cmd, cmd]

