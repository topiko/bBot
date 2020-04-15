# This is the control packag
import numpy as np
#from simple_pid import PID
from state import update_array
from params import WHEEL_DIA, UPRIGHT_THETA, \
        ARDUINO_STEP_MULTIP, RAIL_W, \
        STEPS_PER_REV, \
        TILT_MLTP, A_MLTP1, A_MLTP2, PI

#CTRL_PID = PID(PID_P, PID_I, PID_D, setpoint=UPRIGHT_THETA)

def v_to_cmd_int(vel):
    """
    Get the command integer from velocity.
    """
    rev_per_sec = vel / (WHEEL_DIA * PI)
    steps_per_sec = STEPS_PER_REV * rev_per_sec
    cmd = steps_per_sec / ARDUINO_STEP_MULTIP

    if cmd > 1023:
        cmd = 1023
    elif cmd < -1024:
        cmd = -1024
    return int(cmd)

def wheels_v_to_cmds(cmd_dict):
    """
    Get the individual wheel cmds from desired v and phidot.
    """
    vel = cmd_dict['v']
    phidot = cmd_dict['phidot']

    v_l = vel - phidot * RAIL_W / 2
    v_r = vel + phidot * RAIL_W / 2

    return v_to_cmd_int(v_l), v_to_cmd_int(v_r)

def get_target_theta(state_dict):
    """
    Updates the target theta based on current velocity or somethign..
    """
    tilt_theta = state_dict['v'][0]*TILT_MLTP
    return UPRIGHT_THETA - tilt_theta

def react(state_dict, cmd_dict):
    """
    React to the current state by updating the cmd_dictionary.
    """
    theta = state_dict['theta'][0]
    v_now = state_dict['v'][0]
    cmd_dict['target_theta'] = get_target_theta(state_dict)
    if np.isnan(theta):
        print('Warning: nan-theta')
        theta = UPRIGHT_THETA

    # TODO: FIX better function for a
    delta_theta = theta - cmd_dict['target_theta']
    accel = delta_theta * A_MLTP1 + state_dict['thetadot'][0] * A_MLTP2

    #delta_t = state_dict['time_next'] - state_dict['times'][0]

    cmd_dict['v'] = v_now + accel*state_dict['dt'] #delta_t
    cmd_dict['a'] = accel
    state_dict['a'] = update_array(state_dict['a'], accel)

    v_l, v_r = wheels_v_to_cmds(cmd_dict) #v, phidot)
    cmd_dict['cmd'] = [0, v_l, v_r]
