# This is the control packag
import numpy as np
from simple_pid import PID
from params import WHEEL_DIA, UPRIGHT_THETA, \
        ARDUINO_STEP_MULTIP, WHEEL_DIA, RAIL_W, \
        STEPS_PER_REV, PID_P, PID_I, PID_D, PI

CTRL_PID = PID(PID_P, PID_I, PID_D, setpoint=UPRIGHT_THETA)

def v_to_cmd_int(v):
    """
    Get the command integer from velocity.
    """
    rev_per_sec = v / (WHEEL_DIA * PI)
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
    v = cmd_dict['v']
    phidot = cmd_dict['phidot']

    v_l = v - phidot * RAIL_W / 2
    v_r = v + phidot * RAIL_W / 2

    return v_to_cmd_int(v_l), v_to_cmd_int(v_r)

def react(state_dict, cmd_dict):
    """
    React to the current state by updating the cmd_dictionary.
    """
    theta = state_dict['theta'][0]
    #v_now = state_dict['v'][0]


    if np.isnan(theta):
        theta = UPRIGHT_THETA


    v = CTRL_PID(theta)
    #dt = state_dict['time_next'] - state_dict['times'][0]
    #vt = v_now + accel*dt

    cmd_dict['v'] = v
    #phidot = 90/360*2*PI # 10 deg/sec
    v_l, v_r = wheels_v_to_cmds(cmd_dict) #v, phidot)

    #cmd_dict['phidot'] = phidot
    cmd_dict['cmd'] = [0, v_l, v_r]
