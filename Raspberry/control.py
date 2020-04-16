# This is the control packag
import numpy as np
#from simple_pid import PID
from state import update_array
from params import WHEEL_DIA, UPRIGHT_THETA, \
        ARDUINO_STEP_MULTIP, RAIL_W, \
        STEPS_PER_REV, GRAVITY_ACCEL, \
        TILT_MLTP, A_MLTP1, A_MLTP2, PI, ALPHA, \
        UPRIGHT_THETA

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

def deg_to_rad(degs):
    return degs/180*np.pi

def get_a_01(state_dict, cmd_dict):
    """
    Get the reaction a for given state.
    This is based in driving along: Delta theta = kappa/t
    --> thetadot_target = d Delta theta / dt  = -kappa/t**2 = -1/kappa * (Delta theta)**2
    Remeber though to take the sign into account.
    """
    kappa = .5
    gamma = 2000

    theta = state_dict['theta_predict']
    thetadot = state_dict['thetadot_predict']
    target_theta = cmd_dict['target_theta']

    delta_theta = theta - target_theta
    thetadot_target = - 1 / kappa * delta_theta**2 * np.sign(delta_theta)

    # How much we are off drom the desired thetadot
    delta_thetadot = thetadot_target - thetadot

    # We want to update thetadot so that delta_thetadot gets smaller.
    # Thetadotdot = gamma * delta_thetadot = thetadotdot(theta, a) (get_model_patric)
    accel = (ALPHA*GRAVITY_ACCEL*np.sin(deg_to_rad(theta -UPRIGHT_THETA)) \
             - deg_to_rad(delta_thetadot)*gamma) \
            / (ALPHA*np.cos(deg_to_rad(theta - UPRIGHT_THETA)))

    return accel

def react(state_dict, cmd_dict):
    """
    React to the current state by updating the cmd_dictionary.
    """
    theta = state_dict['theta'][0]
    v_now = state_dict['v'][0]
    cmd_dict['target_theta'] = get_target_theta(state_dict)
    #if np.isnan(theta):
    #    print('Warning: nan-theta')
    #    theta = UPRIGHT_THETA

    # TODO: FIX better function for a
    accel = get_a_01(state_dict, cmd_dict)

    #delta_theta = theta - cmd_dict['target_theta']
    #accel = delta_theta * A_MLTP1 + state_dict['thetadot'][0] * A_MLTP2

    #delta_t = state_dict['time_next'] - state_dict['times'][0]

    cmd_dict['v'] = v_now + accel*state_dict['dt'] #delta_t
    cmd_dict['a'] = accel
    state_dict['a'] = update_array(state_dict['a'], accel)

    v_l, v_r = wheels_v_to_cmds(cmd_dict) #v, phidot)
    cmd_dict['cmd'] = [0, v_l, v_r]
