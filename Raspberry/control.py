"""
This is the control packag
"""
import numpy as np
#from simple_pid import PID
from state import update_array
from params import WHEEL_DIA, UPRIGHT_THETA, \
        ARDUINO_STEP_MULTIP, RAIL_W, \
        STEPS_PER_REV, GRAVITY_ACCEL, \
        PI, ALPHA, BETA, MAX_A, MAX_V
from communication import disable_all, enable_legs, talk

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

def relocate(ser, run_l):
    """
    Drive the fallen robot back to 0 pos.
    Assumes there are aids to hold the robot roughly upright.
    """
    v = -.1 * np.sign(run_l)
    vcmd = v_to_cmd_int(v)
    cmd = [0, vcmd, vcmd]
    time = run_l / v
    dt = .1

    enable_legs(ser, 'run')
    for i in range(time//dt):
        talk(ser, None, {'cmd':cmd})
        time.sleep(dt)
    disable_all(ser, {'mode':'run'})

def get_PID(x, Int, xdot, P, I, D):
    """
    Simple PID, Int term needs to be evaulated outside.
    """
    return -P*x - Int*I - xdot*D

def get_xdd_damp_osc(x, xdot, omega, khi):
    """
    Get the sexond time deriv d^2/dt^2 x for
    damped oscillator given damping param khi, undamped freq omega,
    and current displacement x and its first time deriv xdot.
    """
    return -omega**2*x - 2*khi*omega*xdot

def get_target_theta(state_dict, cmd_dict, ctrl_params_dict):
    """
    Updates the target theta based on current velocity.
    """

    #damping_ratio = ctrl_params_dict['damp_pos']
    #omega = ctrl_params_dict['omega_pos']

    delta_l = state_dict['run_l'][0] - cmd_dict['target_l']
    delta_l_dot = state_dict['v'][0] - cmd_dict['target_v']

    if np.sign(delta_l) != np.sign(state_dict['I_pos']):
        state_dict['I_pos'] += delta_l * state_dict['dt']
    else:
        state_dict['I_pos'] = delta_l * state_dict['dt']

    target_a = get_PID(delta_l,
                       state_dict['I_pos'],
                       delta_l_dot,
                       ctrl_params_dict['P_pos'],
                       ctrl_params_dict['I_pos'],
                       ctrl_params_dict['D_pos'])

    target_a = np.clip(target_a, -MAX_A, MAX_A)
    if ((state_dict['v'][0] < -MAX_V) & (delta_l > MAX_A / 2 * 2**2)) or \
       ((state_dict['v'][0] > MAX_V) & (delta_l < - MAX_A / 2 * 2**2)):
        target_a = 0

    cmd_dict['target_a'] = target_a

    tilt_theta = np.arctan(target_a / GRAVITY_ACCEL) / PI * 180

    return UPRIGHT_THETA + tilt_theta

def deg_to_rad(degs):
    return degs / 180 * PI

def get_a_03(state_dict, cmd_dict, ctrl_params_dict):
    """
    Get the reaction a for given state.
    This is based in driving along: Delta theta = exp^(-kappa/t)
    --> thetadot_target = d Delta theta / dt  =  -kappa * Delta theta
    """

    theta = state_dict['theta_predict']
    thetadot = state_dict['thetadot_predict']
    target_theta = cmd_dict['target_theta']

    delta_theta = theta - target_theta
    delta_thetadot = thetadot

    #if np.sign(delta_theta) != np.sign(state_dict['I_theta']):
    #    state_dict['I_theta'] += delta_theta * state_dict['dt']
    #else:
    #    state_dict['I_theta'] = delta_theta * state_dict['dt']
    state_dict['I_theta'] += delta_theta * state_dict['dt']

    target_thetadotdot = get_PID(delta_theta,
                                 state_dict['I_theta'],
                                 delta_thetadot,
                                 ctrl_params_dict['P_theta'],
                                 ctrl_params_dict['I_theta'],
                                 ctrl_params_dict['D_theta'])

    state_dict['target_thetadotdot'] = \
            update_array(state_dict['target_thetadotdot'],
                         target_thetadotdot)
    accel = (ALPHA*GRAVITY_ACCEL*np.sin(deg_to_rad(theta - UPRIGHT_THETA)) \
             - target_thetadotdot) \
            / (ALPHA*np.cos(deg_to_rad(theta - UPRIGHT_THETA))) # - BETA)

    return np.clip(accel, -10*MAX_A, 10*MAX_A)


def react(state_dict, cmd_dict, ctrl_params_dict):
    """
    React to the current state by updating the cmd_dictionary.
    """
    v_now = state_dict['v'][0]
    #cmd_dict['target_v'] = get_target_v(state_dict, cmd_dict, ctrl_params_dict)
    cmd_dict['target_theta'] = get_target_theta(state_dict, cmd_dict, ctrl_params_dict)

    state_dict['target_theta'] = update_array(state_dict['target_theta'], cmd_dict['target_theta'])
    state_dict['target_x'] = update_array(state_dict['target_x'], cmd_dict['target_x'])
    state_dict['target_l'] = update_array(state_dict['target_l'], cmd_dict['target_l'])
    state_dict['target_y'] = update_array(state_dict['target_y'], cmd_dict['target_y'])
    state_dict['target_v'] = update_array(state_dict['target_v'], cmd_dict['target_v'])
    state_dict['target_a'] = update_array(state_dict['target_a'], cmd_dict['target_a'])
    #  accel = get_a_01(state_dict, cmd_dict)
    accel = get_a_03(state_dict, cmd_dict, ctrl_params_dict)

    cmd_dict['v'] = v_now + accel*state_dict['dt'] #delta_t
    cmd_dict['a'] = accel
    state_dict['a'] = update_array(state_dict['a'], accel)

    v_l, v_r = wheels_v_to_cmds(cmd_dict) #, state_dict) #v, phidot)
    cmd_dict['cmd'] = [0, v_l, v_r]

