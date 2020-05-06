"""
This is the control packag
"""
import numpy as np
#from simple_pid import PID
from state import update_array
from params import WHEEL_DIA, UPRIGHT_THETA, \
        ARDUINO_STEP_MULTIP, RAIL_W, \
        STEPS_PER_REV, GRAVITY_ACCEL, \
        PI, ALPHA, BETA


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

def get_target_v(state_dict, cmd_dict, ctrl_params_dict):
    """
    Get the targeted velocity. This will depend where Patric is and
    where it should be. 1D for now...
    """
    kappa = ctrl_params_dict['kappa_v']
    delta_x = state_dict['x'][0] - cmd_dict['target_x']
    target_v = - delta_x * kappa

    return target_v

def get_target_theta(state_dict, cmd_dict, ctrl_params_dict):
    """
    Updates the target theta based on current velocity.
    If velocity is too low, we tilt forward from the steady pos.
    Delta v = v - target_v
    Again Delta v = exp(-kappa_til_theta*t) --> d (Delta v) / dt = -kappa_... * Delta v
    => a = -kappa... * Delta v
    a should be from model a = kappa**2 * Delta x
    Thetadotdot == 0, see modelpatric
    a = GRAVITY_ACCEL*tan(theta_target - UPRIGHT_THETA)
    => arctan(a/GRAVITY_ACCEL) = theta_target - UPRIGHT_THETA
    => arctan(-kappa... * Delta v / GRAVITY_ACCEL) =: tilt_theta
    """
    #kappa_tilt_theta = ctrl_params_dict['kappa_tilt_theta']
    kappa_v = ctrl_params_dict['kappa_v']
    #kappa_v2 = ctrl_params_dict['kappa_v2']
    delta_x = state_dict['run_l'][0] - cmd_dict['target_l']

    target_v = cmd_dict['target_v']
    target_a = cmd_dict['target_a']
    #target_a = cmd_dict['target_a']
    #dt = state_dict['dt']

    x_correction_v = - kappa_v * delta_x
    target_v_mod = target_v + x_correction_v

    cmd_dict['target_v'] = target_v_mod
    #TODO: Thinkf about why would one have the kappas like this?
    delta_v = state_dict['v'][0] - target_v_mod

    update_v_a = target_a - kappa_v * delta_v
    #update_v_a = kappa_v**2 * delta_x - kappa_v * delta_v + target_a
    #a_model = kappa_v**2 * delta_x

    target_a = update_v_a


    # use_a = - (kappa_v * delta_x + kappa_v2 * delta_v) + target_a

    # tilt_theta1 = np.arctan(use_a / GRAVITY_ACCEL)
    #thetadotdot_rad = state_dict['thetadotdot'][0] / 180 * np.pi

    theta = deg_to_rad(state_dict['theta_predict']) #[0])

    #print(theta, use_a)

    #denom = GRAVITY_ACCEL*ALPHA
    #nom = -BETA*target_a + ALPHA*target_a*np.cos(theta)
    #nom = np.clip(nom, -denom/2, denom/2)
    #tilt_theta = np.arcsin(nom / denom)

    cmd_dict['target_a'] = target_a

    kappa_tilt = ctrl_params_dict['a_to_tilt_mltp']

    tilt_theta = target_a * kappa_tilt
    #tilt_theta = np.arctan(use_a / GRAVITY_ACCEL)

    return UPRIGHT_THETA + tilt_theta #/PI*180


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
            / (ALPHA*np.cos(deg_to_rad(theta - UPRIGHT_THETA)) - BETA)

    return accel

def get_a_02(state_dict, cmd_dict, ctrl_params_dict):
    """
    Get the reaction a for given state.
    This is based in driving along: Delta theta = exp^(-kappa/t)
    --> thetadot_target = d Delta theta / dt  =  -kappa * Delta theta
    """
    kappa = ctrl_params_dict['kappa_theta'] #4.0
    gamma = ctrl_params_dict['gamma_theta'] #1000

    theta = state_dict['theta_predict']
    thetadot = state_dict['thetadot_predict']
    target_theta = cmd_dict['target_theta']

    delta_theta = theta - target_theta
    thetadot_target = - kappa * delta_theta #* np.sign(delta_theta)

    state_dict['target_thetadot'] = update_array(state_dict['target_thetadot'],
                                                 thetadot_target)
    # How much we are off drom the desired thetadot
    delta_thetadot = thetadot_target - thetadot

    #thetadotdot_target = - kappa * delta_thetadot

    # We want to update thetadot so that delta_thetadot gets smaller.
    # Thetadotdot = gamma * delta_thetadot = thetadotdot(theta, a) (get_model_patric)
    accel = (ALPHA*GRAVITY_ACCEL*np.sin(deg_to_rad(theta - UPRIGHT_THETA)) \
             - deg_to_rad(delta_thetadot)*gamma) \
            / (ALPHA*np.cos(deg_to_rad(theta - UPRIGHT_THETA)) - BETA)

    return accel

def get_a_03(state_dict, cmd_dict, ctrl_params_dict):
    """
    Get the reaction a for given state.
    This is based in driving along: Delta theta = exp^(-kappa/t)
    --> thetadot_target = d Delta theta / dt  =  -kappa * Delta theta
    """
    kappa = ctrl_params_dict['kappa_theta'] #4.0
    #gamma = ctrl_params_dict['gamma_theta'] #1000'
    #kappa = 10

    theta = state_dict['theta_predict']
    thetadot = state_dict['thetadot_predict']
    target_theta = cmd_dict['target_theta']

    delta_theta = theta - target_theta
    thetadot_target = - kappa * delta_theta #* np.sign(delta_theta)

    state_dict['target_thetadot'] = update_array(state_dict['target_thetadot'],
                                                 thetadot_target)
    # How much we are off drom the desired thetadot
    delta_thetadot = thetadot - thetadot_target

    thetadotdot_target = - kappa * delta_thetadot

    # We want to update thetadot so that delta_thetadot gets smaller.
    # Thetadotdot = gamma * delta_thetadot = thetadotdot(theta, a) (get_model_patric)
    accel = (ALPHA*GRAVITY_ACCEL*np.sin(deg_to_rad(theta - UPRIGHT_THETA)) \
             - thetadotdot_target) \
            / (ALPHA*np.cos(deg_to_rad(theta - UPRIGHT_THETA)) - BETA)

    return accel


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

    v_l, v_r = wheels_v_to_cmds(cmd_dict) #v, phidot)
    cmd_dict['cmd'] = [0, v_l, v_r]

