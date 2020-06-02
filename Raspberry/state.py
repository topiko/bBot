import numpy as np
from params import UPRIGHT_THETA
from modelpatric import get_thetadotdot



def check_status(state_dict):

    theta = state_dict['theta_measured'][0]

    if ((theta - UPRIGHT_THETA) < -30) or (30 < (theta-UPRIGHT_THETA)):
        print('FELL: {}'.format(theta-UPRIGHT_THETA))
        for key, val in state_dict.items():
            print(key, val)
        return 'fell'
    else:
        return 'upright'

def update_array(arr, latest_value):

    arr[1:] = arr[:-1]
    arr[0] = latest_value
    return arr

def update_cmd_vars(state_dict, cmd_dict):
    """
    These need t be updated before new command is calculated.
    """
    state_dict['v'] = update_array(state_dict['v'], cmd_dict['v'])
    state_dict['phidot'] = update_array(state_dict['phidot'], cmd_dict['phidot'])

def reset_location(state_dict):

    for key in ['x', 'y', 'phi']:
        state_dict[key] = np.zeros(3)


def update_location(state_dict):
    """
    Update the location according to phidot and v.
    """

    dt = state_dict['times'][0] - state_dict['times'][1]
    dphi = state_dict['phidot'][1] * dt
    dl = state_dict['v'][1] * dt
    phi = state_dict['phi'][0]

    # update the turn angle:
    phi_now = phi + dphi
    state_dict['phi'] = update_array(state_dict['phi'], phi_now)

    # Update run length
    l_now = state_dict['run_l'][0] + dl
    state_dict['run_l'] = update_array(state_dict['run_l'], l_now)

    # Update abs run l
    abs_l_now = state_dict['abs_run_l'][0] + abs(dl)
    state_dict['abs_run_l'] = update_array(state_dict['abs_run_l'], abs_l_now)

    # Radius is always positive or None for straight
    turn_radius = np.abs(dl/dphi) if dphi != 0 else None

    # in coordinate system prime, where x is aligned with
    # current heading phi, we have following:
    if turn_radius is None:
        dx_p, dy_p = dl, 0
    else:
        # Note that phidot is defined according to right hand
        # rule where the face of robot is the fingers.
        dx_p = turn_radius * np.sin(dphi) * np.sign(dl)
        dy_p = turn_radius * (1 - np.cos(dphi)) * np.sign(dphi)

    # in primed coords after the turn we have moved:
    dr_p = np.array([dx_p, dy_p]).T

    # the heading is given by the phi - we still need to transform
    # to fixed coord system:
    rotation_matrix = np.array([[np.cos(phi), -np.sin(phi)],
                                [np.sin(phi),  np.cos(phi)]])

    dr = rotation_matrix.dot(dr_p).T
    x = state_dict['x'][0] + dr[0]
    y = state_dict['y'][0] + dr[1]

    state_dict['x'] = update_array(state_dict['x'], x)
    state_dict['y'] = update_array(state_dict['y'], y)


def get_second_deriv(times, values):

    dt1 = times[0] - times[1]
    dt2 = times[1] - times[2]

    return ((values[0] - values[1])/dt1 \
            - (values[1] - values[2])/dt2) / (dt1/2 + dt2/2)

def update_state(state_dict, kl, theta, cur_time):

    times = state_dict['times']
    state_dict['times'] = update_array(times, cur_time)
    state_dict['dt'] = np.diff(times[::-1]).mean()

    # Update measurements
    #===========================================
    state_dict['theta_measured'] = update_array(state_dict['theta_measured'],
                                                theta)
    thetadot_measured = \
            (state_dict['theta_measured'][0] - state_dict['theta_measured'][1]) / \
            (state_dict['times'][0] - state_dict['times'][1])
    state_dict['thetadot_measured'] = \
            update_array(state_dict['thetadot_measured'],
                         thetadot_measured)
    thetadotdot_measured = \
            get_second_deriv(state_dict['times'], state_dict['theta_measured'])
    state_dict['thetadotdot_measured'] = \
            update_array(state_dict['thetadotdot_measured'], thetadotdot_measured)
    state_dict['thetadotdot_measured'][1:] = \
            state_dict['thetadotdot_measured'][:-1]
    #===========================================

    thetadot = (theta - state_dict['theta'][0]) \
        /(state_dict['times'][0] - state_dict['times'][1])

    # Use the klaman to estiamte theta
    if kl is None:
        theta = state_dict['theta_measured'][0]
        thetadot = thetadot_measured
    else:
        theta, thetadot = kl.update(np.array([theta, thetadot]))

    # Update the theta array
    state_dict['theta'] = update_array(state_dict['theta'], theta)
    state_dict['thetadot'] = update_array(state_dict['thetadot'], thetadot)
    state_dict['time_next'] = cur_time + state_dict['dt']

    #for key in ['v', 'phidot']:
    #    # The command is one time step ahead
    #    state_dict[key][1] = cmd_dict[key]


    # update the orientation array:
    #state_dict['thetadot'][2] = state_dict['thetadot'][1]
    state_dict['thetadotdot'][2] = state_dict['thetadotdot'][1]
    state_dict['thetadotdot'][1] = get_second_deriv(state_dict['times'],
                                                    state_dict['theta'])

    update_location(state_dict)

    run_l_arr = state_dict['run_l']
    state_dict['a'][1] = get_second_deriv(times, run_l_arr)

def predict_theta(state_dict, cmd_dict, kl):
    """
    Predict theta using the kalman filter.
    """
    theta = state_dict['theta'][0]
    accel = cmd_dict['a']
    thetadotdot = get_thetadotdot(theta, accel)

    if kl is None:
        state_dict['theta_predict'] = theta
        state_dict['thetadot_predict'] = state_dict['thetadot'][0]
    else:
        state_dict['theta_predict'], state_dict['thetadot_predict'] \
                = kl.predict(control_input=thetadotdot)

