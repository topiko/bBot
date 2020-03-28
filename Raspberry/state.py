import numpy as np
from params import UPRIGHT_THETA




def check_status(state_dict):

    theta = state_dict['theta'][0]

    if ((theta - UPRIGHT_THETA) < -30) or (30 < (theta-UPRIGHT_THETA)):
        print('FELL: {}'.format(theta-UPRIGHT_THETA))
        print(state_dict)
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

def update_location(state_dict):
    """
    Update the location according to phidot and v.
    """

    dt = state_dict['times'][1] - state_dict['times'][0]
    dphi = state_dict['phidot'][1]*dt
    dl = state_dict['v'][1]*dt
    phi = state_dict['phi'][1]

    # update the turn angle:
    phi_now = phi + dphi
    state_dict['phi'] = update_array(state_dict['phi'], phi_now)

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

def update_state(state_dict, theta, cmd_dict, cur_time, t_add):

    times = state_dict['times']
    state_dict['times'] = update_array(times, cur_time)
    state_dict['theta'] = update_array(state_dict['theta'], theta)
    state_dict['time_next'] = cur_time + t_add

    for key in ['v', 'phidot']:
        # The command is one time step ahead
        state_dict[key][1] = cmd_dict[key]


    # update the orientation array:
    state_dict['thetadot'][2] = state_dict['thetadot'][1]
    state_dict['thetadotdot'][2] = state_dict['thetadotdot'][1]

    a, b, _ = fit_parabel(times, state_dict['theta'])
    state_dict['thetadot'][1] = 2*a*times[1] + b
    state_dict['thetadotdot'][1] = 2*a

    # wheel v and a
    a, _, _ = fit_parabel(times, state_dict['v'])
    state_dict['a'][1] = 2*a


def fit_parabel(times, points):

    if (np.diff(times) == 0).all():
        return 0, 0, 0

    p1, p2, p3 = points
    t1, t2, t3 = times

    a = (p1*(t2 - t3) - p2*(t1 - t3) + p3*(t1 - t2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
    b = (-p1*(t2**2 - t3**2) + p2*(t1**2 - t3**2) - p3*(t1**2 - t2**2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
    c = (p1*t2*t3*(t2 - t3) - p2*t1*t3*(t1 - t3) + p3*t1*t2*(t1 - t2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)

    return a, b, c

def predict_theta(state_dict):

    times = state_dict['times']
    thetas = state_dict['theta']

    t_future = state_dict['time_next'] - times[0]
    a, b, c = fit_parabel(times - times[0], thetas)

    theta_predict =  a*t_future**2 + b*t_future + c
    state_dict['theta_predict'] = theta_predict
    #state_dict['theta_predict_hist'] = \
    #        update_array(state_dict['theta_predict_hist'], theta_predict)


