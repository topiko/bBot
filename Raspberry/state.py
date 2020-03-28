import numpy as np
from control import UPRIGHT_THETA

PI = 3.14159267
WHEEL_DIA = .09
STEPS_PER_REV = 400



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

def update_state(state_dict, theta, cmd_dict, cur_time, t_add):

    times = state_dict['times']
    state_dict['times'] = update_array(times, cur_time)
    state_dict['theta'] = update_array(state_dict['theta'], theta)
    state_dict['time_next'] = cur_time + t_add

    if cmd_dict['cmd_to'] == 'wheels':
        for key in ['v', 'phidot']:
            state_dict[key][0] = cmd_dict[key]


    # update the orientation array:
    state_dict['thetadot'][2] = state_dict['thetadot'][1]
    state_dict['thetadotdot'][2] = state_dict['thetadotdot'][1]

    a, b, c = fit_parabel(times, state_dict['theta'])
    state_dict['thetadot'][1] = 2*a*times[1] + b
    state_dict['thetadotdot'][1] = 2*a

    # wheel v and a
    a, b, c = fit_parabel(times, state_dict['v'])
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


