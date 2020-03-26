import numpy as np
from control import UPRIGHT_THETA

PI = 3.14159267
WHEEL_DIA = .09
STEPS_PER_REV = 400



def check_status(orient_arr):

    if ((orient_arr[0,1] - UPRIGHT_THETA) < -30) or (30 < (orient_arr[0,1]-UPRIGHT_THETA)):
        print('FELL: {}'.format(orient_arr[0,1]-UPRIGHT_THETA))
        print(orient_arr)
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
    state_dict['next_time'] = cur_time + t_add

    if cmd_dict['cmd_to'] == 'wheels':
        for key in ['v', 'phidot']:
            state_dict[key][0] = cmd_dict[key]


    # update the orientation array:
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
    thetas = state_dict['thetas']

    t_future = state_dict['next_time'] - times[0]
    a, b, c = fit_parabel(times - times[0], thetas)

    theta_predict =  a*t_future**2 + b*t_future + c
    state_dict['theta_predict'] = theta_predict
    state_dict['theta_predict_hist'] = \
            update_array(state_dict['theta_predict_hist'], theta_predict)


def predict_theta(orient_arr, time_orient):

    # fit 2 order poly to 3 values in orient arr
    # --> w, dw/dt, d**2w/dt**2
    # use these to predict values at time_orient

    t_now = orient_arr[0, 0]
    p1, p2, p3 = orient_arr[:, 1]
    t1, t2, t3 = orient_arr[:, 0] - t_now #orient_arr[0,0]

    if (np.diff(orient_arr[:, 0]) == 0).any():
        print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

    a = (p1*(t2 - t3) - p2*(t1 - t3) + p3*(t1 - t2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
    b = (-p1*(t2**2 - t3**2) + p2*(t1**2 - t3**2) - p3*(t1**2 - t2**2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
    c = (p1*t2*t3*(t2 - t3) - p2*t1*t3*(t1 - t3) + p3*t1*t2*(t1 - t2)) \
        /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)

    p_now = a*time_orient**2 + b*time_orient + c
    w_now = 2*a*time_orient + b
    wdot_now = 2*a

    w_last = 2*a*t1 + b
    wdot_last = 2*a

    orient_arr[1:, :] = orient_arr[:-1, :]
    orient_arr[1, 2:4] = w_last, wdot_last

    orient_arr[0, :4] = t_now + time_orient, p_now, w_now, wdot_now
    orient_arr[0, 4:8] = t_now + time_orient, p_now, w_now, wdot_now

    return orient_arr
