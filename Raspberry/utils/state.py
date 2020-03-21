import numpy as np

PI = 3.14159267
WHEEL_DIA = .09
STEPS_PER_REV = 400

def wheel_cmd_to_v(a):
    return a/STEPS_PER_REV*WHEEL_DIA*PI
def wheel_v_to_cmd(v):
    return v/(WHEEL_DIA*PI)*STEPS_PER_REV


def check_status(orient_arr):

    if ((orient_arr[0,1]-upright_angle) < -30) or (30 < (orient_arr[0,1]-upright_angle)):
        print('FELL: {}'.format(orient_arr[0,1]-upright_angle))
        print(orient_arr)
        return 'fell'
    else:
        return 'upright'

def update_state(orient_arr, orient, cmd, time_orient):

    #orient = orient/ANGLE_FACTOR/PI*180
    orient_arr[0, :2] = time_orient/1e6, orient

    if cmd[0] == 0:
        orient_arr[0, 8] = wheel_cmd_to_v(cmd[1])
        orient_arr[0, 9] = wheel_cmd_to_v(cmd[2])
    else:
        orient_arr[0, 8] = wheel_cmd_to_v(orient_arr[1, 8])
        orient_arr[0, 9] = wheel_cmd_to_v(orient_arr[1, 9])

    # Figure out the past accel:
    dt = orient_arr[0, 0] - orient_arr[1, 0]
    orient_arr[1, 10] = (orient_arr[1, 8] - orient_arr[0, 8])/dt
    orient_arr[1, 11] = (orient_arr[1, 9] - orient_arr[0, 9])/dt

    return orient_arr

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
