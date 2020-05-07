import numpy as np


def get_x_v_a(time, amp, state_dict):
    """
    Define position and vel for patric at given time. Currently simple
    Sin step system.
    """
    sin_period = 6
    hold_period = 4.0
    full_period = sin_period + hold_period * 2

    time = time % full_period
    if (time < sin_period / 2) or \
       ((sin_period / 2 + hold_period < time) \
       and (time < hold_period + sin_period)):
        time = time - hold_period if time > sin_period else time
        xpos = amp * np.cos(time / sin_period * np.pi * 2)
        vval = -amp * np.sin(time / sin_period * np.pi * 2) * 2 * np.pi / sin_period
        aval = amp * np.cos(time / sin_period * np.pi * 2) * (2 * np.pi / sin_period)**2
    elif (sin_period / 2 < time) and (time < hold_period + sin_period / 2):
        xpos = -amp
        vval = 0
        aval = 0
    else:
        xpos = amp
        vval = 0
        aval = 0
    return xpos, vval, aval

def get_x_v_a_2(time, amp, state_dict):
    """
    Define position and vel for patric at given time. Currently simple
    Sin step system.
    """
    sin_period = 4
    hold_period = 4
    full_period = sin_period/2 + hold_period

    time = time % full_period
    if (time < sin_period / 2):
        time = time - hold_period if time > sin_period else time
        xpos = amp * np.cos(time / sin_period * np.pi * 2 + np.pi)
        vval = -amp * np.sin(time / sin_period * np.pi * 2 + np.pi) * 2 * np.pi / sin_period
        aval = amp * np.cos(time / sin_period * np.pi * 2 + np.pi) * (2 * np.pi / sin_period)**2
    elif (sin_period / 2 < time): # and (time < hold_period + sin_period / 2):
        xpos = amp
        vval = 0
        aval = 0
#    else:
#        xpos = amp
#        vval = 0
#        aval = 0
    return state_dict['target_l'][0] + xpos, vval, aval
