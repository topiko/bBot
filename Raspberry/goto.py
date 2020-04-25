import numpy as np


def get_x_v_a(time, amp):
    """
    Define position and vel for patric at given time. Currently simple
    Sin step system.
    """
    sin_period = 2
    hold_period = 1
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
