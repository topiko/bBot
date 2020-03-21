
import sys
import time
import serial

import numpy as np
from utils.communication import talk, listen, \
        enable_legs, disable_all #get_talk_bytes_from_cmd
from utils.state import update_state, predict_theta, \
        check_status
from utils.control import act

if len(sys.argv) == 2:
    MODE = sys.argv[1]
else:
    MODE = 'test_mpu'

SER = serial.Serial(
    port='/dev/serial0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=.001
)


REPORT = True



def __main__():
    i = 0
    wait_sum = 0
    ni = 50
    cmd = [0, 1, 1]
    t0 = 0
    t_add = .015
    status = 'upright'

    # store time and pitch of last 3 time steps
    state_dict = {'theta':np.zeros((4, 3)),
                  'legs':np.zeros((4, 6))}

    # enable the legs:
    if MODE == 'test_mpu':
        pass
    else:
        enable_legs(SER)

    imax = 1000
    store_arr = np.zeros((imax, state_dict['leg'].shape[1]))

    while (i < imax) and (status != 'fell'): # True:

        talk(SER, cmd)
        predict_theta(state_dict['theta'], t_add)

        cmd = act(state_dict)

        orient, imu_time, wait = listen(SER)

        state_dict = update_state(state_dict, orient, cmd, imu_time)
        wait_sum += wait

        if (i > 20):
            status = check_status(state_dict)

        if i%ni == 0:
            t1 = time.time()
            print('Freq: ', ni/(t1-t0))
            print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
        if REPORT:
            print(state_dict)
            print()
            wait_sum = 0
            t0 = time.time()
        if i != 0:
            t_add = np.diff(state_dict[::-1, 0]).mean()

        if REPORT:
            store_arr[i, :] = state_dict[-1, :]
        i += 1
    print(status)

    np.save('orient.npy', store_arr[3:i])
    disable_all(SER)
