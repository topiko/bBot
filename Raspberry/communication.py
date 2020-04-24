import time
from params import PI
import numpy as np
BITS_RESERVED = [2, 11, 11]
NBYTES = 3
ANGLE_FACTOR = -20860.

def get_talk_bytes_from_command(cmd):
    """
    Parse bytes that are send to arduino from cmd.
    """
    cmd[1] += 1024
    cmd[2] += 1024

    def make_number_():
        """
        Make 8 bit number numbers from cmd.
        """
        i = 0
        for nb, cmd_v in zip(BITS_RESERVED, cmd):
            i = i << nb
            i = i | cmd_v
        return i

    def make_bytes_(i):
        vals = [0]*NBYTES
        for k in range(NBYTES):
            # First byte is the msot significant.
            vals[-k-1] = i & 255
            i = i>>8
        return bytearray(vals)

    return make_bytes_(make_number_())


def talk(ser, state_dict, cmd_dict):
    """
    Send command to arduino.
    """
    if state_dict['mode'].startswith('simul'):
        ser.run(state_dict, cmd_dict)
    else:
        sendbytes = get_talk_bytes_from_command(cmd_dict['cmd'])
        ser.write(sendbytes)

def from_orient_int_to_theta(orient_int):
    return orient_int/ANGLE_FACTOR/PI*180



def listen(ser, mode=None):
    """
    Listen to the serial communication and report the received
    values.
    """
    if mode.startswith('simul'):
        return ser.theta_noisy, ser.time, 0
    else:
        t0 = time.time()
        while ser.in_waiting < 2:
            pass
        t1 = time.time()

        init_byte = ser.read(1)
        if init_byte != b'\x00': #b'00000000':
            print(init_byte) #, int(init_byte))
            raise ValueError('Communication issues')

        s = ser.read(2)
        cur_time = ser.read(4)
        #time_pitch = ser.read(2)

        orient_int = int.from_bytes(s, byteorder='big', signed=True)
        cur_time = int.from_bytes(cur_time, byteorder='big', signed=False)/1e6 # from mus to s
        theta = from_orient_int_to_theta(orient_int)

        return theta, cur_time, t1 - t0

def enable_legs(ser, mode):

    if mode.startswith('simul'):
        return
    talk(ser, {'mode':'run'}, {'cmd': [3, 1, 0]})
    time.sleep(.1)

def initialize_state_dict(ser, state_dict):
    """
    Initialize the state dict arrays.
    """
    if state_dict['mode'].startswith('simul'):
        for i in range(len(state_dict['times'])):
            ser.run(state_dict, {'a':0})
            state_dict['theta'][-i-1] = ser.theta
            state_dict['thetadot'][-i-1] = ser.thetadot
            state_dict['times'][-i-1] = ser.time
        state_dict['time_next'] = ser.time + state_dict['dt'] #ser.dt*3
    else:
        for i in range(len(state_dict['times'])):
            talk(ser, state_dict, {'cmd': [0, 0, 0]})
            theta, cur_time, _ = listen(ser, mode=state_dict['mode'])
            state_dict['times'][-i-1] = cur_time
            state_dict['theta'][-i-1] = theta
        state_dict['time_next'] = cur_time + state_dict['dt'] #.016

def disable_all(ser, state_dict):
    """
    Disable all msucles.
    """
    if state_dict['mode'].startswith('simul'):
        return
    talk(ser, state_dict, {'cmd': [0, 0, 0]})
    time.sleep(.1)
    talk(ser, state_dict, {'cmd': [1, 0, 0]})
    time.sleep(.1)
    talk(ser, state_dict, {'cmd': [2, 0, 0]})
    time.sleep(.1)
    talk(ser, state_dict, {'cmd': [3, 0, 0]})
