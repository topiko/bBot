import time
from params import PI
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


def talk(ser, cmd):
    """
    Send command to arduino.
    """
    sendbytes = get_talk_bytes_from_command(cmd)
    ser.write(sendbytes)

def from_orient_int_to_theta(orient_int):
    return orient_int/ANGLE_FACTOR/PI*180



def listen(ser):
    """
    Listen to the serial communication and report the received
    values.
    """
    t0 = time.time()
    while ser.in_waiting < 2:
        pass
    t1 = time.time()

    s = ser.read(2)
    cur_time = ser.read(4)
    #time_pitch = ser.read(2)

    orient_int = int.from_bytes(s, byteorder='big', signed=True)
    cur_time = int.from_bytes(cur_time, byteorder='big', signed=False)/1e6 # from mus to s
    theta = from_orient_int_to_theta(orient_int)

    return theta, cur_time, t1 - t0

def enable_legs(ser):

    talk(ser, [3, 1, 0])
    time.sleep(.1)

def disable_all(ser):

    talk(ser, [0, 0, 0])
    time.sleep(.1)
    talk(ser, [1, 0, 0])
    time.sleep(.1)
    talk(ser, [2, 0, 0])
    time.sleep(.1)
    talk(ser, [3, 0, 0])