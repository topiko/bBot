import time

bits_reserved = [2,11,11]
nbytes = 3

def get_talk_bytes_from_command(cmd):
        
    cmd[1] += 1024
    cmd[2] += 1024

    def make_number_():
        i = 0
        for nb, v in zip(bits_reserved, cmd):
            i = i << nb
            i = i | v
        
        return i 

    def make_bytes_(i):
                                                                                
        vals = [0]*nbytes
        for k in range(nbytes):
            # First byte is the msot significant.
            vals[-k-1] = i & 255
            i = i>>8
        return bytearray(vals)

    return make_bytes_(make_number_())


def talk(ser, cmd):
  
  sendbytes = get_talk_bytes_from_command(cmd)
  ser.write(sendbytes)

def listen(ser):
  
  t0 = time.time()
  while ser.in_waiting<2: pass
  t1 = time.time()

  s = ser.read(2)
  time_pitch = ser.read(4)

  orient = int.from_bytes(s, byteorder='big', signed = True)
  time_pitch = int.from_bytes(time_pitch, byteorder='big', signed = False)
  #print('Imu time: ', time_pitch)
  return orient, time_pitch, t1 - t0

