import time
import serial
import numpy as np
from communication import talk, listen, enable_legs, disable_all #get_talk_bytes_from_cmd
from orientation_utils import update_orient, predict_orient

ser = serial.Serial(
        port='/dev/serial0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=.00001
)

report = True 
# store time and pitch of last 3 time steps
orient_arr = np.zeros((3, 6))  


def act(orient_arr):
    
  a = int((orient_arr[0,1]/35*1024) if not np.isnan(orient_arr[0,1]) else 0)

  if 1023 < a: a = 1023
  elif a < -1024: a = -1024

  return [0,a,a]

def check_status(orient_arr):

    if (orient_arr[0,1] < -30) or (30 < orient_arr[0,1]):
        return 'fell'
    else:
        return 'upright'

i = 0
wait_sum = 0
ni = 20
cmd = [0,1,1]
t0 = 0
t_add = .1
status = 'upright'


# enable the legs:
enable_legs(ser)
imax = 1000
store_arr = np.zeros((imax, orient_arr.shape[1]))

while (i<imax) and (status != 'fell'): # True: 

  talk(ser, cmd)
  orient_arr = predict_orient(orient_arr, t_add)

  cmd = act(orient_arr)
  if (i>20):
      status = check_status(orient_arr)  

  orient, imu_time, wait = listen(ser)

  orient_arr = update_orient(orient_arr, orient, imu_time)
  wait_sum += wait

  if i%ni == 0:
    t1 = time.time()
    print('Freq: ', ni/(t1-t0))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
    if report:
       #print(orient_arr)
       print((orient_arr[:,0] - orient_arr[:, -1]).mean(), t_add)
       print(orient_arr[:,1] - orient_arr[:, -2])
       print()
       t_add = np.diff(orient_arr[::-1,0]).mean()
       wait_sum = 0
       t0 = time.time()
  
  if report:
    store_arr[i,:] = orient_arr[-1,:]

  i+=1

np.save('orient.npy', store_arr[3:])
disable_all(ser)
