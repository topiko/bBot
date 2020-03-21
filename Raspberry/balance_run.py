import time
import serial
import numpy as np
from communication import talk, listen, enable_legs, disable_all #get_talk_bytes_from_cmd
from orientation_utils import update_orient, predict_orient
from simple_pid import PID
import sys

if len(sys.argv) == 2:
    mode = sys.argv[1]
else:
    mode = 'test_mpu'

ser = serial.Serial(
        port='/dev/serial0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=.001
)

upright_angle = 15;
pid = PID(25, 200, -0.1, setpoint=upright_angle)

report = True
# store time and pitch of last 3 time steps
orient_arr = np.zeros((3, 12))

def get_cmd(phi, phidot, phidotdot, v, dt):
    
    phidodot = 0 

def act(orient_arr):

    phi = orient_arr[0,1] if not np.isnan(orient_arr[0,1]) else upright_angle
    a = int(pid(phi))

    #a = get_cmd(phi, phidot, vl)

    if 1023 < a: a = 1023
    elif a < -1024: a = -1024

    return [0,a,a]

def check_status(orient_arr):

    if ((orient_arr[0,1]-upright_angle) < -30) or (30 < (orient_arr[0,1]-upright_angle)):
        print('FELL: {}'.format(orient_arr[0,1]-upright_angle))
        print(orient_arr)
        return 'fell'
    else:
        return 'upright'


i = 0
wait_sum = 0
ni = 50
cmd = [0,1,1]
t0 = 0
t_add = .015
status = 'upright'

# enable the legs:
if mode == 'test_mpu':
    pass
else:
    enable_legs(ser)
#try:
#    accel_legs()
#except KeyboardInterrupt:
#    pass

#disable_all(ser)
#exit()

imax = 3000
store_arr = np.zeros((imax, orient_arr.shape[1]))

while (i<imax) and (status != 'fell'): # True: 

  talk(ser, cmd)
  orient_arr = predict_orient(orient_arr, t_add)

  cmd = act(orient_arr)

  orient, imu_time, wait = listen(ser)

  orient_arr = update_orient(orient_arr, orient, cmd, imu_time)
  wait_sum += wait

  if (i>20):
      status = check_status(orient_arr)

  if i%ni == 0:
    t1 = time.time()
    print('Freq: ', ni/(t1-t0))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
    if report:
       print(orient_arr)
       print()
       wait_sum = 0
       t0 = time.time()
    if i != 0: 
      t_add = np.diff(orient_arr[::-1,0]).mean()

  if report:
    store_arr[i,:] = orient_arr[-1,:]
  i+=1
print(status)

np.save('orient.npy', store_arr[3:i])
disable_all(ser)
