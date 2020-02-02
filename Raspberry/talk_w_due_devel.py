
import time
import serial
import numpy as np
from communication import talk, listen #get_talk_bytes_from_cmd

ser = serial.Serial(
        port='/dev/serial0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=.00001
)

angle_factor = 20000.
pi = 3.14159267
report = True #False #True 
# store time and pitch of last 3 time steps
orient_arr = np.zeros((3, 6))  

def act(orient_arr):
    
  time.sleep(.015)
  a = int(-(orient_arr[0,1]/90)*1024) if not np.isnan(orient_arr[0,1]) else 0
  return [0,a,a]

def update_orient(orient_arr, orient, time_orient):

  orient = orient/angle_factor/pi*180
  orient_arr[0,:2] = time_orient/1e6, orient 

  return orient_arr

def predict_orient(orient_arr, add_time):
  
  # fit 2 order poly to 3 values in orient arr 
  # --> w, dw/dt, d**2w/dt**2 
  # use these to predict values at time_orient
  p1, p2, p3 = orient_arr[:, 1] 
  t1, t2, t3 = orient_arr[:, 0] - orient_arr[0,0]

  if (np.diff(orient_arr[:, 0]) == 0).any(): 
      print('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')

  a = (p1*(t2 - t3) - p2*(t1 - t3) + p3*(t1 - t2)) \
       /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  b = (-p1*(t2**2 - t3**2) + p2*(t1**2 - t3**2) - p3*(t1**2 - t2**2)) \
          /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  c = (p1*t2*t3*(t2 - t3) - p2*t1*t3*(t1 - t3) + p3*t1*t2*(t1 - t2)) \
          /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  
  time_orient = add_time #np.diff(orient_arr[:,0]).mean()
  p_now = a*time_orient**2 + b*time_orient + c
  w_now = 2*a*time_orient + b
  wdot_now = 2*a
  
  w_last = 2*a*t2 + b
  wdot_last = 2*a
   
  orient_arr[1:,:] = orient_arr[:-1,:]
  orient_arr[1,2:4] = w_last, wdot_last
  orient_arr[0,:] = orient_arr[0,0] + time_orient, p_now, w_now, wdot_now, p_now, orient_arr[0,0] + time_orient

  return orient_arr


i = 0
orient = 0 
wait_sum = 0
ni = 5
cmd = [0,1,1]
t_init = time.time()
t0 = 0
t_add = .1
while True: 

  # time pitch is measure is roughly here...  
  talk(ser, cmd)
  orient_arr = predict_orient(orient_arr, t_add)

  cmd = act(orient_arr)
  orient, imu_time, wait = listen(ser)

  orient_arr = update_orient(orient_arr, orient, imu_time)

  wait_sum += wait

  if i%ni == 0:
    t1 = time.time()
    print('Freq: ', ni/(t1-t0))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
    if report:
      print(orient_arr)
      print((orient_arr[:,0] - orient_arr[:, -1]).mean(), t_add)
      print(orient_arr[:,1] - orient_arr[:, -2])

    print()
    t_add = np.diff(orient_arr[::-1,0]).mean()
    wait_sum = 0
    t0 = time.time()

  i+=1
