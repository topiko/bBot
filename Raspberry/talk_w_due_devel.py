
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
report = False #True 
# store time and pitch of last 3 time steps
orient_arr = np.zeros((3, 5))  

def act(orient_arr):
    
  time.sleep(.05)
  a = int(-(orient_arr[0,0]/90)*1024)
  return [0,a,a]

def update_orient(orient_arr, orient, time_orient):

  orient = orient/angle_factor/pi*180
  orient_arr[0,:2] = time_orient, orient 

  return orient_arr

def predict_orient(orient_arr, time_orient):
  
  # fit 2 order poly to 3 values in orient arr 
  # --> w, dw/dt, d**2w/dt**2 
  # use these to predict values at time_orient
  p1, p2, p3 = orient_arr[:, 1] 
  t1, t2, t3 = orient_arr[:, 0]

  a = (p1*(t2 - t3) - p2*(t1 - t3) + p3*(t1 - t2)) \
       /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  b = (-p1*(t2**2 - t3**2) + p2*(t1**2 - t3**2) - p3*(t1**2 - t2**2)) \
          /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  c = (p1*t2*t3*(t2 - t3) - p2*t1*t3*(t1 - t3) + p3*t1*t2*(t1 - t2)) \
          /(t1**2*t2 - t1**2*t3 - t1*t2**2 + t1*t3**2 + t2**2*t3 - t2*t3**2)
  
  p_now = a*time_orient**2 + b*time_orient + c
  w_now = 2*a*time_orient + b
  wdot_now = 2*a
  
  tl = orient_arr[0,0]
  w_last = 2*a*tl + b
  wdot_last = 2*a
   
  orient_arr[1:,:] = orient_arr[:-1,:]
  orient_arr[1,2:-1] = w_last, wdot_last
  orient_arr[0,:] = time_orient, p_now, w_now, wdot_now, p_now

  return orient_arr


i = 0
orient = 0 
wait_sum = 0
ni = 1
cmd = [0,1,1]
t_init = time.time()
t0 = 0
while True: #:orient not in [0, 1023]:

  # time pitch is measure is roughly here...  
  time_orient = time.time() - t_init
  talk(ser, cmd)
  orient_arr = predict_orient(orient_arr, time_orient)

  cmd = act(orient_arr)
  orient, wait = listen(ser)

  orient_arr = update_orient(orient_arr, orient, time_orient)


  wait_sum += wait

  i+=1
  if i%ni == 0:
    t1 = time.time()
    print('Time per 1 loop: {:.03f}ms'.format((t1-t0)*1000/ni))
    print('Pitch = {:.2f}deg, Freq: {}'.format(orient, ni/(t1-t0)))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
    print(orient_arr)
    wait_sum = 0
    t0 = time.time()
