import numpy as np

angle_factor = 20860.
pi = 3.14159267
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

