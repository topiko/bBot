
import time
import serial
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
  

def act(orient):
    
  time.sleep(.0015)
  a = int(-(orient/90)*1024)
  return [0,a,a]

i = 0
orient = 0 
wait_sum = 0
ni = 20
cmd = [0,1,1]
t0 = 0
while True: #:orient not in [0, 1023]:

  talk(ser, cmd)

  cmd = act(orient)
  orient, wait = listen(ser)
  orient = orient/angle_factor/pi*180

  wait_sum += wait

  i+=1
  if i%ni == 0:
    t1 = time.time()
    print('Time per 1 loop: {:.03f}ms'.format((t1-t0)*1000/ni))
    print('Pitch = {:.2f}deg, Freq: {}'.format(orient, ni/(t1-t0)))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
  
    wait_sum = 0
    t0 = time.time()
