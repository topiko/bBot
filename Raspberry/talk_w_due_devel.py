
import time
import serial
import struct


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
report = True 

def make_bytes(control_cmd):

  return bytearray([3,1,2])

def talk(control_cmd):
  
  sendbytes = make_bytes(control_cmd)
  ser.write(sendbytes)

def listen():
  
  t0 = time.time()
  while ser.in_waiting<1: pass
  t1 = time.time()

  s = ser.read(2)
 
  orient = int.from_bytes(s, byteorder='big', signed = True)
  if report:
    print('Time spent = {:.3f}ms'.format((t1-t0)*1000))

  return orient/angle_factor/pi*180, t1 - t0
  

def act(i):
  a = i >> 8
  b = i & 0xff

  time.sleep(.001)




  return a, b

i = 0
orient = 10
wait_sum = 0
t0 = time.time()
while True: #:orient not in [0, 1023]:

  a ,b = act(i)
  talk(1)
  act(i)
  orient, wait = listen()

  wait_sum += wait

  if report:
    print(orient)
    print() 
  i+=1
  if i%100 == 0:
    t1 = time.time()
    print('Pitch = {:.2f}deg, Freq: {}'.format(orient, i/(t1-t0)))
    print('Fraction time waiting serial: {:.3f}'.format(wait_sum/(t1-t0)))
  
