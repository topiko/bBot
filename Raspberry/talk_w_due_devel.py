
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

def make_bytes(control_cmd):

    return bytearray([0,1])

def talk(control_cmd):
  
  sendbytes = make_bytes(control_cmd)
  ser.write(sendbytes)
  #for s in [a, b, c]:
  #  send = struct.pack('>B', s)
  #  ser.write(send)
  
  orient = 0
  n = 0
  while orient != 0:
    s = ser.read(2)
    orient = int.from_bytes(s, byteorder='big', signed = True)
    n += 1;
  
  if n>1:
      print('Had to read {} times.'.fromat(n))

  return orient/angle_factor/pi*180

def act(i):
  a = i >> 8
  b = i & 0xff
  return a, b

i = 0
orient = 10
t0 = time.time()
while True: #:orient not in [0, 1023]:

  a,b  = act(i)
  orient = talk(a,b)

  print(orient)
  print() 

  i+=1
  if i%100 == 0:
    t1 = time.time()
    print('Freq: ', i/(t1-t0))
  
