
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
def talk(a,b,c):
  
  #print('send: ', a,b,c)
  for s in [a, b, c]:
    send = struct.pack('>B', s)
    ser.write(send)

  s = ser.read(2)
  
  return int.from_bytes(s, byteorder='big', signed = True)/angle_factor/pi*180
  #return struct.unpack('>b', s)

def act(i):
  a = i >> 8
  b = i & 0xff
  c = 3
  return a, b, c

i = 0
orient = 10
t0 = time.time()
mes = 0
while True: #:orient not in [0, 1023]:

  a,b,c  = act(i)
  orient = talk(a,b,c)
  if orient !=0:

    mes += 1  
    print(orient)
    print() 

  i+=1
  if i%100 == 0:
    t1 = time.time()
    print('Freq: ', mes/(t1-t0))
  
