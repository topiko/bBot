
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

def talk(a,b,c):

  for s in [1, a, b]:
    send = struct.pack('>B', s)
    ser.write(send)
  
  s = ser.read(2)
  return int.from_bytes(s, byteorder='big')
  #return struct.unpack('>B', s)

def act(orient):
  a = i >> 8
  b = i & 0xff
  c = 3
  return a, b, c

i = 0
orient = 0
t0 = time.time()
while orient not in [0, 1023]:
  
  # a,b,c = get action from orient
  
  a,b,c  = act(orient)
  orient = talk(a,b,c)


  i+=1
  if i%100 == 0:
    t1 = time.time()
    print('Freq: ', i/(t1-t0))

