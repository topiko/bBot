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

i = 0
t0 = time.time()
while 1:
  send = struct.pack('>B', i)
  #send = struct.pack('<H', i)
  #print('send:    ', i)

  ser.write(send)
  s = int(ser.read(3))
  #print('receive: ', s)
  #print()

  i+=1
  if i%100 == 0:
    t1 = time.time()
    print('Freq: ', i/(t1-t0))
