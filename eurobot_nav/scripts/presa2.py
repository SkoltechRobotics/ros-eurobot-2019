import serial
import time
import sys

import struct

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

ser.write(bytearray([0x01, 0x02, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
ser.write(bytearray([0x14]))
string = ser.read(3)
print(string)
time.sleep(1)
ser.write(bytearray([0x14]))
string = ser.read(3)
print(string)

#set speed
btarr = bytearray([0x08])
btarr.extend(bytearray(struct.pack("f", 0.2)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
ser.write(btarr)
string = ser.read(3)
print(string)
time.sleep(5);
btarr = bytearray([0x08])
btarr.extend(bytearray(struct.pack("f", 0.01)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
ser.write(btarr)
string = ser.read(3)
print(string)

#set pump low
ser.write(bytearray([0x13]))
string = ser.read(3)
print(string)

time.sleep(1)

#turn on pump
ser.write(bytearray([0x11]))
string = ser.read(3)
print(string)

time.sleep(1)

#set pump high
ser.write(bytearray([0x14]))
string = ser.read(3)
print(string)

#set speed
btarr = bytearray([0x08])
btarr.extend(bytearray(struct.pack("f", -0.208)))
btarr.extend(bytearray(struct.pack("f", 0.078)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
ser.write(btarr)
string = ser.read(3)
print(string)
time.sleep(5);

#set speed
btarr = bytearray([0x08])
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.785)))
ser.write(btarr)
string = ser.read(3)
print(string)
time.sleep(2);

#set speed 0
btarr = bytearray([0x08])
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
btarr.extend(bytearray(struct.pack("f", 0.0)))
ser.write(btarr)
string = ser.read(3)
print(string)

#set pump low
ser.write(bytearray([0x13]))
string = ser.read(3)
print(string)

time.sleep(1)

#turn off pump
ser.write(bytearray([0x12]))
string = ser.read(3)
print(string)
