#!/usr/bin/env python
import serial
import struct
import time
from hexdump import hexdump

# sudo chmod 666 /dev/ttyUSB0

ser = serial.Serial('/dev/ttyUSB0', 115200)
print(ser)

def cmd(steer, speed):
  ret = b"\xcd\xab" + struct.pack("hhH", steer, speed, 0xabcd ^ steer ^ speed)
  ser.write(ret)

def getframe():
  while 1:
    while ser.read() != b"\xcd":
      continue
    if ser.read() == b"\xab":
      return struct.unpack("hhhhhhHH", ser.read(8*2))

if __name__ == "__main__":
  while 1:
    cmd(0, 100)
    dat = getframe()
    print(time.time(), dat)



