#!/usr/bin/env python
import serial
import struct
import time
from hexdump import hexdump

# sudo chmod 666 /dev/ttyUSB0

def open_serial():
  return serial.Serial('/dev/ttyUSB0', 115200)

def cmd(ser, steer, speed):
  ret = b"\xcd\xab" + struct.pack("hhH", steer, speed, 0xabcd ^ steer ^ speed)
  ser.write(ret)

def getframe(ser):
  while 1:
    while ser.read() != b"\xcd":
      continue
    if ser.read() == b"\xab":
      return struct.unpack("hhhhhhHH", ser.read(8*2))

from selfdrive.car.interfaces import CarInterfaceBase
from cereal import car

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.ser = open_serial()

  def update(self, c, can_strings):
    dat = getframe(self.ser)
    ret = car.CarState.new_message()
    ret.wheelSpeeds.fl = -dat[2]
    ret.wheelSpeeds.fr = dat[3]
    return ret

  def apply(self, c):
    cmd(self.ser, int(c.actuators.steer), int(c.actuators.accel))

if __name__ == "__main__":
  ci = CarInterface(car.CarParams.new_message(), None, None)
  while 1:
    cs = ci.update(None, None)
    print(cs.wheelSpeeds)
    ret = car.CarControl.new_message()
    ret.actuators.steer = 0
    ret.actuators.accel = 100
    ci.apply(ret)





