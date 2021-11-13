#!/usr/bin/env python
import serial
import struct
import time
from hexdump import hexdump
import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker

# sudo chmod 666 /dev/ttyUSB0

def open_serial():
  return serial.Serial('/dev/ttyUSB0', 115200)

def cmd(ser, steer, speed):
  ret = b"\xcd\xab" + struct.pack("hhH", steer, speed, 0xabcd ^ (steer & 0xFFFF) ^ (speed & 0xFFFF))
  ser.write(ret)

def getframe(ser):
  while 1:
    while ser.read() != b"\xcd":
      continue
    if ser.read() == b"\xab":
      return struct.unpack("hhhhhhHH", ser.read(8*2))

if __name__ == "__main__":
  pm = messaging.PubMaster(['can'])
  can_sock = messaging.sub_sock('sendcan')
  cp = CANParser("comma_body", [("SPEED", "BODY_COMMAND", 0),
                                ("STEER", "BODY_COMMAND", 0)], [("BODY_COMMAND", 5)])
  packer = CANPacker("comma_body")
  ser = open_serial()
  while 1:
    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    cp.update_strings(can_strs)
    if cp.can_valid:
      cmd(ser, int(cp.vl['BODY_COMMAND']['STEER']), int(cp.vl['BODY_COMMAND']['SPEED']))
    dat = getframe(ser)
    msg = packer.make_can_msg("BODY_SENSOR", 0, 
      {'SPEED_L': dat[2],
       'SPEED_R': dat[3],
       'BAT_VOLTAGE': dat[4],
       'BOARD_TEMP': dat[5]})
    dat = messaging.new_message('can', size=1)
    dat.can[0] = {"address": msg[0], "busTime": msg[1], "dat": msg[2], "src": msg[3]}
    pm.send('can', dat)

