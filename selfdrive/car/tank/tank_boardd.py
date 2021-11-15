#!/usr/bin/env python3
import usb.core # pylint: disable=import-error
import usb.util # pylint: disable=import-error
import struct

import cereal.messaging as messaging
from opendbc.can.parser import CANParser

ID_VENDOR_LEGO = 0x0694
ID_PRODUCT_EV3 = 0x0005
EP_IN  = 0x81
EP_OUT = 0x01

DIRECT_COMMAND_REPLY = b'\x00'
OP_OUTPUT_SPEED = b'\xA5'
OP_OUTPUT_START = b'\xA6'
OP_OUTPUT_STOP = b'\xA3'
PORT_A = 1
PORT_D = 8


class EV3:
  def __init__(self, host = None):
    self.device = None
    mac_addr = None
    hosts = []
    for dev in usb.core.find(find_all=True, idVendor=ID_VENDOR_LEGO, idProduct=ID_PRODUCT_EV3):
      tmp = usb.util.get_string(dev, dev.iSerialNumber).upper()
      mac_addr = ':'.join((tmp[i:i + 2] for i in range(0, len(tmp), 2)))
      hosts.append(mac_addr)

      if self.device is not None:
        raise RuntimeError('multiple EV3 devices found, you need to set argument host')
      if host is not None:
        if mac_addr == host.upper():
          self.device = dev
          break
      else:
        self.device = dev

    if self.device is None:
      if mac_addr is None:
        raise RuntimeError('No EV3 device found')
      else:
        raise RuntimeError(f'found EV3 devices: {hosts} but not {host}')

    # handle interfaces
    for i in self.device.configurations()[0].interfaces():
      try:
        if self.device.is_kernel_driver_active(i.index):
          self.device.detach_kernel_driver(i.index)
      except NotImplementedError:
        pass
    self.device.set_configuration()
    self.device.read(EP_IN, 1024, 100)

  def send_cmd(self, ops: bytes, local_mem: int=0, global_mem: int=0):
    cmd = b''.join([
      struct.pack('<h', len(ops) + 5),
      struct.pack('<h', 42),
      DIRECT_COMMAND_REPLY,
      struct.pack('<h', local_mem*1024 + global_mem),
      ops
    ])
    try:
      self.device.write(EP_OUT, cmd, 100)
      self.device.read(EP_IN, 1024, 100)
    except BaseException:
      pass



# Generate comands

def LCX(value):
  if -32 <= value < 0:
    return struct.pack('b', 0x3F & (value + 64))
  if 0 <= value < 32:
    return struct.pack('b', value)
  if -127 <= value <= 127:
    return b'\x81' + struct.pack('<b', value)
  if -32767 <= value <= 32767:
    return b'\x82' + struct.pack('<h', value)
  return b'\x83' + struct.pack('<i', value)

def start(speed_left, speed_right):
  return b''.join([
    OP_OUTPUT_SPEED,
    LCX(0),           # LAYER
    LCX(PORT_A),      # PORT
    LCX(speed_left),  # SPEED

    OP_OUTPUT_SPEED,
    LCX(0),           # LAYER
    LCX(PORT_D),      # PORT
    LCX(speed_right), # SPEED

    OP_OUTPUT_START,
    LCX(0),
    LCX(PORT_A + PORT_D)
  ])

def stop():
  return b''.join([
    OP_OUTPUT_STOP,
    LCX(0),               # LAYER
    LCX(PORT_A + PORT_D), # PORT
    LCX(1)                # BRAKE
  ])


if __name__ == '__main__':
  can_sock = messaging.sub_sock('sendcan')
  cp = CANParser("comma_tank", [("SPEED_LEFT", "TANK_COMMAND", 0),
                                ("SPEED_RIGHT", "TANK_COMMAND", 0)], [("TANK_COMMAND", 5)])
  ev3 = EV3()

  try:
    while 1:
      can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
      cp.update_strings(can_strs)
      if cp.can_valid:
        speed_left = int(cp.vl['TANK_COMMAND']['SPEED_LEFT'])
        speed_right = int(cp.vl['TANK_COMMAND']['SPEED_RIGHT'])
        if speed_left != 0 and speed_right != 0:
          ev3.send_cmd(start(speed_left, speed_right))
        else:
          ev3.send_cmd(stop())
  except KeyboardInterrupt:
    ev3.send_cmd(stop())
    raise
