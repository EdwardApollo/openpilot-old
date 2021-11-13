#!/usr/bin/env python
import serial
import struct
import time
from hexdump import hexdump
import cereal.messaging as messaging

from selfdrive.car.interfaces import CarInterfaceBase
from cereal import car

from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
packer = CANPacker("comma_body")

from selfdrive.car.interfaces import CarStateBase
class CarState(CarStateBase):
  def update(self, cp):
    ret = car.CarState.new_message()
    ret.wheelSpeeds.fl = -cp.vl['BODY_SENSOR']['SPEED_L']
    ret.wheelSpeeds.fr = cp.vl['BODY_SENSOR']['SPEED_R']
    return ret

  @staticmethod
  def get_can_parser(CP):
    return CANParser("comma_body", [("SPEED_L", "BODY_SENSOR", 0),
                                    ("SPEED_R", "BODY_SENSOR", 0)], [], enforce_checks=False)

class CarInterface(CarInterfaceBase):
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    ret = self.CS.update(self.cp)

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    msg = packer.make_can_msg("BODY_COMMAND", 0,
      {"SPEED": int(c.actuators.accel),
       "STEER": int(c.actuators.steer)})
    return [msg]

if __name__ == "__main__":
  sm = messaging.SubMaster(['liveLocationKalman'])

  can_sock = messaging.sub_sock('can')
  pm = messaging.PubMaster(['sendcan'])

  CP = car.CarParams.new_message()
  ci = CarInterface(CP, None, CarState)

  from common.realtime import Ratekeeper
  rk = Ratekeeper(50)
  i = 0

  kp = -2000
  ki = 0
  kd = -200

  last_err = 0
  while 1:
    sm.update()
    err = sm['liveLocationKalman'].orientationNED.value[1] #- 0.032
    i += err
    d = err - last_err
    last_err = err

    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    cs = ci.update(None, can_strs)

    ret = car.CarControl.new_message()
    ret.actuators.steer = 0
    ret.actuators.accel = 0
    #ret.actuators.accel = int(err*kp + i*ki + d*kd)
    print("%7.2f %7.2f %7.2f %7.2f" % (err*180/3.1415, err, i, d), cs.wheelSpeeds, ret.actuators.accel)
    msgs = ci.apply(ret)
    pm.send('sendcan', can_list_to_can_capnp(msgs, msgtype='sendcan'))
    rk.keep_time()





