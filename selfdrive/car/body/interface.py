#!/usr/bin/env python
import serial
import struct
import time
from hexdump import hexdump
import numpy as np
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
  from common.realtime import Ratekeeper
  rk = Ratekeeper(50)

  can_sock = messaging.sub_sock('can')
  pm = messaging.PubMaster(['sendcan'])
  CP = car.CarParams.new_message()
  ci = CarInterface(CP, None, CarState)

  sm = messaging.SubMaster(['sensorEvents', 'liveLocationKalman'])
  gyro = 0
  accel = None
  dt = 1/50
  intgyro = None

  kp = 4000
  ki = 0
  kd = 500
  i = 0

  last_err = 0

  #set_point = np.deg2rad(-2)
  set_point = np.deg2rad(-3.2)

  while 1:
    sm.update()

    desired_speed = 0
    measured_speed = 0
    P_speed = 0
    set_point =  P_speed * (desired_speed - measured_speed)
    try:
      err = (-sm['liveLocationKalman'].orientationNED.value[1]) - set_point
      d =  -sm['liveLocationKalman'].angularVelocityDevice.value[1]
    except Exception:
      continue

    #print(np.rad2deg(intgyro), np.rad2deg(accel))
    #rk.keep_time()

    i += err
    i = np.clip(i, -2, 2)
    last_err = err

    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    cs = ci.update(None, can_strs)

    ret = car.CarControl.new_message()
    ret.actuators.steer = 0
    #ret.actuators.accel = 0
    ret.actuators.accel = int(np.clip(err*kp + i*ki + d*kd, -200, 200))
    print("%7.2f %7.2f %7.2f %7.2f" % (err*180/3.1415, err, i, d), cs.wheelSpeeds, ret.actuators.accel)
    msgs = ci.apply(ret)
    pm.send('sendcan', can_list_to_can_capnp(msgs, msgtype='sendcan'))
    rk.keep_time()



