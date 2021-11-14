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

import atexit

if __name__ == "__main__":
  from common.realtime import Ratekeeper
  dtn = 80
  rk = Ratekeeper(dtn)

  can_sock = messaging.sub_sock('can')
  pm = messaging.PubMaster(['sendcan'])
  CP = car.CarParams.new_message()
  ci = CarInterface(CP, None, CarState)

  def done():
    print("sending 0")
    ret = car.CarControl.new_message()
    msgs = ci.apply(ret)
    pm.send('sendcan', can_list_to_can_capnp(msgs, msgtype='sendcan'))

  atexit.register(done)

  sm = messaging.SubMaster(['sensorEvents', 'liveLocationKalman', 'testJoystick'])
  gyro = 0
  accel = None
  dt = 1/dtn
  intgyro = None

  kp = 1000
  ki = 0
  kd = 250
  i = 0
  d = 0
  i_speed = 0

  last_err = 0

  #set_point = np.deg2rad(-2)
  set_point = np.deg2rad(-0)

  acc_err = 0

  joy_x,joy_y = 0,0
  measured_speed = 0
  measured_steer = 0
  desired_speed = 0
  desired_steer = 0
  time = 0.0
  torque_a_filt = 0.0
  torque_b_filt = 0.0
  while 1:
    time += dt
    sm.update()

    try:
      joy_x = sm['testJoystick'].axes[0]
      joy_y = sm['testJoystick'].axes[1]
    except Exception:
      pass


    alpha = 1.0
    desired_speed = (1. - alpha)*desired_speed + alpha *(joy_y/2)
    desired_steer = (1. - alpha)*desired_steer + alpha*joy_x
    kp_speed = 0.001
    ki_speed = 0.00001
    i_speed += ki_speed * (desired_speed - measured_speed)
    i_speed = np.clip(i_speed, -0.1, 0.1)
    set_point =  kp_speed * (desired_speed - measured_speed) + i_speed
    try:
      err = (-sm['liveLocationKalman'].orientationNED.value[1]) - set_point
      d_new =  -sm['liveLocationKalman'].angularVelocityDevice.value[1]
      alpha_d = 1.0
      d = (1. - alpha_d) * d + alpha * d_new
      d =  np.clip(d, -1., 1.)
    except Exception:
      continue

    err += np.deg2rad(joy_y)/50


    #print(np.rad2deg(intgyro), np.rad2deg(accel))
    #rk.keep_time()

    i += err
    i = np.clip(i, -2, 2)
    last_err = err

    can_strs = messaging.drain_sock_raw(can_sock, wait_for_one=False)
    cs = ci.update(None, can_strs)
    measured_speed = 0.5 * (cs.wheelSpeeds.fl + cs.wheelSpeeds.fr)
    measured_steer = -cs.wheelSpeeds.fl + cs.wheelSpeeds.fr

    ret = car.CarControl.new_message()
    speed = int(np.clip(err*kp + acc_err*ki + d*kd, -200, 200))

    kp_steer = 0.2
    diff_torque = kp_steer * (desired_steer - measured_steer)

    #ret.actuators.steer = speed + diff_torque
    #ret.actuators.accel = speed - diff_torque
    #A = 200
    #print(time)
    #ret.actuators.steer = float(A * np.sin(50*time)) # speed + diff_torque
    #ret.actuators.accel = float(A * np.sin(50*time)) # speed - diff_torque
    torque_a = speed + diff_torque
    torque_b = speed - diff_torque
    alpha_torque = .3
    torque_a_filt = (1. - alpha_torque) * torque_a_filt + alpha_torque * torque_a
    torque_b_filt = (1. - alpha_torque) * torque_b_filt + alpha_torque * torque_b
    ret.actuators.steer = torque_a_filt
    ret.actuators.accel = torque_b_filt
    #ret.actuators.steer = joy_x*2
    #ret.actuators.accel = 5
    #ret.actuators.accel =
    #print("%7.2f %7.2f %7.2f %7.2f" % (err*180/3.1415, err, acc_err, d), cs.wheelSpeeds, ret.actuators.accel, joy_x, joy_y)
    print(f'joy forward : {joy_y}, joy steer: {joy_x}')
    print(f'Set point : {set_point}')
    print(f'speeds : {cs.wheelSpeeds}')
    acc_err += cs.wheelSpeeds.fl + cs.wheelSpeeds.fr
    msgs = ci.apply(ret)
    pm.send('sendcan', can_list_to_can_capnp(msgs, msgtype='sendcan'))
    rk.keep_time()



