#!/usr/bin/env python3
import unittest

from parameterized import parameterized

from cereal import car, log
from selfdrive.car.car_helpers import interfaces
from selfdrive.car.honda.values import CAR as HONDA
from selfdrive.car.toyota.values import CAR as TOYOTA
from selfdrive.car.nissan.values import CAR as NISSAN
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.vehicle_model import VehicleModel


class TestLatControl(unittest.TestCase):

  @parameterized.expand([(HONDA.CIVIC, LatControlPID), (TOYOTA.RAV4, LatControlTorque), (TOYOTA.PRIUS, LatControlINDI), (NISSAN.LEAF, LatControlAngle)])
  def test_saturation(self, car_name, controller):
    CarInterface, CarController, CarState = interfaces[car_name]
    CP = CarInterface.get_params(car_name)
    CI = CarInterface(CP, CarController, CarState)
    VM = VehicleModel(CP)

    controller = controller(CP, CI)


    CS = car.CarState.new_message()
    CS.vEgo = 30

    last_actuators = car.CarControl.Actuators.new_message()

    params = log.LiveParametersData.new_message()

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, CP, VM, params, last_actuators, 1, 0)

    self.assertTrue(lac_log.saturated)


if __name__ == "__main__":
  unittest.main()
