#!/usr/bin/env python3
from opendbc.can.packer import CANPacker
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error
from selfdrive.car import crc8_pedal

packer = CANPacker("hyundai_kia_generic")
rpacker = CANPacker("hyundai_kia_mando_front_radar")


def can_function(pm, speed, angle, idx, cruise_button, is_engaged, blinkers, SAS11, MDPS12):
  msg = []
  left_blinker, right_blinker = blinkers["left"], blinkers["right"]

  speed = speed * 3.6  # convert m/s to kph
  msg.append(packer.make_can_msg("WHL_SPD11", 0, {
    "WHL_SPD_FL": speed,
    "WHL_SPD_FR": speed,
    "WHL_SPD_RL": speed,
    "WHL_SPD_RR": speed
  }, -1))

  msg.append(packer.make_can_msg("CGW1", 0, {
    "CF_Gway_DrvDrSw": 0,
    "CF_Gway_AstDrSw": 0,
    "CF_Gway_DrvSeatBeltSw": 1,
    "CF_Gway_TurnSigLh": int(left_blinker),
    "CF_Gway_TurnSigRh": int(right_blinker)
  }, idx))
  
  msg.append(packer.make_can_msg("CGW2", 0, {"CF_Gway_RRDrSw": 0, "CF_Gway_RLDrSw": 0}, idx))

  msg.append(packer.make_can_msg("TCS13", 0, {"ACCEnable": 0, "ACC_REQ": int(is_engaged)}, idx))
  msg.append(packer.make_can_msg("CLU11", 0, {"CF_Clu_CruiseSwState": cruise_button}, idx))
  msg.append(packer.make_can_msg("LVR12", 0, {"CF_Lvr_Gear": 5}, idx))
  msg.append(packer.make_can_msg("TCS15", 0, {}, idx))
  msg.append(packer.make_can_msg("ESP12", 0, {}, idx))
  msg.append(packer.make_can_msg("CGW4", 0, {}, idx))
  msg.append(packer.make_can_msg("EMS12", 0, {}, idx))
  msg.append(packer.make_can_msg("EMS16", 0, {}, idx))
  msg.append(packer.make_can_msg("LKAS11", 2, {}, idx))
  if SAS11 is not None:
    msg.append(SAS11)
  else:
    msg.append(packer.make_can_msg("SAS11", 0, {"SAS_Angle": angle}, idx))
  if MDPS12 is not None:
    msg.append(MDPS12)
  else:
    msg.append(packer.make_can_msg("MDPS12", 0, {"CR_Mdps_StrColTq": 0}, idx))

#   values = {"COUNTER_PEDAL": idx & 0xF}
#   checksum = crc8_pedal(packer.make_can_msg("GAS_SENSOR", 0, {"COUNTER_PEDAL": idx & 0xF}, -1)[2][:-1])
#   values["CHECKSUM_PEDAL"] = checksum
#   msg.append(packer.make_can_msg("GAS_SENSOR", 0, values, -1))

#   msg.append(packer.make_can_msg("GEARBOX", 0, {"GEAR": 4, "GEAR_SHIFTER": 8}, idx))
#   msg.append(packer.make_can_mscruise_buttonsg("GAS_PEDAL_2", 0, {}, idx))
#   msg.append(packer.make_can_msg("SEATBELT_STATUS", 0, {"SEATBELT_DRIVER_LATCHED": 1}, idx))
#   msg.append(packer.make_can_msg("STEER_STATUS", 0, {"STEER_TORQUE_SENSOR": torque}, idx))
#   msg.append(packer.make_can_msg("STEERING_SENSORS", 0, {"STEER_ANGLE": angle}, idx))
#   msg.append(packer.make_can_msg("VSA_STATUS", 0, {}, idx))
#   msg.append(packer.make_can_msg("STANDSTILL", 0, {"WHEELS_MOVING": 1 if speed >= 1.0 else 0}, idx))
#   msg.append(packer.make_can_msg("STEER_MOTOR_TORQUE", 0, {}, idx))
#   msg.append(packer.make_can_msg("EPB_STATUS", 0, {}, idx))
#   msg.append(packer.make_can_msg("CRUISE_PARAMS", 0, {}, idx))
#   msg.append(packer.make_can_msg("CRUISE", 0, {}, idx))
#   msg.append(packer.make_can_msg("POWERTRAIN_DATA", 0, {"ACC_STATUS": int(is_engaged)}, idx))
#   msg.append(packer.make_can_msg("HUD_SETTING", 0, {}, idx))

#   # *** cam bus ***
#   msg.append(packer.make_can_msg("STEERING_CONTROL", 2, {}, idx))
#   msg.append(packer.make_can_msg("ACC_HUD", 2, {}, idx))
#   msg.append(packer.make_can_msg("BRAKE_COMMAND", 2, {}, idx))

  # *** radar bus ***
  # if idx % 5 == 0:
  #   msg.append(rpacker.make_can_msg("RADAR_DIAGNOSTIC", 1, {"RADAR_STATE": 0x79}, -1))
  #   for i in range(16):
  #     msg.append(rpacker.make_can_msg("TRACK_%d" % i, 1, {"LONG_DIST": 255.5}, -1))

  pm.send('can', can_list_to_can_capnp(msg))
