import cereal.messaging as messaging
from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error

def send_cmd(pm, speed_left, speed_right):
  msg = packer.make_can_msg("TANK_COMMAND", 0,
    {"SPEED_LEFT": int(speed_left),
     "SPEED_RIGHT": int(speed_right)})
  pm.send('sendcan', can_list_to_can_capnp(msgs, msgtype='sendcan'))


if __name__ == "__main__":
  packer = CANPacker("comma_tank")
  pm = messaging.PubMaster(['sendcan'])

  for i in range(3):
    send_cmd(pm, 100, -100)
    time.sleep(0.3)
    send_cmd(pm, 0, 0)
    time.sleep(0.3)

    send_cmd(pm, -100, 100)
    time.sleep(0.3)
    send_cmd(pm, 0, 0)
    time.sleep(0.3)
