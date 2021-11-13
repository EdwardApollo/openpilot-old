import time
from curses import wrapper

import cereal.messaging as messaging
from cereal import car
from opendbc.can.packer import CANPacker
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error

def send_cmd(pm, packer, speed_left, speed_right):
  msg = packer.make_can_msg("TANK_COMMAND", 0,
    {"SPEED_LEFT": int(speed_left),
     "SPEED_RIGHT": int(speed_right)})
  pm.send('sendcan', can_list_to_can_capnp([msg], msgtype='sendcan'))


def main(stdscr):
  packer = CANPacker("comma_tank")
  pm = messaging.PubMaster(['sendcan'])

  stdscr.clear()
  stdscr.addstr(0, 0, 'Stopped')
  stdscr.refresh()

  while True:
    c = stdscr.getch()
    stdscr.clear()

    if c == ord('w'):
      stdscr.addstr(0, 0, 'Moving forward')
      send_cmd(pm, packer, 100, 100)
    elif c == ord('a'):
      stdscr.addstr(0, 0, 'Turning left')
      send_cmd(pm, packer, 100, -100)
    elif c == ord('d'):
      stdscr.addstr(0, 0, 'Turning right')
      send_cmd(pm, packer, -100, 100)
    elif c == ord('s'):
      stdscr.addstr(0, 0, 'Moving backward')
      send_cmd(pm, packer, -100, -100)
    elif c == ord(' '):
      stdscr.addstr(0, 0, 'Stopped')
      send_cmd(pm, packer, 0, 0)
    stdscr.refresh()

if __name__ == "__main__":
  wrapper(main)
