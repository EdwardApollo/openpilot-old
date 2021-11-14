import sys
from curses import wrapper
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp  # pylint: disable=no-name-in-module,import-error

def send_cmd(pm, packer, speed_left, speed_right):
  msg = packer.make_can_msg("TANK_COMMAND", 0,
    {"SPEED_LEFT": int(speed_left),
     "SPEED_RIGHT": int(speed_right)})
  pm.send('sendcan', can_list_to_can_capnp([msg], msgtype='sendcan'))


# Manual control
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


# Automatic control
def modelcontrol():
  packer = CANPacker("comma_tank")
  pm = messaging.PubMaster(['sendcan'])
  sm = messaging.SubMaster(['driverState'])
  running_bump_prob = 0

  try:
    while True:
      sm.update()
      bump_prob = sm['driverState'].occludedProb
      running_bump_prob = .8 * running_bump_prob + .2 * bump_prob
      print(bump_prob, running_bump_prob)

      if bump_prob > .5:
        send_cmd(pm, packer, 100, -100)
      else:
        send_cmd(pm, packer, 100, 100)
  except KeyboardInterrupt:
    send_cmd(pm, packer, 0, 0)
    raise


# Entry point
if __name__ == "__main__":
  if sys.argv[1] == 'manual':
    wrapper(main)
  elif sys.argv[1] == 'auto':
    modelcontrol()
  else:
    raise Exception("Please select either 'manual' or 'auto' mode")
