#!/usr/bin/env python3
import subprocess
import time
import unittest
from multiprocessing import Queue

from cereal import messaging
from selfdrive.manager.helpers import unblock_stdout
from tools.sim import bridge
from tools.sim.bridge import CarlaBridge


class TestCarlaIntegration(unittest.TestCase):
  """
  Tests need Carla simulator to run
  """
  processes = None

  def setUp(self):
    self.processes = []
    # We want to make sure that carla_sim docker is still running. Skip output shell
    subprocess.run("docker rm -f carla_sim", shell=True, stderr=subprocess.PIPE, check=False)

    self.processes.append(subprocess.Popen(".././start_carla.sh"))
    # Too many lagging messages in bridge.py can cause a crash. This prevents it.
    unblock_stdout()

  def test_run_bridge(self):
    # Test bridge connect with carla and runs without any errors for 60 seconds
    test_duration = 60

    carla_bridge = CarlaBridge(bridge.parse_args(['--low_quality']))
    p = carla_bridge.run(Queue(), retries=3)
    self.processes = [p]

    time.sleep(test_duration)

    self.assertEqual(p.exitcode, None, f"Bridge process should be running, but exited with code {p.exitcode}")

  def test_engage(self):
    # Startup manager and bridge.py. Check processes are running, then engage and verify.
    p_manager = subprocess.Popen("./launch_openpilot.sh", cwd='../')
    self.processes.append(p_manager)

    sm = messaging.SubMaster(['controlsState', 'carEvents', 'managerState'])
    q = Queue()
    carla_bridge = CarlaBridge(bridge.parse_args(['--low_quality']))
    p_bridge = carla_bridge.run(q, retries=3)
    self.processes.append(p_bridge)

    max_time_per_step = 20

    # Wait for bridge to startup
    start_waiting = time.monotonic()
    while not carla_bridge.started and time.monotonic() < start_waiting + max_time_per_step:
      time.sleep(0.1)
    self.assertEqual(p_bridge.exitcode, None, f"Bridge process should be running, but exited with code {p_bridge.exitcode}")

    start_time = time.monotonic()
    no_car_events_issues_once = False
    car_event_issues = []
    not_running = []
    while time.monotonic() < start_time + max_time_per_step:
      sm.update()

      not_running = [p.name for p in sm['managerState'].processes if not p.running and p.shouldBeRunning]
      car_event_issues = [event.name for event in sm['carEvents'] if any([event.noEntry, event.softDisable, event.immediateDisable])]

      if sm.all_alive() and len(car_event_issues) == 0 and len(not_running) == 0:
        no_car_events_issues_once = True
        break

    self.assertTrue(no_car_events_issues_once, f"Failed because sm offline, or CarEvents '{car_event_issues}' or processes not running '{not_running}'")

    start_time = time.monotonic()
    min_counts_control_active = 100
    control_active = 0

    while time.monotonic() < start_time + max_time_per_step:
      sm.update()

      q.put("cruise_up")  # Try engaging

      if sm.all_alive() and sm['controlsState'].active:
        control_active += 1

        if control_active == min_counts_control_active:
          break

    self.assertEqual(min_counts_control_active, control_active, f"Simulator did not engage a minimal of {min_counts_control_active} steps was {control_active}")

  def tearDown(self):
    print("Test shutting down. CommIssues are acceptable")
    for p in reversed(self.processes):
      p.terminate()

    for p in reversed(self.processes):
      if isinstance(p, subprocess.Popen):
        p.wait(15)
      else:
        p.join(15)
    subprocess.run("docker rm -f carla_sim", shell=True, stderr=subprocess.PIPE, check=False)



if __name__ == "__main__":
  unittest.main()
