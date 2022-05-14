#!/usr/bin/env python3
import os
import json
import random
import unittest
import time
import capnp
from cffi import FFI

from cereal import log
import cereal.messaging as messaging
from cereal.services import service_list
from common.params import Params

from selfdrive.manager.process_config import managed_processes

SENSOR_DECIMATION = 1
VISION_DECIMATION = 1

LIBLOCATIOND_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '../liblocationd.so'))


class TestLocationdLib(unittest.TestCase):
  def setUp(self):
    header = '''typedef ...* Localizer_t;
Localizer_t localizer_init();
void localizer_get_message_bytes(Localizer_t localizer, bool inputsOK, bool sensorsOK, bool gpsOK, bool msgValid, char *buff, size_t buff_size);
void localizer_handle_msg_bytes(Localizer_t localizer, const char *data, size_t size);'''

    self.ffi = FFI()
    self.ffi.cdef(header)
    self.lib = self.ffi.dlopen(LIBLOCATIOND_PATH)

    self.localizer = self.lib.localizer_init()

    self.buff_size = 2048
    self.msg_buff = self.ffi.new(f'char[{self.buff_size}]')

  def localizer_handle_msg(self, msg_builder):
    bytstr = msg_builder.to_bytes()
    self.lib.localizer_handle_msg_bytes(self.localizer, self.ffi.from_buffer(bytstr), len(bytstr))

  def localizer_get_msg(self, t=0, inputsOK=True, sensorsOK=True, gpsOK=True, msgValid=True):
    self.lib.localizer_get_message_bytes(self.localizer, inputsOK, sensorsOK, gpsOK, msgValid, self.ffi.addressof(self.msg_buff, 0), self.buff_size)
    return log.Event.from_bytes(self.ffi.buffer(self.msg_buff), nesting_limit=self.buff_size // 8)

  def test_liblocalizer(self):
    msg = messaging.new_message('liveCalibration')
    msg.liveCalibration.validBlocks = random.randint(1, 10)
    msg.liveCalibration.rpyCalib = [random.random() / 10 for _ in range(3)]

    self.localizer_handle_msg(msg)
    liveloc = self.localizer_get_msg()
    self.assertTrue(liveloc is not None)

  @unittest.skip("temporarily disabled due to false positives")
  def test_device_fell(self):
    msg = messaging.new_message('sensorEvents', 1)
    msg.sensorEvents[0].sensor = 1
    msg.sensorEvents[0].timestamp = msg.logMonoTime
    msg.sensorEvents[0].type = 1
    msg.sensorEvents[0].init('acceleration')
    msg.sensorEvents[0].acceleration.v = [10.0, 0.0, 0.0]  # zero with gravity
    self.localizer_handle_msg(msg)

    ret = self.localizer_get_msg()
    self.assertTrue(ret.liveLocationKalman.deviceStable)

    msg = messaging.new_message('sensorEvents', 1)
    msg.sensorEvents[0].sensor = 1
    msg.sensorEvents[0].timestamp = msg.logMonoTime
    msg.sensorEvents[0].type = 1
    msg.sensorEvents[0].init('acceleration')
    msg.sensorEvents[0].acceleration.v = [50.1, 0.0, 0.0]  # more than 40 m/s**2
    self.localizer_handle_msg(msg)

    ret = self.localizer_get_msg()
    self.assertFalse(ret.liveLocationKalman.deviceStable)

  def test_posenet_spike(self):
    for _ in range(SENSOR_DECIMATION):
      msg = messaging.new_message('carState')
      msg.carState.vEgo = 6.0  # more than 5 m/s
      self.localizer_handle_msg(msg)

    ret = self.localizer_get_msg()
    self.assertTrue(ret.liveLocationKalman.posenetOK)

    for _ in range(20 * VISION_DECIMATION):  # size of hist_old
      msg = messaging.new_message('cameraOdometry')
      msg.cameraOdometry.rot = [0.0, 0.0, 0.0]
      msg.cameraOdometry.rotStd = [0.1, 0.1, 0.1]
      msg.cameraOdometry.trans = [0.0, 0.0, 0.0]
      msg.cameraOdometry.transStd = [2.0, 0.1, 0.1]
      self.localizer_handle_msg(msg)

    for _ in range(20 * VISION_DECIMATION):  # size of hist_new
      msg = messaging.new_message('cameraOdometry')
      msg.cameraOdometry.rot = [0.0, 0.0, 0.0]
      msg.cameraOdometry.rotStd = [1.0, 1.0, 1.0]
      msg.cameraOdometry.trans = [0.0, 0.0, 0.0]
      msg.cameraOdometry.transStd = [10.1, 0.1, 0.1]  # more than 4 times larger
      self.localizer_handle_msg(msg)

    ret = self.localizer_get_msg()
    self.assertFalse(ret.liveLocationKalman.posenetOK)


class TestLocationdProc(unittest.TestCase):
  MAX_WAITS = 1000
  LLD_MSGS = ['gpsLocationExternal', 'cameraOdometry', 'carState', 'sensorEvents', 'liveCalibration']

  def setUp(self):
    random.seed(123489234)

    self.pm = messaging.PubMaster(self.LLD_MSGS)

    managed_processes['locationd'].prepare()
    managed_processes['locationd'].start()

    time.sleep(1)

  def tearDown(self):
    managed_processes['locationd'].stop()

  def send_msg(self, msg):
    self.pm.send(msg.which(), msg)
    waits_left = self.MAX_WAITS
    while waits_left and not self.pm.all_readers_updated(msg.which()):
      time.sleep(0)
      waits_left -= 1
    time.sleep(0.0001)

  def get_fake_msg(self, name, t):
    try:
      msg = messaging.new_message(name)
    except capnp.lib.capnp.KjException:
      msg = messaging.new_message(name, 0)

    if name == "gpsLocationExternal":
      msg.gpsLocationExternal.flags = 1
      msg.gpsLocationExternal.verticalAccuracy = 1.0
      msg.gpsLocationExternal.speedAccuracy = 1.0
      msg.gpsLocationExternal.bearingAccuracyDeg = 1.0
      msg.gpsLocationExternal.vNED = [0.0, 0.0, 0.0]
      msg.gpsLocationExternal.latitude = self.lat
      msg.gpsLocationExternal.longitude = self.lon
      msg.gpsLocationExternal.altitude = self.alt
    elif name == 'cameraOdometry':
      msg.cameraOdometry.rot = [0.0, 0.0, 0.0]
      msg.cameraOdometry.rotStd = [0.0, 0.0, 0.0]
      msg.cameraOdometry.trans = [0.0, 0.0, 0.0]
      msg.cameraOdometry.transStd = [0.0, 0.0, 0.0]
    msg.logMonoTime = t
    return msg

  def test_params_gps(self):
    # first reset params
    Params().delete('LastGPSPosition')

    self.lat = 30 + (random.random() * 10.0)
    self.lon = -70 + (random.random() * 10.0)
    self.alt = 5 + (random.random() * 10.0)
    self.fake_duration = 90  # secs
    # get fake messages at the correct frequency, listed in services.py
    fake_msgs = []
    for sec in range(self.fake_duration):
      for name in self.LLD_MSGS:
        for j in range(int(service_list[name].frequency)):
          fake_msgs.append(self.get_fake_msg(name, int((sec + j / service_list[name].frequency) * 1e9)))

    for fake_msg in sorted(fake_msgs, key=lambda x: x.logMonoTime):
      self.send_msg(fake_msg)
    time.sleep(1)  # wait for async params write

    lastGPS = json.loads(Params().get('LastGPSPosition'))

    self.assertAlmostEqual(lastGPS['latitude'], self.lat, places=3)
    self.assertAlmostEqual(lastGPS['longitude'], self.lon, places=3)
    self.assertAlmostEqual(lastGPS['altitude'], self.alt, places=3)


if __name__ == "__main__":
  unittest.main()
