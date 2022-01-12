#!/usr/bin/env python3
import datetime
import os
import time
from pathlib import Path
from typing import Dict, NoReturn, Optional, Tuple
from collections import namedtuple, OrderedDict

import psutil
from smbus2 import SMBus

import cereal.messaging as messaging
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.params import Params, ParamKeyType
from common.realtime import DT_TRML, sec_since_boot
from common.dict_helpers import strip_deprecated_keys
from selfdrive.controls.lib.alertmanager import set_offroad_alert
from selfdrive.controls.lib.pid import PIController
from selfdrive.hardware import EON, TICI, PC, HARDWARE
from selfdrive.loggerd.config import get_available_percent
from selfdrive.swaglog import cloudlog
from selfdrive.thermald.power_monitoring import PowerMonitoring
from selfdrive.version import terms_version, training_version
from selfdrive.statsd import statlog

ThermalStatus = log.DeviceState.ThermalStatus
NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength
CURRENT_TAU = 15.   # 15s time constant
TEMP_TAU = 5.   # 5s time constant
DISCONNECT_TIMEOUT = 5.  # wait 5 seconds before going offroad after disconnect so you get an alert

ThermalBand = namedtuple("ThermalBand", ['min_temp', 'max_temp'])

# List of thermal bands. We will stay within this region as long as we are within the bounds.
# When exiting the bounds, we'll jump to the lower or higher band. Bands are ordered in the dict.
THERMAL_BANDS = OrderedDict({
  ThermalStatus.green: ThermalBand(None, 80.0),
  ThermalStatus.yellow: ThermalBand(75.0, 96.0),
  ThermalStatus.red: ThermalBand(80.0, 107.),
  ThermalStatus.danger: ThermalBand(94.0, None),
})

# Override to highest thermal band when offroad and above this temp
OFFROAD_DANGER_TEMP = 79.5 if TICI else 70.0

prev_offroad_states: Dict[str, Tuple[bool, Optional[str]]] = {}

def read_tz(x):
  if x is None:
    return 0

  try:
    with open(f"/sys/devices/virtual/thermal/thermal_zone{x}/temp") as f:
      return int(f.read())
  except FileNotFoundError:
    return 0


def read_thermal(thermal_config):
  dat = messaging.new_message('deviceState')
  dat.deviceState.cpuTempC = [read_tz(z) / thermal_config.cpu[1] for z in thermal_config.cpu[0]]
  dat.deviceState.gpuTempC = [read_tz(z) / thermal_config.gpu[1] for z in thermal_config.gpu[0]]
  dat.deviceState.memoryTempC = read_tz(thermal_config.mem[0]) / thermal_config.mem[1]
  dat.deviceState.ambientTempC = read_tz(thermal_config.ambient[0]) / thermal_config.ambient[1]
  dat.deviceState.pmicTempC = [read_tz(z) / thermal_config.pmic[1] for z in thermal_config.pmic[0]]
  return dat


def setup_eon_fan():
  os.system("echo 2 > /sys/module/dwc3_msm/parameters/otg_switch")


last_eon_fan_val = None
def set_eon_fan(val):
  global last_eon_fan_val

  if last_eon_fan_val is None or last_eon_fan_val != val:
    bus = SMBus(7, force=True)
    try:
      i = [0x1, 0x3 | 0, 0x3 | 0x08, 0x3 | 0x10][val]
      bus.write_i2c_block_data(0x3d, 0, [i])
    except OSError:
      # tusb320
      if val == 0:
        bus.write_i2c_block_data(0x67, 0xa, [0])
      else:
        bus.write_i2c_block_data(0x67, 0xa, [0x20])
        bus.write_i2c_block_data(0x67, 0x8, [(val - 1) << 6])
    bus.close()
    last_eon_fan_val = val


# temp thresholds to control fan speed - high hysteresis
_TEMP_THRS_H = [50., 65., 80., 10000]
# temp thresholds to control fan speed - low hysteresis
_TEMP_THRS_L = [42.5, 57.5, 72.5, 10000]
# fan speed options
_FAN_SPEEDS = [0, 16384, 32768, 65535]


def handle_fan_eon(controller, max_cpu_temp, fan_speed, ignition):
  new_speed_h = next(speed for speed, temp_h in zip(_FAN_SPEEDS, _TEMP_THRS_H) if temp_h > max_cpu_temp)
  new_speed_l = next(speed for speed, temp_l in zip(_FAN_SPEEDS, _TEMP_THRS_L) if temp_l > max_cpu_temp)

  if new_speed_h > fan_speed:
    # update speed if using the high thresholds results in fan speed increment
    fan_speed = new_speed_h
  elif new_speed_l < fan_speed:
    # update speed if using the low thresholds results in fan speed decrement
    fan_speed = new_speed_l

  set_eon_fan(fan_speed // 16384)

  return fan_speed


def handle_fan_uno(controller, max_cpu_temp, fan_speed, ignition):
  new_speed = int(interp(max_cpu_temp, [40.0, 80.0], [0, 80]))

  if not ignition:
    new_speed = min(30, new_speed)

  return new_speed


last_ignition = False
def handle_fan_tici(controller, max_cpu_temp, fan_speed, ignition):
  global last_ignition

  controller.neg_limit = -(80 if ignition else 30)
  controller.pos_limit = -(30 if ignition else 0)

  if ignition != last_ignition:
    controller.reset()

  fan_pwr_out = -int(controller.update(
                     setpoint=75,
                     measurement=max_cpu_temp,
                     feedforward=interp(max_cpu_temp, [60.0, 100.0], [0, -80])
                  ))

  last_ignition = ignition
  return fan_pwr_out


def set_offroad_alert_if_changed(offroad_alert: str, show_alert: bool, extra_text: Optional[str]=None):
  if prev_offroad_states.get(offroad_alert, None) == (show_alert, extra_text):
    return
  prev_offroad_states[offroad_alert] = (show_alert, extra_text)
  set_offroad_alert(offroad_alert, show_alert, extra_text)


def thermald_thread() -> NoReturn:

  pm = messaging.PubMaster(['deviceState'])

  pandaState_timeout = int(1000 * 2.5 * DT_TRML)  # 2.5x the expected pandaState frequency
  pandaState_sock = messaging.sub_sock('pandaStates', timeout=pandaState_timeout)
  sm = messaging.SubMaster(["peripheralState", "gpsLocationExternal", "managerState", "controlsState"])

  fan_speed = 0
  count = 0

  onroad_conditions: Dict[str, bool] = {
    "ignition": False,
  }
  startup_conditions: Dict[str, bool] = {}
  startup_conditions_prev: Dict[str, bool] = {}

  off_ts = None
  started_ts = None
  started_seen = False
  thermal_status = ThermalStatus.green
  usb_power = True

  network_type = NetworkType.none
  network_strength = NetworkStrength.unknown
  network_info = None
  modem_version = None
  registered_count = 0
  nvme_temps = None
  modem_temps = None

  current_filter = FirstOrderFilter(0., CURRENT_TAU, DT_TRML)
  temp_filter = FirstOrderFilter(0., TEMP_TAU, DT_TRML)
  pandaState_prev = None
  should_start_prev = False
  in_car = False
  handle_fan = None
  is_uno = False
  ui_running_prev = False
  engaged_prev = False

  params = Params()
  power_monitor = PowerMonitoring()
  no_panda_cnt = 0

  HARDWARE.initialize_hardware()
  thermal_config = HARDWARE.get_thermal_config()

  # TODO: use PI controller for UNO
  controller = PIController(k_p=0, k_i=2e-3, neg_limit=-80, pos_limit=0, rate=(1 / DT_TRML))

  while True:
    pandaStates = messaging.recv_sock(pandaState_sock, wait=True)

    sm.update(0)
    peripheralState = sm['peripheralState']

    msg = read_thermal(thermal_config)

    if pandaStates is not None and len(pandaStates.pandaStates) > 0:
      pandaState = pandaStates.pandaStates[0]

      # If we lose connection to the panda, wait 5 seconds before going offroad
      if pandaState.pandaType == log.PandaState.PandaType.unknown:
        no_panda_cnt += 1
        if no_panda_cnt > DISCONNECT_TIMEOUT / DT_TRML:
          if onroad_conditions["ignition"]:
            cloudlog.error("Lost panda connection while onroad")
          onroad_conditions["ignition"] = False
      else:
        no_panda_cnt = 0
        onroad_conditions["ignition"] = pandaState.ignitionLine or pandaState.ignitionCan

      in_car = pandaState.harnessStatus != log.PandaState.HarnessStatus.notConnected
      usb_power = peripheralState.usbPowerMode != log.PeripheralState.UsbPowerMode.client

      # Setup fan handler on first connect to panda
      if handle_fan is None and peripheralState.pandaType != log.PandaState.PandaType.unknown:
        is_uno = peripheralState.pandaType == log.PandaState.PandaType.uno

        if TICI:
          cloudlog.info("Setting up TICI fan handler")
          handle_fan = handle_fan_tici
        elif is_uno or PC:
          cloudlog.info("Setting up UNO fan handler")
          handle_fan = handle_fan_uno
        else:
          cloudlog.info("Setting up EON fan handler")
          setup_eon_fan()
          handle_fan = handle_fan_eon

      # Handle disconnect
      if pandaState_prev is not None:
        if pandaState.pandaType == log.PandaState.PandaType.unknown and \
          pandaState_prev.pandaType != log.PandaState.PandaType.unknown:
          params.clear_all(ParamKeyType.CLEAR_ON_PANDA_DISCONNECT)
      pandaState_prev = pandaState

    # these are expensive calls. update every 10s
    if (count % int(10. / DT_TRML)) == 0:
      try:
        network_type = HARDWARE.get_network_type()
        network_strength = HARDWARE.get_network_strength(network_type)
        network_info = HARDWARE.get_network_info()  # pylint: disable=assignment-from-none
        nvme_temps = HARDWARE.get_nvme_temperatures()
        modem_temps = HARDWARE.get_modem_temperatures()

        # Log modem version once
        if modem_version is None:
          modem_version = HARDWARE.get_modem_version()  # pylint: disable=assignment-from-none
          if modem_version is not None:
            cloudlog.warning(f"Modem version: {modem_version}")

        if TICI and (network_info.get('state', None) == "REGISTERED"):
          registered_count += 1
        else:
          registered_count = 0

        if registered_count > 10:
          cloudlog.warning(f"Modem stuck in registered state {network_info}. nmcli conn up lte")
          os.system("nmcli conn up lte")
          registered_count = 0

      except Exception:
        cloudlog.exception("Error getting network status")

    msg.deviceState.freeSpacePercent = get_available_percent(default=100.0)
    msg.deviceState.memoryUsagePercent = int(round(psutil.virtual_memory().percent))
    msg.deviceState.cpuUsagePercent = [int(round(n)) for n in psutil.cpu_percent(percpu=True)]
    msg.deviceState.gpuUsagePercent = int(round(HARDWARE.get_gpu_usage_percent()))
    msg.deviceState.networkType = network_type
    msg.deviceState.networkStrength = network_strength
    if network_info is not None:
      msg.deviceState.networkInfo = network_info
    if nvme_temps is not None:
      msg.deviceState.nvmeTempC = nvme_temps
      for i, temp in enumerate(nvme_temps):
        statlog.gauge(f"nvme_temperature{i}", temp)
    if modem_temps is not None:
      msg.deviceState.modemTempC = modem_temps
      for i, temp in enumerate(modem_temps):
        statlog.gauge(f"modem_temperature{i}", temp)

    msg.deviceState.screenBrightnessPercent = HARDWARE.get_screen_brightness()
    msg.deviceState.batteryPercent = HARDWARE.get_battery_capacity()
    msg.deviceState.batteryCurrent = HARDWARE.get_battery_current()
    msg.deviceState.usbOnline = HARDWARE.get_usb_present()
    current_filter.update(msg.deviceState.batteryCurrent / 1e6)

    max_comp_temp = temp_filter.update(
      max(max(msg.deviceState.cpuTempC), msg.deviceState.memoryTempC, max(msg.deviceState.gpuTempC))
    )

    if handle_fan is not None:
      fan_speed = handle_fan(controller, max_comp_temp, fan_speed, onroad_conditions["ignition"])
      msg.deviceState.fanSpeedPercentDesired = fan_speed

    is_offroad_for_5_min = (started_ts is None) and ((not started_seen) or (off_ts is None) or (sec_since_boot() - off_ts > 60 * 5))
    if is_offroad_for_5_min and max_comp_temp > OFFROAD_DANGER_TEMP:
      # If device is offroad we want to cool down before going onroad
      # since going onroad increases load and can make temps go over 107
      thermal_status = ThermalStatus.danger
    else:
      current_band = THERMAL_BANDS[thermal_status]
      band_idx = list(THERMAL_BANDS.keys()).index(thermal_status)
      if current_band.min_temp is not None and max_comp_temp < current_band.min_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx - 1]
      elif current_band.max_temp is not None and max_comp_temp > current_band.max_temp:
        thermal_status = list(THERMAL_BANDS.keys())[band_idx + 1]

    # **** starting logic ****

    # Ensure date/time are valid
    now = datetime.datetime.utcnow()
    startup_conditions["time_valid"] = (now.year > 2020) or (now.year == 2020 and now.month >= 10)
    set_offroad_alert_if_changed("Offroad_InvalidTime", (not startup_conditions["time_valid"]))

    startup_conditions["up_to_date"] = params.get("Offroad_ConnectivityNeeded") is None or params.get_bool("DisableUpdates") or params.get_bool("SnoozeUpdate")
    startup_conditions["not_uninstalling"] = not params.get_bool("DoUninstall")
    startup_conditions["accepted_terms"] = params.get("HasAcceptedTerms") == terms_version

    # with 2% left, we killall, otherwise the phone will take a long time to boot
    startup_conditions["free_space"] = msg.deviceState.freeSpacePercent > 2
    startup_conditions["completed_training"] = params.get("CompletedTrainingVersion") == training_version or \
                                               params.get_bool("Passive")
    startup_conditions["not_driver_view"] = not params.get_bool("IsDriverViewEnabled")
    startup_conditions["not_taking_snapshot"] = not params.get_bool("IsTakingSnapshot")
    # if any CPU gets above 107 or the battery gets above 63, kill all processes
    # controls will warn with CPU above 95 or battery above 60
    onroad_conditions["device_temp_good"] = thermal_status < ThermalStatus.danger
    set_offroad_alert_if_changed("Offroad_TemperatureTooHigh", (not onroad_conditions["device_temp_good"]))

    if TICI:
      set_offroad_alert_if_changed("Offroad_StorageMissing", (not Path("/data/media").is_mount()))

    # Handle offroad/onroad transition
    should_start = all(onroad_conditions.values())
    if started_ts is None:
      should_start = should_start and all(startup_conditions.values())

    if should_start != should_start_prev or (count == 0):
      params.put_bool("IsOnroad", should_start)
      params.put_bool("IsOffroad", not should_start)

      params.put_bool("IsEngaged", False)
      engaged_prev = False
      HARDWARE.set_power_save(not should_start)

    if sm.updated['controlsState']:
      engaged = sm['controlsState'].enabled
      if engaged != engaged_prev:
        params.put_bool("IsEngaged", engaged)
        engaged_prev = engaged

      try:
        with open('/dev/kmsg', 'w') as kmsg:
          kmsg.write(f"[thermald] engaged: {engaged}")
      except Exception:
        pass

    if should_start:
      off_ts = None
      if started_ts is None:
        started_ts = sec_since_boot()
        started_seen = True
    else:
      if onroad_conditions["ignition"] and (startup_conditions != startup_conditions_prev):
        cloudlog.event("Startup blocked", startup_conditions=startup_conditions, onroad_conditions=onroad_conditions)

      started_ts = None
      if off_ts is None:
        off_ts = sec_since_boot()

    # Offroad power monitoring
    power_monitor.calculate(peripheralState, onroad_conditions["ignition"])
    msg.deviceState.offroadPowerUsageUwh = power_monitor.get_power_used()
    msg.deviceState.carBatteryCapacityUwh = max(0, power_monitor.get_car_battery_capacity())
    current_power_draw = HARDWARE.get_current_power_draw()  # pylint: disable=assignment-from-none
    msg.deviceState.powerDrawW = current_power_draw if current_power_draw is not None else 0

    # Check if we need to disable charging (handled by boardd)
    msg.deviceState.chargingDisabled = power_monitor.should_disable_charging(onroad_conditions["ignition"], in_car, off_ts)

    # Check if we need to shut down
    if power_monitor.should_shutdown(peripheralState, onroad_conditions["ignition"], in_car, off_ts, started_seen):
      cloudlog.info(f"shutting device down, offroad since {off_ts}")
      # TODO: add function for blocking cloudlog instead of sleep
      time.sleep(10)
      HARDWARE.shutdown()

    # If UI has crashed, set the brightness to reasonable non-zero value
    ui_running = "ui" in (p.name for p in sm["managerState"].processes if p.running)
    if ui_running_prev and not ui_running:
      HARDWARE.set_screen_brightness(20)
    ui_running_prev = ui_running

    msg.deviceState.chargingError = current_filter.x > 0. and msg.deviceState.batteryPercent < 90  # if current is positive, then battery is being discharged
    msg.deviceState.started = started_ts is not None
    msg.deviceState.startedMonoTime = int(1e9*(started_ts or 0))

    last_ping = params.get("LastAthenaPingTime")
    if last_ping is not None:
      msg.deviceState.lastAthenaPingTime = int(last_ping)

    msg.deviceState.thermalStatus = thermal_status
    pm.send("deviceState", msg)

    if EON and not is_uno:
      set_offroad_alert_if_changed("Offroad_ChargeDisabled", (not usb_power))

    should_start_prev = should_start
    startup_conditions_prev = startup_conditions.copy()

    # log more stats
    statlog.gauge("free_space_percent", msg.deviceState.freeSpacePercent)
    statlog.gauge("gpu_usage_percent", msg.deviceState.gpuUsagePercent)
    statlog.gauge("memory_usage_percent", msg.deviceState.memoryUsagePercent)
    for i, usage in enumerate(msg.deviceState.cpuUsagePercent):
      statlog.gauge(f"cpu{i}_usage_percent", usage)
    for i, temp in enumerate(msg.deviceState.cpuTempC):
      statlog.gauge(f"cpu{i}_temperature", temp)
    for i, temp in enumerate(msg.deviceState.gpuTempC):
      statlog.gauge(f"gpu{i}_temperature", temp)
    statlog.gauge("memory_temperature", msg.deviceState.memoryTempC)
    statlog.gauge("ambient_temperature", msg.deviceState.ambientTempC)
    for i, temp in enumerate(msg.deviceState.pmicTempC):
      statlog.gauge(f"pmic{i}_temperature", temp)
    statlog.gauge("fan_speed_percent_desired", msg.deviceState.fanSpeedPercentDesired)
    statlog.gauge("screen_brightness_percent", msg.deviceState.screenBrightnessPercent)

    # report to server once every 10 minutes
    if (count % int(600. / DT_TRML)) == 0:
      if EON and started_ts is None and msg.deviceState.memoryUsagePercent > 40:
        cloudlog.event("High offroad memory usage", mem=msg.deviceState.memoryUsagePercent)

      cloudlog.event("STATUS_PACKET",
                     count=count,
                     pandaStates=(strip_deprecated_keys(pandaStates.to_dict()) if pandaStates else None),
                     peripheralState=strip_deprecated_keys(peripheralState.to_dict()),
                     location=(strip_deprecated_keys(sm["gpsLocationExternal"].to_dict()) if sm.alive["gpsLocationExternal"] else None),
                     deviceState=strip_deprecated_keys(msg.to_dict()))

    count += 1


def main() -> NoReturn:
  thermald_thread()


if __name__ == "__main__":
  main()
