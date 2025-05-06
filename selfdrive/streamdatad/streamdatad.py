#!/usr/bin/env python3
import socket
import msgpack
from time import monotonic
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log
from openpilot.system.version import get_version, get_commit, get_short_branch
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE

BUFFER_SIZE = 1024
UDP_PORT = 5006
TCP_PORT = 5007
params = Params()
dongleID = params.get("DongleId").decode("utf-8")
SM_UPDATE_INTERVAL = 33

def extract_model_data(data_dict):
  extracted_data = {key: data_dict.get(key) for key in ("position", "acceleration", "frameId")}
  list_keys = ("laneLines", "roadEdges")
  expected_size = len(extracted_data) + sum(len(data_dict.get(key, [])) for key in list_keys)
  for key in list_keys:
    if (value := data_dict.get(key)) and isinstance(value, list):
      for i, item in enumerate(value, 1):
        extracted_data[f"{key[:-1]}{i}"] = item
        if len(extracted_data) == expected_size:
          return extracted_data
  return extracted_data

def filter_keys(data_dict, keys_to_keep):
  result = {}
  for key in keys_to_keep:
    if key in data_dict:
      result[key] = data_dict[key]
      if len(result) == len(keys_to_keep):
        break
  return result

def safe_get(key, default_value='', bool_value=False):
  """
  Safely retrieves a parameter value while handling exceptions and type conversions.
  :param key: The parameter key to retrieve
  :param default_value: Default value to return in case of an exception
  :param bool_value: Whether to retrieve the value as a boolean
  :return: The retrieved value or the default value
  """
  try:
    if bool_value:
      return params.get_bool(key) or bool(default_value)
    value = params.get(key)
    if value is None:
      return default_value
    if isinstance(value, bytes):
      value = value.decode('utf-8')
    return value
  except Exception:
    return default_value

def safe_put_all(settings_to_put, non_bool_values=None):
  """
  Safely sets multiple parameter values from the settings dictionary.
  :param settings_to_put: The settings dictionary containing the values.
  :param non_bool_values: A set of param keys to be treated as non-boolean.
  """
  if non_bool_values is None:
    non_bool_values = set()
  try:
    for param_key, value in settings_to_put.items():
      params.put_bool(param_key, value) if param_key not in non_bool_values else params.put(param_key, str(value))
  except Exception as e:
    print(f"Error putting '{param_key}': {e}")

def deviceStatus(sm):
  if sm['peripheralState'].pandaType == log.PandaState.PandaType.unknown:
    return "error"
  # TODO: Add initialising if alerts.hasSevere from k_alerts
  return "ready"

def remainingDataUpload(sm):
  uploader_state = sm['uploaderState']
  return f"{uploader_state.immediateQueueSize + uploader_state.rawQueueSize} MB"

class Streamer:
  def __init__(self, sm=None):
    self.local_ip = "0.0.0.0"  # Bind to all network interfaces, allowing connections from any available network.
    self.ip = None
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm \
      else messaging.SubMaster(['modelV2', 'deviceState', 'peripheralState',\
      'controlsState', 'uploaderState', 'radarState', 'liveCalibration', 'carParams',\
      'carControl', 'driverStateV2', 'driverMonitoringState', 'carState', 'longitudinalPlan'])
    self.rk = Ratekeeper(20)  # Ratekeeper for 20 Hz loop
    self.last_periodic_time = 0  # Track last periodic task
    self.last_calibration_sent = 0
    self.setup_sockets()

  def check_calibration(self, is_offroad, cur_time):
    # Check calibration status and reset if engine on and calibration invalid
    if not is_offroad and self.sm['liveCalibration'].calStatus in \
      (log.LiveCalibrationData.Status.invalid, log.LiveCalibrationData.Status.uncalibrated) \
      and cur_time - self.last_calibration_sent > 0.1:
        # Reset calibration, retry every 0.1 seconds
        self.last_calibration_sent = cur_time
        params.remove("CalibrationParams")
        params.remove("LiveTorqueParameters")

  def setup_sockets(self):
    (udp_sock := self.udp_sock).bind(((local_ip := self.local_ip), UDP_PORT))
    udp_sock.setblocking(False)
    (tcp_sock := self.tcp_sock).setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    tcp_sock.bind((local_ip, TCP_PORT))
    tcp_sock.listen(1)
    tcp_sock.setblocking(False)

  def send_udp_message(self):
    if self.ip:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (data := extract_model_data(sm['modelV2'].to_dict())).update(filter_keys(sm['radarState'].to_dict(), ("leadOne", "leadTwo")))
      data.update(filter_keys(sm['liveCalibration'].to_dict(), ["height"]))
      data.update(filter_keys(sm['carParams'].to_dict(), "openpilotLongitudinalControl"))
      data.update(filter_keys(sm['carState'].to_dict(), ["vEgoCluster"]))
      data.update(sm['carControl'].to_dict())
      data.update(sm['deviceState'].to_dict())
      data.update(sm['driverStateV2'].to_dict())
      data.update(sm['controlsState'].to_dict())
      data.update(filter_keys(sm['driverMonitoringState'].to_dict(), ["isActiveMode", "events"]))
      data.update(filter_keys(sm['longitudinalPlan'].to_dict(), ["personality"]))

      # Pack and send
      try:
        self.udp_sock.sendto(msgpack.packb(data), (self.ip, UDP_PORT))
      except BlockingIOError:
        pass

  def send_tcp_message(self, is_offroad):
    if self.tcp_conn:
      try:
        self.sm.update(SM_UPDATE_INTERVAL)
        #(sm := self.sm).update(SM_UPDATE_INTERVAL)
        sett = {'isOffroad': is_offroad}
        sett['dongleID'] = dongleID
        sett['gitCommit'] = get_commit()[:7]
        sett['currentVersion'] = get_version()
        sett['osVersion'] = HARDWARE.get_os_version()
        sett['currentBranch'] = get_short_branch()
        #sett['connectivityStatus'] = str(sm['deviceState'].networkType)
        #sett['deviceStatus'] = deviceStatus(sm)
        #sett['remainingDataUpload'] = remainingDataUpload(sm)

        # Define the keys for each category
        bool_keys = [
          'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
          'LogVideoWifiOnly', 'IsMetric', 'SshEnabled', 'ExperimentalMode', 'RecordFront'
        ]

        string_keys = [
          'LongitudinalPersonality', 'HardwareSerial', 'FeaturesPackage', 'FixFingerprint'
        ]

        sett.update({key: safe_get(key, bool_value=True) for key in bool_keys})
        sett.update({key: safe_get(key) for key in string_keys})
        self.tcp_conn.sendall(msgpack.packb(sett))

      except socket.error:
        self.tcp_conn = None  # Reset connection on error

  def accept_new_connection(self):
    if not self.tcp_conn:
      try:
        self.tcp_conn, addr = self.tcp_sock.accept()
      except socket.error:
        pass

  def receive_udp_message(self):
    try:
      message, addr = self.udp_sock.recvfrom(BUFFER_SIZE)
      if message and dongleID in msgpack.unpackb(message): # Assume message only contains dongle ID list
        self.ip = addr[0]  # Update client IP
    except Exception:
      pass

  def receive_tcp_message(self, is_offroad):
    if self.tcp_conn:
      try:
        if (message := self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT)):
          try:
            settings = msgpack.unpackb(message)

            if dongleID in settings.pop('deviceList', []):
              if (msg_type := settings.pop('msgType', None)) == 'saveToggles' and is_offroad:
                safe_put_all(settings)
              elif msg_type == 'saveConfig':
                safe_put_all(settings, settings.keys())

          except Exception as e:
            print(f"\nError: {e}\nRaw TCP: {message}")
      except Exception:
        pass

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.send_udp_message()

      if (cur_time := monotonic()) - self.last_periodic_time >= 0.333: # 3 Hz
        self.receive_udp_message()
        self.last_periodic_time = cur_time
        self.accept_new_connection()
        self.receive_tcp_message(is_offroad := params.get_bool("IsOffroad"))
        self.send_tcp_message(is_offroad)
        self.check_calibration(is_offroad, cur_time)

      self.rk.keep_time()

  def close_connections(self):
    if self.tcp_conn:
      self.tcp_conn.close()
    self.udp_sock.close()

def main():
  streamer = Streamer()
  streamer.streamd_thread()

if __name__ == "__main__":
  main()
