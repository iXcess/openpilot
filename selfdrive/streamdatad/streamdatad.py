#!/usr/bin/env python3

import socket
import msgpack
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log, car
from openpilot.system.version import get_version, get_commit, get_short_branch
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE

BUFFER_SIZE = 1024
UDP_PORT = 5006
TCP_PORT = 5007
params = Params()

def extract_model_data(model_dict):
  # Extract 'position' and 'acceleration' directly
  extracted_data = {"position": model_dict.get("position"),"acceleration": model_dict.get("acceleration") }

  # Flatten laneLines and roadEdges efficiently with single lookup
  for key in ("laneLines", "roadEdges"):
    value = model_dict.get(key)
    if isinstance(value, list):
      prefix = key[:-1]
      extracted_data.update((f"{prefix}{i}", item) for i, item in enumerate(value, 1))

  return extracted_data

def filter_keys(model_dict, keys_to_keep):
  result = {}
  for key in keys_to_keep:
    if key in model_dict:
      result[key] = model_dict[key]
      if len(result) == len(keys_to_keep):
        break
  return result

def safe_get(key, default_value='', decode_utf8=False, to_float=False, bool_value=False):
  """
  Safely retrieves a parameter value while handling exceptions and type conversions.
  :param params: The params object
  :param key: The parameter key to retrieve
  :param default_value: Default value to return in case of an exception
  :param decode_utf8: Whether to decode the retrieved value as UTF-8
  :param to_float: Whether to convert the value to float
  :param bool_value: Whether to retrieve the value as boolean
  :return: The retrieved value or the default value
  """
  try:
    if bool_value:
      return params.get_bool(key)  # Get boolean value
    value = params.get(key) or default_value  # Get the value or default
    if decode_utf8 and value:
      return value.decode('utf-8')  # Decode UTF-8 if needed
    if to_float and value:
      return float(value)  # Convert to float if needed
    return value  # Return the retrieved or default value
  except Exception as e:
    print(f"Exception occurred while retrieving key '{key}': {e}")
    return default_value  # Return the default value in case of an exception

def safe_put_all(settings_to_put, mapping, non_bool_values=None):
  """
  Safely sets multiple parameter values from the settings dictionary.
  :param settings: The settings dictionary containing the values.
  :param mapping: A dictionary mapping param keys to their respective settings keys.
  :param non_bool_values: A set of param keys to be treated as non-boolean.
  """
  if non_bool_values is None:
    non_bool_values = set()  # Default to an empty set if not provided

  for param_key, settings_key in mapping.items():
    try:
      value = settings_to_put[settings_key]  # Retrieve the value from the settings dictionary
      if param_key in non_bool_values:
        params.put(param_key, str(value))  # Convert the value to a string and set it
      else:
        if not isinstance(value, bool):
          continue  # Skip if the value is expected to be boolean but isn't
        params.put_bool(param_key, value)  # Set the value as boolean
    except KeyError:
      # Skip if the settings key does not exist
      pass
    except Exception as e:
      print(f"Exception occurred while setting param '{param_key}' with value from '{settings_key}': {e}")

def deviceStatus(sm):
  if sm['peripheralState'].pandaType == log.PandaState.PandaType.unknown:
    return "error"
  else: # TODO: Add initialising if alerts.hasSevere from k_alerts
    return "ready"

def remainingDataUpload(sm):
  uploader_state = sm['uploaderState']
  immediate_queue_size = uploader_state.immediateQueueSize
  raw_queue_size = uploader_state.rawQueueSize
  return f"{immediate_queue_size + raw_queue_size} MB"

class Streamer:
  def __init__(self, sm=None):
    #self.local_ip = "192.168.100.1"
    self.local_ip = "0.0.0.0"  # Bind to all network interfaces, allowing connections from any available network.
    self.ip = None
    self.requestInfo = False
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm \
      else messaging.SubMaster(['modelV2', 'deviceState', 'peripheralState',\
      'controlsState', 'uploaderState', 'radarState', 'liveCalibration', 'carParams',\
      'carControl', 'driverStateV2', 'driverMonitoringState', 'carState', 'longitudinalPlan'])
    self.rk = Ratekeeper(10)  # Ratekeeper for 10 Hz loop

    self.setup_sockets()

  def setup_sockets(self):
    self.udp_sock.bind((self.local_ip, UDP_PORT))
    self.udp_sock.setblocking(False)
    self.tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    self.tcp_sock.bind((self.local_ip, TCP_PORT))
    self.tcp_sock.listen(1)
    self.tcp_sock.setblocking(False)

  def send_udp_message(self):
    if self.ip:
      (sm := self.sm).update(10) # update every 10 ms

      data = extract_model_data(sm['modelV2'].to_dict())
      data.update(filter_keys(sm['radarState'].to_dict(), ("leadOne", "leadTwo")))
      data.update(filter_keys(sm['liveCalibration'].to_dict(), ["height"]))
      data.update(filter_keys(sm['carParams'].to_dict(), "openpilotLongitudinalControl"))
      data.update(filter_keys(sm['carState'].to_dict(), ["vEgoCluster"]))
      data.update(sm['carControl'].to_dict())
      data.update(sm['deviceState'].to_dict())
      data.update(sm['driverStateV2'].to_dict())
      data.update(sm['controlsState'].to_dict())
      data.update(filter_keys(sm['driverMonitoringState'].to_dict(), ["isActiveMode"]))
      data.update(filter_keys(sm['longitudinalPlan'].to_dict(), ["personality"]))
      data.update(dict(car.CarEvent.EventName.schema.enumerants.items()))

      # Pack and send
      message = msgpack.packb(data)
      try:
        self.udp_sock.sendto(message, (self.ip, UDP_PORT))
      except BlockingIOError:
        pass

  def send_tcp_message(self):
    if self.tcp_conn:
      try:
        (sm := self.sm).update(10)
        sett = {}
        sett['connectivityStatus'] = str(sm['deviceState'].networkType)
        sett['deviceStatus'] = deviceStatus(sm)
        controlsState = sm['controlsState']
        sett['alerts'] = f"{controlsState.alertText1}\n{controlsState.alertText2}"
        sett['remainingDataUpload'] = remainingDataUpload(sm)
        # TODO send uploadStatus in selfdrive/loggerd/uploader.py
        sett['gitCommit'] = get_commit()[:7]
        # TODO include bukapilot changes in selfdrive/updated.py
        sett['updateStatus'] = safe_get("UpdaterState")

        sett['isOffroad'] = safe_get("IsOffroad", bool_value=True)
        sett['enableBukapilot'] = safe_get("OpenpilotEnabledToggle", bool_value=True)
        sett['quietMode'] = safe_get("QuietMode", bool_value=True)
        sett['enableAssistedLaneChange'] = safe_get("IsAlcEnabled", bool_value=True)
        sett['enableLaneDepartureWarning'] = safe_get("IsLdwEnabled", bool_value=True)
        sett['uploadVideoWiFiOnly'] = safe_get("LogVideoWifiOnly", bool_value=True)
        sett['apn'] = safe_get("GsmApn")
        sett['enableRoaming'] = safe_get("GsmRoaming", bool_value=True)
        sett['driverPersonality'] = safe_get("LongitudinalPersonality", decode_utf8=True)
        sett['useMetricSystem'] = safe_get("IsMetric", bool_value=True)
        sett['enableSSH'] = safe_get("SshEnabled", bool_value=True)
        sett['experimentalModel'] = safe_get("ExperimentalMode", bool_value=True)
        sett['recordUploadDriverCamera'] = safe_get("RecordFront", bool_value=True)
        sett['featurePackage'] = safe_get("FeaturesPackage")
        sett['fixFingerprint'] = safe_get("FixFingerprint")

        if self.requestInfo:
          sett['requestDeviceInfo'] = True
          sett['dongleID'] = safe_get("DongleId", decode_utf8=True)
          sett['serial'] = safe_get("HardwareSerial", decode_utf8=True)
          sett['hostname'] = socket.gethostname()
          sett['currentVersion'] = get_version()
          sett['osVersion'] = HARDWARE.get_os_version()
          sett['currentBranch'] = get_short_branch()
          sett['currentChangelog'] = safe_get("UpdaterCurrentReleaseNotes")
          self.requestInfo = False

        self.tcp_conn.sendall(msgpack.packb(sett))

      except socket.error:
        self.tcp_conn = None  # Reset connection on error

  def accept_new_connection(self):
    if not self.tcp_conn:
      try:
        self.tcp_conn, addr = self.tcp_sock.accept()
        self.ip = addr[0]  # Update client IP for UDP messages
      except socket.error:
        pass

  def receive_udp_message(self):
    try:
      message, addr = self.udp_sock.recvfrom(BUFFER_SIZE)
      self.ip = addr[0]  # Update client IP
    except Exception:
      pass

  def receive_tcp_message(self):
    if self.tcp_conn:
      try:
        message = self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT)
        if message:
          try:
            settings = msgpack.unpackb(message)
            offroad = params.get_bool("IsOffroad")
            self.requestInfo = settings['requestDeviceInfo']

            if offroad and not self.requestInfo:
              # Set values
              # print("\nPutting parameters")
              mapping={
                "OpenpilotEnabledToggle":"enableBukapilot",
                "QuietMode":"quietMode",
                "IsAlcEnabled":"enableAssistedLaneChange",
                "IsLdwEnabled":"enableLaneDepartureWarning",
                "LogVideoWifiOnly":"uploadVideoWiFiOnly",
                "GsmRoaming":"enableRoaming",
                "IsMetric":"useMetricSystem",
                "SshEnabled":"enableSSH",
                "ExperimentalMode":"experimentalModel",
                "RecordFront":"recordUploadDriverCamera",
              }

              # non_bool_values = {}
              safe_put_all(settings, mapping)

          except Exception as e:
            print(f"\nError: {e}\nRaw TCP: {message}")
      except Exception:
        pass

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.receive_udp_message()
      self.send_udp_message()
      self.accept_new_connection()
      self.receive_tcp_message()
      self.send_tcp_message()
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

