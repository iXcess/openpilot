#!/usr/bin/env python3
import socket
import msgpack
from time import monotonic
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log
from openpilot.system.version import get_version, get_commit
# from openpilot.system.version import get_short_branch
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

def safe_get(key, is_bool=False):
  try:
    return params.get_bool(key) if is_bool else params.get(key).decode('utf-8')
  except Exception:
    return False if is_bool else ''

def safe_put_all(settings_to_put, is_bool=False):
  try:
    for param_key, value in settings_to_put.items():
      if is_bool:
        params.put_bool(param_key, value)
      else:
        params.put(param_key, str(value))
  except Exception as e:
    print(f"Error putting parameter: {e}")

def deviceStatus(sm):
  if sm['peripheralState'].pandaType == log.PandaState.PandaType.unknown:
    return "error"
  # TODO: Add initialising if alerts.hasSevere from k_alerts
  return "ready"

def remainingDataUpload(sm):
  uploader_state = sm['uploaderState']
  return f"{uploader_state.immediateQueueSize + uploader_state.rawQueueSize} MB"

def reset_calibration():
  params.remove("CalibrationParams")
  params.remove("LiveTorqueParameters")

def do_reboot(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.put_bool("DoReboot", True)

def update_dict_from_sm(target_dict, sm_subset, key):
  target_dict[key] = str(getattr(sm_subset, key))

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
    self.setup_sockets()

  def setup_sockets(self):
    (udp_sock := self.udp_sock).bind(((local_ip := self.local_ip), UDP_PORT))
    udp_sock.setblocking(False)
    (tcp_sock := self.tcp_sock).setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    tcp_sock.bind((local_ip, TCP_PORT))
    tcp_sock.listen(1)
    tcp_sock.setblocking(False)

  def send_udp_message(self, is_metric):
    if self.ip:
      (data := extract_model_data((sm := self.sm)['modelV2'].to_dict())).update(sm['carControl'].to_dict())
      if is_metric is not None:
        data["IsMetric"] = is_metric
      data['dongleID'] = dongleID
      data.update(sm['deviceState'].to_dict())
      data.update(sm['driverStateV2'].to_dict())
      data.update(sm['controlsState'].to_dict())
      update_dict_from_sm(data, sm['radarState'], "leadOne")
      update_dict_from_sm(data, sm['radarState'], "leadTwo")
      update_dict_from_sm(data, sm['driverMonitoringState'], "isActiveMode")
      update_dict_from_sm(data, sm['driverMonitoringState'], "events")
      update_dict_from_sm(data, sm['liveCalibration'], "height")
      update_dict_from_sm(data, sm['carParams'], "openpilotLongitudinalControl")
      update_dict_from_sm(data, sm['carState'], "vEgoCluster")
      update_dict_from_sm(data, sm['longitudinalPlan'], "personality")

      # Pack and send
      try:
        self.udp_sock.sendto(msgpack.packb(data), (self.ip, UDP_PORT))
      except (BlockingIOError, OSError):
        pass
      except Exception as e:
        print(f"Unexpected error while sending UDP message: {e}")

  def send_tcp_message(self, is_offroad, state, is_metric):
    if self.tcp_conn:
      try:
        sett = {'isOffroad': is_offroad}
        sett['dongleID'] = dongleID
        sett['gitCommit'] = get_commit()[:7]
        sett['currentVersion'] = get_version()
        sett['osVersion'] = HARDWARE.get_os_version()
        sett["state"] = str(state)
        sett['IsMetric'] = is_metric
        #update_dict_from_sm(sett, (sm := self.sm)['deviceState'], "connectivityStatus")
        #sett['currentBranch'] = get_short_branch()
        #sett['deviceStatus'] = deviceStatus(sm)
        #sett['remainingDataUpload'] = remainingDataUpload(sm)

        # Define the keys for each category
        bool_keys = [
          'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
          'SshEnabled', 'ExperimentalMode', 'RecordFront'
        ]

        string_keys = [
          'LongitudinalPersonality', 'HardwareSerial', 'FeaturesPackage', 'FixFingerprint'
        ]

        for key in bool_keys:
          sett[key] = safe_get(key, True)
        for key in string_keys:
          sett[key] = safe_get(key)
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
      if message and dongleID in msgpack.unpackb(message): # Message only contains dongle ID list
        self.ip = addr[0]  # Update client IP
    except Exception:
      pass

  def receive_tcp_message(self, is_offroad, state):
    if self.tcp_conn:
      try:
        if message := self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT):
          try:
            settings = msgpack.unpackb(message)

            if dongleID in settings.pop('deviceList', []):
              if (msg_type := settings.pop('msgType', None)) == 'saveToggles' and is_offroad:
                safe_put_all(settings, True)
              elif msg_type == 'saveConfig':
                #TODO: Add code to set fingerprint and features
                safe_put_all(settings)
              else:
                if msg_type == 'resetCalibration':
                  reset_calibration()
                elif msg_type == 'reboot':
                  do_reboot(state)

          except Exception as e:
            print(f"\nError: {e}\nRaw TCP: {message}")
      except Exception:
        pass

  def streamd_thread(self):
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      self.rk.monitor_time()
      is_metric = None

      if (cur_time := monotonic()) - self.last_periodic_time >= 0.333: # 3 Hz
        is_metric = params.get_bool("IsMetric")
        self.receive_udp_message()
        self.last_periodic_time = cur_time
        self.accept_new_connection()
        self.receive_tcp_message(
          is_offroad := params.get_bool("IsOffroad"),
          state := sm['controlsState'].state
        )
        self.send_tcp_message(is_offroad, state, is_metric)

      self.send_udp_message(is_metric)
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
