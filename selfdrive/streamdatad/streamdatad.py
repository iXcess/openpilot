#!/usr/bin/env python3
import socket
import msgpack
import subprocess
import psutil
from time import monotonic
from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE

BUFFER_SIZE = 32768 # If buffer too small, SSH keys will not be fully received.
BIND_IP = "0.0.0.0"  # Bind to all network interfaces, allowing connections from any available network.
UDP_PORT = 5006
TCP_PORT = 5007
params = Params()
DONGLE_ID = params.get("DongleId").decode("utf-8")
SM_UPDATE_INTERVAL = 33

def get_wlan_ip():
  interfaces=psutil.net_if_addrs()
  stats=psutil.net_if_stats()
  if "wlan0" in interfaces:
    addr=next((a.address for a in interfaces["wlan0"] if a.family==socket.AF_INET and a.address), None)
    return addr if addr and stats.get("wlan0",{}).isup else "Not Connected"
  for iface in interfaces:
    if iface.startswith("wl") and iface!="wlan1" and stats.get(iface,{}).isup:
      addr=next((a.address for a in interfaces[iface] if a.family==socket.AF_INET and a.address), None)
      if addr:
        return addr
  return "Not Connected"

def check_for_updates():
  subprocess.Popen(["pkill", "-SIGUSR1", "-f", "system.updated.updated"])

def fetch_update():
  subprocess.Popen(["pkill", "-SIGHUP", "-f", "system.updated.updated"])

def extract_model_data(data_dict):
  extracted_data = {key: data_dict.get(key) for key in ("position", "acceleration", "frameId")}
  list_keys = ("laneLines", "roadEdges")
  expected_size = len(extracted_data) + sum(len(data_dict.get(k, [])) for k in list_keys)
  for key in list_keys:
    value = data_dict.get(key)
    if isinstance(value, list):
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
  for param_key, value in settings_to_put.items():
    try:
      params.put_bool(param_key, value) if is_bool else params.put(param_key, str(value))
    except Exception as e:
      print(f"Error putting {param_key}: {e}")

def reset_calibration():
  params.remove("CalibrationParams")
  params.remove("LiveTorqueParameters")
  # Parameters below need to be removed for newer op version
  # params.remove("LiveParameters")
  # params.remove("LiveParametersV2")
  # params.remove("LiveDelay")

def do_reboot(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.put_bool("DoReboot", True)

def update_dict_from_sm(target_dict, sm_subset, keys):
  try:
    c = sm_subset.to_dict()
    for k in keys:
      target_dict[k] = c[k]
  except Exception:
    return

class Streamer:
  def __init__(self, sm=None):
    self.udp_send_ip = None
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm else messaging.SubMaster([
      'modelV2', 'controlsState', 'radarState', 'liveCalibration',
      'driverMonitoringState', 'carState', 'longitudinalPlan',
    ])
    self.rk = Ratekeeper(25)  # Ratekeeper for 25 Hz loop
    self.last_periodic_time = 0  # Track last periodic task
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.setup_sockets()

  def setup_sockets(self):
    (udp_sock := self.udp_sock).bind((BIND_IP, UDP_PORT))
    udp_sock.setblocking(False)
    (tcp_sock := self.tcp_sock).setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    tcp_sock.bind((BIND_IP, TCP_PORT))
    tcp_sock.listen(1)
    tcp_sock.setblocking(False)

  def send_udp_message(self, is_metric):
    if send_ip := self.udp_send_ip:
      (data := extract_model_data((sm := self.sm)['modelV2'].to_dict())).update(sm['controlsState'].to_dict())
      if is_metric is not None:
        data["IsMetric"] = is_metric
      data['dongleID'] = DONGLE_ID
      update_dict_from_sm(data, sm['radarState'], ["leadOne", "leadTwo"])
      update_dict_from_sm(data, sm['driverMonitoringState'], ["isActiveMode", "events"])
      update_dict_from_sm(data, sm['liveCalibration'], ["height"])
      update_dict_from_sm(data, sm['carState'], ["vEgoCluster"])
      update_dict_from_sm(data, sm['longitudinalPlan'], ["personality"])

      # Pack and send
      try:
        self.udp_sock.sendto(msgpack.packb(data), (send_ip, UDP_PORT))
      except (BlockingIOError, OSError):
        pass
      except Exception:
        pass

  def send_tcp_message(self, is_offroad, state, is_metric):
    if tcp_conn := self.tcp_conn:
      try:
        sett = {'isOffroad': is_offroad}
        sett['dongleID'] = DONGLE_ID
        sett['gitCommit'] = get_commit()[:7]
        sett['currentVersion'] = get_version()
        sett['osVersion'] = HARDWARE.get_os_version()
        sett["state"] = str(state)
        sett['IsMetric'] = is_metric
        sett['localIP'] = self.local_wlan_ip

        # Define the keys for each category
        bool_keys = {
          'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
          'SshEnabled', 'ExperimentalMode', 'RecordFront', 'UpdateAvailable',
          'UpdaterFetchAvailable'
        }

        string_keys = {
          'LongitudinalPersonality', 'HardwareSerial', 'FeaturesPackage', 'FixFingerprint',
          'UpdaterCurrentReleaseNotes', 'UpdaterTargetBranch', 'UpdaterState', 'UpdateFailedCount',
          'LastUpdateTime', 'GithubUsername'
        }

        for key in bool_keys:
          sett[key] = safe_get(key, True)
        for key in string_keys:
          sett[key] = safe_get(key, False)
        tcp_conn.sendall(msgpack.packb(sett))

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
      if message and DONGLE_ID in msgpack.unpackb(message): # Message only contains dongle ID list
        self.udp_send_ip = addr[0]  # Update client IP
    except Exception:
      pass

  def receive_tcp_message(self, is_offroad, state):
    if tcp_conn := self.tcp_conn:
      try:
        if message := tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT):
          try:
            settings = msgpack.unpackb(message)
            # Check if account is valid
            if DONGLE_ID in settings.pop('deviceList', []):
              match settings.pop('msgType'):
                case 'saveToggles':
                  is_offroad and safe_put_all(settings, True)
                case 'saveConfig':
                  #TODO: Add code to set fingerprint and features
                  safe_put_all(settings)
                case 'resetCalibration':
                  reset_calibration()
                case 'reboot':
                  do_reboot(state)
                case 'tncAccepted':
                  params.put("HasAcceptedTerms", terms_version)
                  params.put("CompletedTrainingVersion", training_version)
                case 'changeTargetBranch':
                  if targetBranch := settings.get('targetBranch'):
                    params.put("UpdaterTargetBranch", targetBranch)
                    check_for_updates()
                case 'update':
                  match settings.get('action'):
                    case 'check':
                      check_for_updates()
                    case 'install':
                      do_reboot(state)
                    case 'fetch':
                      fetch_update()
                case 'ssh':
                  if username := settings.get('username'):
                    params.put("GithubUsername", username)
                    params.put("GithubSshKeys", settings.get('keys'))
                  else:
                    params.remove("GithubUsername")
                    params.remove("GithubSshKeys")

          except Exception as e:
            print(f"\nError: {e}\nRaw TCP: {message}")
      except Exception:
        pass

  def streamd_thread(self):
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (rk:= self.rk).monitor_time()
      is_metric = None

      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1: # 1 Hz
        self.last_1hz_task_time = cur_time
        self.local_wlan_ip = get_wlan_ip()

      if cur_time - self.last_periodic_time >= 0.333: # 3 Hz
        self.last_periodic_time = cur_time
        self.accept_new_connection()
        self.receive_tcp_message(is_offroad := params.get_bool("IsOffroad"), state := sm['controlsState'].state)
        self.send_tcp_message(is_offroad, state, is_metric := params.get_bool("IsMetric"))
        self.receive_udp_message()

      self.send_udp_message(is_metric)
      rk.keep_time()

def main():
  Streamer().streamd_thread()

if __name__ == "__main__":
  main()
