#!/usr/bin/env python3
import socket
import msgpack
import subprocess
import psutil
import threading
import re
import math
from time import monotonic
from bluezero import adapter, peripheral
import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_version, get_commit, terms_version, training_version
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.selfdrive.car.fingerprints import _FINGERPRINTS as FINGERPRINTS
from openpilot.common.features import Features

MESSAGE_HZ = 10 # Expected message rate, must match app value

# BLE advertising name
BLE_NAME = "KommuBLE"

# Channel IDs
CHANNEL_VISUALISATION = 0x01
CHANNEL_SETTINGS = 0x02

# BLE Nordic UART UUIDs
UART_SERVICE      = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
RX_CHARACTERISTIC = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'  # Write from phone
TX_CHARACTERISTIC = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # Notify to phone

SM_UPDATE_INTERVAL = 33 # in ms, the interval where capnp submaster updates
WIFI_CONNECT_TIMEOUT_SECONDS = 20 # Timeout for device Wi-Fi connection attempts
NO_NETWORK_REGEX = re.compile(r"no network.*ssid", re.IGNORECASE)
params = Params()
DONGLE_ID = (params.get("DongleId") or b"").decode()
SUPPORTED_MODELS = {getattr(car, 'value', car) for car in FINGERPRINTS}
features = Features()

# Call functions with cached values only once
GIT_COMMIT = get_commit()[:7]
CUR_VERSION = get_version()
OS_VERSION = HARDWARE.get_os_version()

def chunk_and_send(ble, channel: int, payload: bytes, CHUNK_SIZE=240):
  cnts = chunk_and_send.__dict__.setdefault("_counters", {})
  # get & increment counter, cycle 1–255 for msg_id
  cnts[channel] = msg_id = cnts.get(channel, 0) % 255 + 1
  view = memoryview(payload)
  for seg_idx in range(total_segments := -(-len(payload) // CHUNK_SIZE)):
    offset = seg_idx * CHUNK_SIZE
    ble.send(bytes([channel, msg_id, total_segments, seg_idx]) + view[offset : offset + CHUNK_SIZE])

def forget_wifi_network(ssid):
  if not ssid:
    return False
  threading.Thread(daemon=True, target=lambda: subprocess.run(["sudo", "nmcli", "con", "delete", ssid], text=True)).start()
  return True

def check_for_updates():
  subprocess.Popen(["pkill", "-SIGUSR1", "-f", "system.updated.updated"])

def fetch_update():
  subprocess.Popen(["pkill", "-SIGHUP", "-f", "system.updated.updated"])

def change_branch_and_update(target_branch):
  params.put("UpdaterTargetBranch", target_branch)
  check_for_updates()

def extract_model_data(data_dict):
  try:
    data = {key: data_dict[key] for key in ("position", "frameId")}
    data["accelerationX"] = data_dict.get("acceleration", {}).get("x")
    for key in ("laneLines", "roadEdges", "laneLineProbs", "roadEdgeStds"):
      for i, item in enumerate(data_dict[key], 1):
        data[f"{key[:-1]}{i}"] = item
    return data
  except Exception:
    return {}

def safe_get(key, is_bool=False):
  """Safely retrieve a parameter value."""
  try:
    return params.get_bool(key) if is_bool else params.get(key).decode()
  except Exception:
    return False if is_bool else ''

def safe_put_all(settings_to_put, is_bool=False):
  """Safely store multiple parameters."""
  for param_key, value in settings_to_put.items():
    try:
      (params.put_bool_nonblocking if is_bool else params.put_nonblocking)(param_key, value if is_bool else str(value))
    except Exception as e:
      cloudlog.error(f"Error putting {param_key}: {e}")

def reset_calibration(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")
    # Parameters below need to be removed for newer op version
    # params.remove("LiveParameters")
    # params.remove("LiveParametersV2")
    # params.remove("LiveDelay")

def do_reboot(state):
  if state == log.ControlsState.OpenpilotState.disabled:
    params.put_bool_nonblocking("DoReboot", True)

def update_dict_from_sm(target_dict, sm_subset, keys):
  try:
    c = sm_subset.to_dict()
    for k in keys:
      target_dict[k] = c[k]
  except KeyError:
    pass

def quantize(o, key_name=None):
  if isinstance(o, dict):
    return {k: quantize(v, k) for k, v in o.items()}
  if isinstance(o, list):
    return [quantize(v, key_name) for v in o]
  if isinstance(o, float):
    if math.isnan(o):
      return None
    # keep 3dp if probability or key is vEgoCluster
    return round(o, 3) if 0 < abs(o) < 1 or key_name == "vEgoCluster" else round(o)
  return o

def is_supported_model(name: str) -> bool:
  return name.upper() in SUPPORTED_MODELS

class BLEBridge:
  """Threaded BLE Nordic UART bridge with RX and TX."""
  def __init__(self):
    self.ad = list(adapter.Adapter.available())[0]
    self.dev = peripheral.Peripheral(self.ad.address, local_name=BLE_NAME, appearance=1344)

    self.rx_queue = []
    self.tx_char = None

    # Add UART service
    self.dev.add_service(srv_id=1, uuid=UART_SERVICE, primary=True)

    # RX: phone -> device (write)
    self.dev.add_characteristic(srv_id=1, chr_id=1, uuid=RX_CHARACTERISTIC,
                                value=[], notifying=False,
                                flags=['write', 'write-without-response'],
                                write_callback=self.on_write)

    # TX: device -> phone (notify)
    self.dev.add_characteristic(srv_id=1, chr_id=2, uuid=TX_CHARACTERISTIC,
                                value=[], notifying=False,
                                flags=['notify'],
                                notify_callback=self.notify_state)

    self.dev.on_connect = self.on_connect
    self.dev.on_disconnect = self.on_disconnect

  def on_connect(self, dev):
    print(f"BLE Connected: {dev.address}")

  def on_disconnect(self, adapter_addr, dev_addr):
    print(f"BLE Disconnected: {dev_addr}")

  def notify_state(self, notifying, characteristic):
    self.tx_char = characteristic if notifying else None

  def on_write(self, value, options):
    """Receive bytes from phone and store in queue."""
    self.rx_queue.append(bytes(value))

  def send(self, payload: bytes):
    """Send bytes to phone via BLE."""
    if self.tx_char:
      self.tx_char.set_value(list(payload))

  def read(self):
    """Pop next received BLE packet if available."""
    if self.rx_queue:
      return self.rx_queue.pop(0)
    return None

  def start(self):
    """Start BLE peripheral loop."""
    self.dev.publish()
    while True:
      pass  # just keep thread running

class Streamer:
  """Handles visualisation and settings BLE streams."""
  def __init__(self, sm=None):
    self.ble = BLEBridge()
    self.sm = sm if sm else messaging.SubMaster([
      'modelV2', 'controlsState', 'radarState', 'liveCalibration',
      'driverMonitoringState', 'carState', 'longitudinalPlan',
    ])
    self.rk = Ratekeeper(MESSAGE_HZ) # Ratekeeper for loop
    self.last_periodic_time = 0 # Track last periodic task
    self.last_1hz_task_time = 0
    self.local_wlan_ip = None
    self.active_wlan_ssid = None
    self.current_wifi_iface_name = None
    self.wifi_connect_attempt_ssid = None
    self.wifi_connect_attempt_start_time = None

  def connect_to_wifi(self, ssid, password, cur_time):
    if not (ssid := ssid.strip()):
      return False
    self.wifi_connect_attempt_ssid = ssid
    self.wifi_connect_attempt_start_time = cur_time
    cmd = ['dev', 'wifi', 'connect', ssid]
    if password:
      cmd.extend(['password', password])
    if ifname := self.current_wifi_iface_name:
      cmd.extend(['ifname', ifname])
    def run_nmcli():
      result = subprocess.run(["sudo", "nmcli"] + cmd, text=True, capture_output=True)
      if result.returncode != 0 and NO_NETWORK_REGEX.search(result.stderr):
        cloudlog.warning(f"Wi-Fi SSID {ssid} not found, clearing attempt.")
        self.wifi_connect_attempt_ssid = None
        self.wifi_connect_attempt_start_time = None
        return False
    threading.Thread(target=run_nmcli, daemon=True).start()
    return True

  def update_wlan_info_async(self):
    def get_wlan_info():
      if (interfaces := psutil.net_if_addrs()) and (stats := psutil.net_if_stats()) and "wlan0" in interfaces and stats.get("wlan0", {}).isup:
        selected_iface = "wlan0"
      else:
        selected_iface = next(
          (iface for iface in interfaces if iface.startswith("wl") and iface != "wlan1" and stats.get(iface, {}).isup and
           any(a.family == socket.AF_INET for a in interfaces[iface])), None)
      ip_address = ssid = None
      if selected_iface:
        ip_address = next((a.address for a in interfaces[selected_iface] if a.family == socket.AF_INET), None)
        try:
          if (result := subprocess.run(
            ['nmcli', '-t', '-f', 'active,ssid,device', 'dev', 'wifi'], capture_output=True, text=True, timeout=0.1
          )).returncode == 0 and (output := result.stdout):
            for line in output.splitlines():
              if (parts := line.split(':')) and len(parts) >= 3 and parts[0] == 'yes' and parts[2] == selected_iface:
                ssid = parts[1]
                break
        except subprocess.TimeoutExpired:
          pass
        except Exception:
          pass
      self.local_wlan_ip = ip_address
      self.active_wlan_ssid = ssid
      self.current_wifi_iface_name = selected_iface
    threading.Thread(target=get_wlan_info, daemon=True).start()

  def send_visualisation_message(self, is_metric):
    (data := extract_model_data((sm := self.sm)['modelV2'].to_dict())).update(sm['controlsState'].to_dict())
    data["IsMetric"] = is_metric
    data['dongleID'] = DONGLE_ID
    update_dict_from_sm(data, sm['radarState'], ["leadOne", "leadTwo"])
    update_dict_from_sm(data, sm['driverMonitoringState'], ["isActiveMode", "events"])
    data["heightVal"] = sm['liveCalibration'].to_dict().get("height", [None])[0]
    update_dict_from_sm(data, sm['carState'], ["vEgoCluster"])
    update_dict_from_sm(data, sm['longitudinalPlan'], ["personality"])
    data = quantize(data)
    try:
      payload = msgpack.packb(data)
      chunk_and_send(self.ble, CHANNEL_VISUALISATION, payload)
    except Exception as e:
      cloudlog.error(f"BLE visualisation sending error: {e}")

  def send_settings_message(self, is_offroad, state, is_metric):
    sett = {'isOffroad': is_offroad}
    sett['dongleID'] = DONGLE_ID
    sett['gitCommit'] = GIT_COMMIT
    sett['currentVersion'] = CUR_VERSION
    sett['osVersion'] = OS_VERSION
    sett["state"] = str(state)
    sett['IsMetric'] = is_metric
    sett['localIP'] = self.local_wlan_ip
    sett['activeWlanSSID'] = \
      f"Connecting to\n{attempt_ssid}" if (attempt_ssid := self.wifi_connect_attempt_ssid) else self.active_wlan_ssid

    bool_keys = {
      'OpenpilotEnabledToggle', 'QuietMode', 'IsAlcEnabled', 'IsLdwEnabled',
      'SshEnabled', 'ExperimentalMode', 'RecordFront', 'UpdateAvailable',
      'UpdaterFetchAvailable'
    }
    string_keys = {
      'LongitudinalPersonality', 'HardwareSerial', 'FeaturesPackage', 'FixFingerprint',
      'UpdaterTargetBranch', 'UpdaterState', 'UpdateFailedCount',
      'LastUpdateTime', 'GithubUsername'
    }

    for key in bool_keys:
      sett[key] = safe_get(key, True)
    for key in string_keys:
      sett[key] = safe_get(key, False)
    try:
      payload = msgpack.packb(sett)
      chunk_and_send(self.ble, CHANNEL_SETTINGS, payload)
    except Exception as e:
      cloudlog.error(f"BLE settings sending error: {e}")

  def receive_settings_message(self, state, cur_time, is_offroad):
    message = self.ble.read()
    if not message:
      return
    try:
      if message[0] != CHANNEL_SETTINGS:  # Only handle settings messages
        return
      settings = msgpack.unpackb(message[1:])
      # Check if account is valid
      if DONGLE_ID in settings.pop('deviceList', []):
        match settings.pop('msgType'):
          case 'saveToggle':
            safe_put_all(settings, True)
          case 'saveConfig':
            if (fix_fp := settings.pop('FixFingerprint', None)) is not None:
              if (fix_fp := fix_fp.strip()) == "" or is_supported_model(fix_fp):
                safe_put_all({'FixFingerprint': fix_fp})
            if (features_to_add := settings.pop('FeaturesPackage', None)) is not None:
              features.set_features(features_to_add)
            safe_put_all(settings)
          case 'resetCalibration':
            reset_calibration(state)
          case 'reboot':
            do_reboot(state)
          case 'tncAccepted':
            params.put_nonblocking("HasAcceptedTerms", terms_version)
            params.put_nonblocking("CompletedTrainingVersion", training_version)
          case 'changeTargetBranch':
            if targetBranch := settings.get('targetBranch'):
              threading.Thread(target=change_branch_and_update, args=(targetBranch,)).start()
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
              params.put_nonblocking("GithubUsername", username)
              params.put_nonblocking("GithubSshKeys", settings.get('keys'))
            else:
              params.remove("GithubUsername")
              params.remove("GithubSshKeys")
          case 'wifi':
            if (ssid := settings.get('ssid')):
              match settings.get('action'):
                case 'connect':
                  self.connect_to_wifi(ssid, settings.get('password'), cur_time)
                case 'forget':
                  forget_wifi_network(ssid)
          case 'formatSD':
            if is_offroad:
              safe_put_all({"FormatSDCard": True}, True)
    except Exception as e:
      cloudlog.error(f"BLE settings receiving error: {e}")

  def streamd_thread(self):
    threading.Thread(target=self.ble.start, daemon=True).start()
    is_metric = None
    while True:
      (sm := self.sm).update(SM_UPDATE_INTERVAL)
      (rk := self.rk).monitor_time()

      if (cur_time := monotonic()) - self.last_1hz_task_time >= 1:  # 1 Hz tasks
        self.last_1hz_task_time = cur_time
        self.update_wlan_info_async()
        if attempt_ssid := self.wifi_connect_attempt_ssid:
          if ((connected := self.active_wlan_ssid == attempt_ssid) or
            (cur_time - self.wifi_connect_attempt_start_time) >= WIFI_CONNECT_TIMEOUT_SECONDS):
              if not connected:
                cloudlog.warning(f"Timeout reached, forgetting SSID {attempt_ssid}")
                forget_wifi_network(attempt_ssid)
              else:
                cloudlog.info(f"Wi-Fi {attempt_ssid} connected")
              self.wifi_connect_attempt_ssid = None
              self.wifi_connect_attempt_start_time = None

      if cur_time - self.last_periodic_time >= 0.333:  # 3 Hz settings
        self.last_periodic_time = cur_time
        self.receive_settings_message(state := sm['controlsState'].state, cur_time, is_offroad := params.get_bool("IsOffroad"))
        self.send_settings_message(is_offroad, state, is_metric := params.get_bool("IsMetric"))

      self.send_visualisation_message(is_metric)
      rk.keep_time()

def main():
  Streamer().streamd_thread()

if __name__ == "__main__":
  main()
