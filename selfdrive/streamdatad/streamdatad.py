#!/usr/bin/env python3

import socket
import msgpack
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
      'controlsState', 'uploaderState', 'radarState', 'liveCalibration', 'carParams'])
    self.rk = Ratekeeper(10)  # Ratekeeper for 10 Hz loop

    self.setup_sockets()

  def deviceStatus(self):
    if self.sm['peripheralState'].pandaType == log.PandaState.PandaType.unknown:
      return "error"
    else: # TODO: Add initialising if alerts.hasSevere from k_alerts
      return "ready"

  def remainingDataUpload(self):
    uploader_state = self.sm['uploaderState']
    immediate_queue_size = uploader_state.immediateQueueSize
    raw_queue_size = uploader_state.rawQueueSize
    return f"{immediate_queue_size + raw_queue_size} MB"

  def setup_sockets(self):
    self.udp_sock.bind((self.local_ip, UDP_PORT))
    self.udp_sock.setblocking(False)
    self.tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Enable reuse for TCP socket
    self.tcp_sock.bind((self.local_ip, TCP_PORT))
    self.tcp_sock.listen(1)
    self.tcp_sock.setblocking(False)

  def flatten_model_data(self, model_dict):
    # Clean up disengagePredictions inside 'meta'
    if (meta := model_dict.get("meta")) and isinstance(meta, dict):
      meta.pop("disengagePredictions", None)

    # Define the special handling keys and their prefixes
    special_keys = {
      "laneLines": "laneLine",
      "leadsV3": "lead",
      "roadEdges": "roadEdge"
    }

    # Modify model_dict directly for efficiency
    for key, prefix in special_keys.items():
      if (value := model_dict.pop(key, None)) and isinstance(value, list):
        model_dict.update((f"{prefix}{i}", item) for i, item in enumerate(value, 1))

    return model_dict


  def send_udp_message(self):
    if self.ip:
      self.sm.update(10) # update every 10 ms
      modelV2 = self.sm['modelV2'].to_dict()
      radarState = self.sm['radarState'].to_dict()
      liveCalibration = self.sm['liveCalibration'].to_dict()
      carParams = self.sm['carParams'].to_dict()
      brand = next((d['brand'] for d in carParams.pop('carFw', []) if 'brand' in d), None)

      data = self.flatten_model_data(modelV2)
      data.update(radarState)
      data.update(liveCalibration)
      data.update(carParams)
      data['brand'] = brand

      # Pack and send
      message = msgpack.packb(data)
      self.udp_sock.sendto(message, (self.ip, UDP_PORT))

  def send_tcp_message(self):
    if self.tcp_conn:
      try:
        self.sm.update(10)
        sett = {}
        sett['connectivityStatus'] = str(self.sm['deviceState'].networkType)
        sett['deviceStatus'] = self.deviceStatus()
        sett['alerts'] = (str(self.sm['controlsState'].alertText1) + "\n" + str(self.sm['controlsState'].alertText2))
        sett['remainingDataUpload'] = self.remainingDataUpload()
        # TODO send uploadStatus in selfdrive/loggerd/uploader.py
        sett['gitCommit'] = get_commit()[:7]
        # TODO include bukapilot changes in selfdrive/updated.py
        sett['updateStatus'] = params.get("UpdaterState") or ''

        sett['isOffroad'] = params.get_bool("IsOffroad")
        sett['enableBukapilot'] = params.get_bool("OpenpilotEnabledToggle")
        sett['quietMode'] = params.get_bool("QuietMode")
        sett['enableAssistedLaneChange'] = params.get_bool("IsAlcEnabled")
        sett['enableLaneDepartureWarning'] = params.get_bool("IsLdwEnabled")
        sett['uploadVideoWiFiOnly'] = params.get_bool("LogVideoWifiOnly")
        sett['apn'] = params.get("GsmApn") or ''
        sett['enableRoaming'] = params.get_bool("GsmRoaming")
        sett['driverPersonality'] = params.get("LongitudinalPersonality").decode('utf-8') or ''
        sett['useMetricSystem'] = params.get_bool("IsMetric")
        sett['enableSSH'] = params.get_bool("SshEnabled")
        sett['experimentalModel'] = params.get_bool("ExperimentalMode")
        sett['recordUploadDriverCamera'] = params.get_bool("RecordFront")
        sett['stopDistanceOffset'] = float(params.get("StoppingDistanceOffset") or 0)
        sett['pathSkewOffset'] = float(params.get("DrivePathOffset") or 0)
        sett['devicePowerOffTime'] = float(params.get("PowerSaverEntryDuration") or 0)
        # TODO add code for change branch
        sett['changeBranchStatus'] = params.get("ChangeBranchStatus") or ''
        sett['featurePackage'] = params.get("FeaturesPackage") or ''
        sett['fixFingerprint'] = params.get("FixFingerprint") or ''

        if self.requestInfo:
          sett['requestDeviceInfo'] = True
          sett['dongleID'] = params.get("DongleId").decode('utf-8') or ''
          sett['serial'] = params.get("HardwareSerial").decode('utf-8') or ''
          sett['hostname'] = socket.gethostname()
          sett['currentVersion'] = get_version()
          sett['osVersion'] = HARDWARE.get_os_version()
          sett['currentBranch'] = get_short_branch()
          sett['currentChangelog'] = params.get("UpdaterCurrentReleaseNotes") or ''
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
              print("Received settings:")
              print(f"EnableBukapilot: {settings['enableBukapilot']}")
              print(f"QuietMode: {settings['quietMode']}")
              print(f"EnableAssistedLaneChange: {settings['enableAssistedLaneChange']}")
              print(f"EnableLaneDepartureWarning: {settings['enableLaneDepartureWarning']}")
              print(f"UploadVideoWiFiOnly: {settings['uploadVideoWiFiOnly']}")
              print(f"EnableRoaming: {settings['enableRoaming']}")
              print(f"UseMetricSystem: {settings['useMetricSystem']}")
              print(f"EnableSSH: {settings['enableSSH']}")
              print(f"ExperimentalModel: {settings['experimentalModel']}")
              print(f"RecordUploadDriverCamera: {settings['recordUploadDriverCamera']}")
              print(f"StopDistanceOffset: {settings['stopDistanceOffset']}")
              print(f"PathSkewOffset: {settings['pathSkewOffset']}")
              print(f"DevicePowerOffTime: {settings['devicePowerOffTime']}")

              # Set values
              print("\nPutting parameters")
              params.put_bool("OpenpilotEnabledToggle", settings['enableBukapilot'])
              params.put_bool("QuietMode", settings['quietMode'])
              params.put_bool("IsAlcEnabled", settings['enableAssistedLaneChange'])
              params.put_bool("IsLdwEnabled", settings['enableLaneDepartureWarning'])
              params.put_bool("LogVideoWifiOnly", settings['uploadVideoWiFiOnly'])
              params.put_bool("GsmRoaming", settings['enableRoaming'])
              params.put_bool("IsMetric", settings['useMetricSystem'])
              params.put_bool("SshEnabled", settings['enableSSH'])
              params.put_bool("ExperimentalMode", settings['experimentalModel'])
              params.put_bool("RecordFront", settings['recordUploadDriverCamera'])
              params.put("StoppingDistanceOffset", str(settings['stopDistanceOffset']))
              params.put("DrivePathOffset", str(settings['pathSkewOffset']))
              params.put("PowerSaverEntryDuration", str(settings['devicePowerOffTime']))

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

