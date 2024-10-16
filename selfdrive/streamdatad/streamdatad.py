import socket
import fcntl
import struct
import os

from openpilot.common.realtime import Ratekeeper
import cereal.messaging as messaging
from cereal import log
from openpilot.system.version import get_version, get_commit, get_short_branch
from openpilot.common.params import Params

BUFFER_SIZE = 1024
UDP_PORT = 5006
TCP_PORT = 5007
params = Params()

def get_wlan_ip():
  for iface in os.listdir('/sys/class/net/'):
    if iface.startswith('wl'):
      try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip = socket.inet_ntoa(fcntl.ioctl(
          s.fileno(), 0x8915, struct.pack('256s', bytes(iface[:15], 'utf-8'))
        )[20:24])
        return ip
      except IOError:
        pass

class Streamer:
  def __init__(self, sm=None):
    self.local_ip = get_wlan_ip()
    self.ip = None
    self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.tcp_conn = None
    self.sm = sm if sm \
      else messaging.SubMaster(['navInstruction', 'settings', 'deviceState', 'peripheralState',\
      'controlsState', 'uploaderState'])
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

  def send_udp_message(self):
    if self.ip:
      self.sm.update()
      message = self.sm['navInstruction'].as_builder().to_bytes()
      self.udp_sock.sendto(message, (self.ip, UDP_PORT))

  def send_tcp_message(self):
    if self.tcp_conn:
      try:
        sett = self.sm['settings'].as_builder()

        sett.connectivityStatus = str(self.sm['deviceState'].networkType)
        sett.deviceStatus = self.deviceStatus()
        sett.alerts = (str(self.sm['controlsState'].alertText1) + "\n" + str(self.sm['controlsState'].alertText2))
        sett.remainingDataUpload = self.remainingDataUpload()
        # TODO send uploadStatus in selfdrive/loggerd/uploader.py
        sett.gitCommit = get_commit()
        # TODO include bukapilot changes in selfdrive/updated.py
        sett.updateStatus = params.get("UpdaterState") or ''

        settings_open = True
        if settings_open:
          sett.enableBukapilot = params.get_bool("OpenpilotEnabledToggle")
          sett.quietMode = params.get_bool("QuietMode")
          sett.enableAssistedLaneChange = params.get_bool("IsAlcEnabled")
          sett.enableLaneDepartureWarning = params.get_bool("IsLdwEnabled")
          sett.uploadVideoWiFiOnly = params.get_bool("LogVideoWifiOnly")
          sett.apn = params.get("GsmApn") or ''
          sett.enableRoaming = params.get_bool("GsmRoaming")
          sett.driverPersonality = params.get("LongitudinalPersonality")
          sett.useMetricSystem = params.get_bool("IsMetric")
          sett.enableSSH = params.get_bool("SshEnabled")
          sett.experimentalModel = params.get_bool("ExperimentalMode")
          sett.stopDistanceOffset = params.get("StoppingDistanceOffset") or 0
          sett.pathSkewOffset = params.get("DrivePathOffset") or 0
          sett.devicePowerOffTime = params.get("PowerSaverEntryDuration") or 0
          # TODO add code for change branch
          sett.changeBranchStatus = params.get("ChangeBranchStatus") or ''
          sett.featurePackage = params.get("FeaturesPackage") or ''
          sett.fixFingerprint = params.get("FixFingerprint") or ''

        request_info = True
        if request_info:
          sett.dongleID = params.get("DongleId")
          sett.serial = params.get("HardwareSerial") or ''
          sett.ipAddress = get_wlan_ip()
          sett.hostname = socket.gethostname()
          sett.currentVersion = get_version()
          sett.currentBranch = get_short_branch()
          sett.currentChangelog = params.get("UpdaterCurrentReleaseNotes") or ''

        self.tcp_conn.sendall(sett.to_bytes())
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

      try:
        nav_instruction = messaging.navInstruction.from_bytes(message)
        print(f"Deserialized UDP navInstruction: {nav_instruction.key.decode('utf-8')} = {nav_instruction.value.decode('utf-8')}")
      except Exception:
        print(f"Schema error. Raw UDP: {message.decode('utf-8')}")
    except socket.error:
      pass

  def receive_tcp_message(self):
    if self.tcp_conn:
      try:
        message = self.tcp_conn.recv(BUFFER_SIZE, socket.MSG_DONTWAIT)
        if message:
          try:
            # Deserialize the message using the Settings struct
            settings = messaging.settings.from_bytes(message)
            print(f"Received settings: {settings.key.decode('utf-8')} = {settings.value.decode('utf-8')}")
          except Exception:
            print(f"Schema error. Raw TCP: {message.decode('utf-8')}")
      except socket.error:
        pass

  def streamd_thread(self):
    while True:
      self.rk.monitor_time()
      self.send_udp_message()
      self.accept_new_connection()
      self.send_tcp_message()
      self.receive_udp_message()
      self.receive_tcp_message()
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

