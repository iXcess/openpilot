import random
import subprocess
import hashlib
import os

from cereal import log
from openpilot.system.hardware.base import HardwareBase, ThermalConfig

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength


class Ka2(HardwareBase):
  def get_os_version(self):
    with open("/VERSION") as f:
      return f.read().strip()

  def get_device_type(self):
    return "ka2"

  def get_sound_card_online(self):
    return True

  def reboot(self, reason=None):
    subprocess.check_output(["sudo", "reboot"])

  def uninstall(self):
    Path("/data/__system_reset__").touch()
    os.sync()
    self.reboot()

  def get_imei(self, slot):
    # generate fake 15 digit imei from eth0 mac address
    mac = subprocess.getoutput("cat /sys/class/net/eth0/address")
    clean_mac = mac.replace(':', '').replace('-', '')

    return hashlib.sha256(clean_mac.encode()).hexdigest()[:15]

  def get_serial(self):
    return subprocess.check_output("grep 'Serial' /proc/cpuinfo | sed 's/.*: //'", shell=True, text=True).strip()

  def get_network_info(self):
    return None

  def get_network_type(self):
    return NetworkType.wifi

  def get_sim_info(self):
    return {
      'sim_id': '',
      'mcc_mnc': None,
      'network_type': ["Unknown"],
      'sim_state': ["ABSENT"],
      'data_connected': False
    }

  def get_network_strength(self, network_type):
    return NetworkStrength.unknown

  def get_current_power_draw(self):
    return 0

  def get_som_power_draw(self):
    return 0

  def shutdown(self):
    print("SHUTDOWN!")

  def get_thermal_config(self):
    return ThermalConfig(cpu=((None,), 1), gpu=((None,), 1), mem=(None, 1), bat=(None, 1), pmic=((None,), 1))

  def set_screen_brightness(self, percentage):
    pass

  def get_screen_brightness(self):
    return 0

  def set_power_save(self, powersave_enabled):
    pass

  def get_gpu_usage_percent(self):
    return 0

  def get_modem_temperatures(self):
    return []

  def get_nvme_temperatures(self):
    return []

  def initialize_hardware(self):
    pass

  def get_networks(self):
    return None
