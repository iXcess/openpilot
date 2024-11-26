#pragma once

#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

#include "common/params.h"
#include "common/util.h"
#include "system/hardware/base.h"

class HardwareKa2 : public HardwareNone {
public:
  static constexpr float MAX_VOLUME = 0.9;
  static constexpr float MIN_VOLUME = 0.1;
  static bool TICI() { return false; }
  static bool AGNOS() { return false; }
  static bool KA2() { return true; }
  static std::string get_os_version() {
    return "KommuOS " + util::read_file("/VERSION");
  }

  static std::string get_name() {
    return "KommuAssist2";
  }

  static cereal::InitData::DeviceType get_device_type() {
    return cereal::InitData::DeviceType::KA2;
  }

  // TODO static int get_voltage() { return std::atoi(util::read_file("/sys/class/hwmon/hwmon1/in1_input").c_str()); }
  // TODO static int get_current() { return std::atoi(util::read_file("/sys/class/hwmon/hwmon1/curr1_input").c_str()); }

  static std::string get_serial() {
    static std::string serial("");
    if (serial.empty()) {
      std::ifstream stream("/proc/cmdline");
      std::string cmdline;
      std::getline(stream, cmdline);

      auto start = cmdline.find("serialno=");
      if (start == std::string::npos) {
        serial = "cccccc";
      } else {
        auto end = cmdline.find(" ", start + 9);
        serial = cmdline.substr(start + 9, end - start - 9);
      }
    }
    return serial;
  }

  static void reboot() { std::system("sudo reboot"); }
  static void poweroff() { std::system("sudo poweroff"); }

  /* TODO
  static std::map<std::string, std::string> get_init_logs() {
    std::map<std::string, std::string> ret = {
      {"/BUILD", util::read_file("/BUILD")},
      {"lsblk", util::check_output("lsblk -o NAME,SIZE,STATE,VENDOR,MODEL,REV,SERIAL")},
      {"SOM ID", util::read_file("/sys/devices/platform/vendor/vendor:gpio-som-id/som_id")},
    };

    std::string bs = util::check_output("abctl --boot_slot");
    ret["boot slot"] = bs.substr(0, bs.find_first_of("\n"));

    std::string temp = util::read_file("/dev/disk/by-partlabel/ssd");
    temp.erase(temp.find_last_not_of(std::string("\0\r\n", 3))+1);
    ret["boot temp"] = temp;

    // TODO: log something from system and boot
    for (std::string part : {"xbl", "abl", "aop", "devcfg", "xbl_config"}) {
      for (std::string slot : {"a", "b"}) {
        std::string partition = part + "_" + slot;
        std::string hash = util::check_output("sha256sum /dev/disk/by-partlabel/" + partition);
        ret[partition] = hash.substr(0, hash.find_first_of(" "));
      }
    }

    return ret;
  }
  */

  static bool get_ssh_enabled() { return Params().getBool("SshEnabled"); }
  static void set_ssh_enabled(bool enabled) { Params().putBool("SshEnabled", enabled); }

  static void config_cpu_rendering(bool offscreen) {
    if (offscreen) {
      setenv("QT_QPA_PLATFORM", "offscreen", 1); // offscreen doesn't work with EGL/GLES
    }
    setenv("__GLX_VENDOR_LIBRARY_NAME", "mesa", 1);
    setenv("LP_NUM_THREADS", "0", 1); // disable threading so we stay on our assigned CPU
  }
};
