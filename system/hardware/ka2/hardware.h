#pragma once

#include <string>

#include "system/hardware/base.h"

class HardwareKa2 : public HardwareNone {
public:
  static std::string get_os_version() { 
    return "KAOS " + util::read_file("/VERSION"); 
  }

  static std::string get_name() { 
    return "KommuAssist2"; 
  }

  static cereal::InitData::DeviceType get_device_type() {
     return cereal::InitData::DeviceType::KA2; 
  }

  static bool TICI() { return false; }
  static bool AGNOS() { return false; }

  static void config_cpu_rendering(bool offscreen) {
    if (offscreen) {
      setenv("QT_QPA_PLATFORM", "offscreen", 1);
    }
    setenv("__GLX_VENDOR_LIBRARY_NAME", "mesa", 1);
    setenv("LP_NUM_THREADS", "0", 1); // disable threading so we stay on our assigned CPU
  }
};
