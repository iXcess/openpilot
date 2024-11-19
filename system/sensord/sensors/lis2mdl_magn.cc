#include "system/sensord/sensors/lis2mdl_magn.h"

#include <cassert>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

LIS2MDL_Magn::LIS2MDL_Magn(I2CBus *bus) : I2CSensor(bus) {}

int LIS2MDL_Magn::init() {
  int ret = verify_chip_id(LIS2MDL_WHO_AM_I, {LIS2MDL_CHIP_ID});
  if (ret == -1) return -1;

  ret = set_register(LIS2MDL_REG_CFG_REG_A, LIS2MDL_TEMP_COMP_MODE | LIS2MDL_ODR_100HZ | LIS2MDL_MODE_CONT);
  if (ret < 0) {
    goto fail;
  }
  
  ret = set_register(LIS2MDL_REG_CFG_REG_B, LIS2MDL_LOW_PASS_ON);
  if (ret < 0) {
    goto fail;
  }

  // enable interrupt
  /*
  ret = set_register(LIS2MDL_REG_CFG_REG_C, LIS2MDL_MAGN_INT_ON);
  if (ret < 0) {
    goto fail;
  }
  */


  // lis2mdl has a 10ms wakeup time from deep suspend mode
  util::sleep_for(10);

fail:
  return ret;
}


int LIS2MDL_Magn::shutdown()  {
  // enter sleep
  int ret = set_register(LIS2MDL_REG_CFG_REG_A, LIS2MDL_LOW_POWER_MODE);
  if (ret < 0) {
    LOGE("Could not set LIS2MDL into sleep mode!");
  }

  return ret;
}


bool LIS2MDL_Magn::get_event(MessageBuilder &msg, uint64_t ts) {
  uint64_t start_time = nanos_since_boot();

  uint8_t buffer[6];
  int len = read_register(LIS2MDL_REG_MAGN_DATA, buffer, sizeof(buffer));
  assert(len == 6);

  float scale = 1.5; // sensitivity scale factor from datasheet
  float x = -read_16_bit(buffer[5], buffer[4]) * scale;
  float y = -read_16_bit(buffer[1], buffer[0]) * scale;
  float z = read_16_bit(buffer[3], buffer[2]) * scale;

  auto event = msg.initEvent().initMagnetometer();
  event.setSource(cereal::SensorEventData::SensorSource::LIS2MDL);
  event.setVersion(1);
  event.setSensor(SENSOR_MAGNETOMETER_UNCALIBRATED);
  event.setType(SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED);
  event.setTimestamp(start_time);

  float xyz[] = {x, y, z};
  auto svec = event.initMagneticUncalibrated();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}

