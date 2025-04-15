from cereal import car

# reference: http://sunshine2k.de/articles/coding/crc/understanding_crc.html#ch3
crc8_lut_8h2f = []

def init_lut_crc8_8h2f():
  poly = 0x2F

  for i in range(256):
    crc = i
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
    crc8_lut_8h2f.append(crc)

def get_crc8_8h2f(dat):
  crc = 0xFF    # initial value for crc8_8h2f
  for i in range(len(dat)):
    crc ^= dat[i]
    crc = crc8_lut_8h2f[crc]

  return crc ^ 0xFF

def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, wheel_touch_chime, raw_cnt,\
    lks_aux, lks_audio, lks_tactile, lks_assist_mode, lka_enable):
  """Creates a CAN message for the Proton LKA Steer Command."""
  values = {
    "LKA_ENABLE": lka_enable,
    "LKAS_ENGAGED1": steer_req,
    "LKAS_LINE_ACTIVE": steer_req,
    "STEER_CMD": abs(steer) if steer_req else 0,
    "STEER_DIR": steer <= 0,
    "COUNTER": raw_cnt,
    "LDW_READY": 1,
    # Disable steering vibration for LDW if steer not enabled and LKS set to Warn Only mode and Tactile warning type
    "LDW_STEERING": 0,
    "SET_ME_1": 1,
    "LKS_STATUS": 1,
    "STOCK_LKS_AUX": lks_aux,
    "LKS_WARNING_AUDIO_TYPE": lks_audio,
    "LKS_WARNING_TACTILE_TYPE": lks_tactile,
    "LKS_ASSIST_MODE" : lks_assist_mode,
    "HAND_ON_WHEEL_WARNING": wheel_touch_warning,
    "WHEEL_WARNING_CHIME": wheel_touch_chime,
  }

  dat = packer.make_can_msg("ADAS_LKAS", 0, values)[2]
  values["CHECKSUM"] = get_crc8_8h2f(dat[:-1])

  return packer.make_can_msg("ADAS_LKAS", 0, values)

def create_hud(packer, steer, steer_req, ldw, rlane, llane):
  """Creates a CAN message for the Proton LKA Steer Command."""
  steer_dir = steer <= 0
  values = {
    "LANE_DEPARTURE_AUDIO_RIGHT": ldw and not steer_dir,
    "LANE_DEPARTURE_AUDIO_LEFT": ldw and steer_dir,
    "LEFT_LANE_VISIBLE_DISENGAGE": 0,
    "RIGHT_LANE_VISIBLE_DISENGAGE": 0,
    "STEER_REQ_RIGHT": steer_req,
    "STEER_REQ_LEFT": steer_req,
    "STEER_REQ_MAJOR": 1 if steer_req else 0,
    "LLANE_CHAR": 0x91 if steer_req else 0x4b,
    "CURVATURE": 0x3f if steer_req else 0x3f,
    "RLANE_CHAR": 0xaa if steer_req else 0x3d,
  }

  dat = packer.make_can_msg("LKAS", 0, values)[2]
  return packer.make_can_msg("LKAS", 0, values)

def create_lead_detect(packer, is_lead, steer_req):
  """Creates a CAN message for the Proton LKA Steer Command."""
  values = {
    "LEAD_DISTANCE": 30,
    "NEW_SIGNAL_1": 0x7f,
    "NEW_SIGNAL_2": 0x7e,
    "IS_LEAD2": is_lead,
    "IS_LEAD1": is_lead,
    "LEAD_TOO_NEAR": 0,
  }

  dat = packer.make_can_msg("ADAS_LEAD_DETECT", 0, values)[2]
  return packer.make_can_msg("ADAS_LEAD_DETECT", 0, values)

def create_pcm(packer, steer, steer_req, raw_cnt):

  values = {
    "ACC_SET_SPEED": 0x23 if steer_req else 0,
    "SET_DISTANCE": 1 if steer_req else 0,
    "NEW_SIGNAL_1": 3,
    "ACC_SET": 1 if steer_req else 0,
    "COUNTER": raw_cnt,
    "ACC_ON_OFF_BUTTON": 1,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[2]
  values["CHECKSUM"] = get_crc8_8h2f(dat[:-1])

  return packer.make_can_msg("PCM_BUTTONS", 0, values)

def create_acc_cmd(packer, accel, enabled, raw_cnt, gas_override, standstill):
  accel_cmd = accel * 15
  if gas_override:
    accel_cmd = 0
  values = {
    "CMD": accel_cmd,
    "CMD_OFFSET1": accel_cmd,
    "CMD_OFFSET2": accel_cmd,
    "ACC_REQ": enabled,
    "NOT_ACC_REQ": not enabled,
    "SET_ME_1": 1,
    "CRUISE_ENABLE": enabled and not gas_override,
    "COUNTER": raw_cnt,

    # not sure
    "BRAKE_ENGAGED": 0,
    "SET_ME_X6A": 0x6A,
    "RISING_ENGAGE": 0,
    "UNKNOWN1": 0,
    "STATIONARY": standstill and not gas_override,
    "STANDSTILL_REQ": standstill and not gas_override,
    # 5 = Standstill, 3 = Accelerate, 4 = Brake, 1 = Maintain speed
    "MOTION_CONTROL": 5 if standstill else 3 if accel > 0 else 4 if accel < 0 else 1
  }

  dat = packer.make_can_msg("ACC_CMD", 0, values)[2]
  values["CHECKSUM"] = get_crc8_8h2f(dat[:-1])

  return packer.make_can_msg("ACC_CMD", 0, values)

def send_buttons(packer, count, send_cruise):
  """Spoof ACC Button Command."""

  if send_cruise:
   values = {
      "NEW_SIGNAL_1": 1,
      "CRUISE_BTN": 1,
      "SET_ME_BUTTON_PRESSED": 1,
      "COUNTER": count,
    }
  else:
    values = {
      "SET_BUTTON": 0,
      "RES_BUTTON": 1,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 1,
      "SET_ME_BUTTON_PRESSED": 1,
      "COUNTER": count,
    }

  dat = packer.make_can_msg("ACC_BUTTONS", 0, values)[2]
  values["CHECKSUM"] = get_crc8_8h2f(dat[:-1])

  return packer.make_can_msg("ACC_BUTTONS", 0, values)

init_lut_crc8_8h2f()
