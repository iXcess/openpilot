from openpilot.common.numpy_fast import clip
from typing import List

def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, lks_aux, lks_audio, \
    lks_tactile, lks_assist_mode, lka_enable, stock_ldw):

  values = {
    "LKA_ENABLE": lka_enable,
    "LKAS_ENGAGED1": steer_req,
    "LKAS_LINE_ACTIVE": steer_req,
    "STEER_CMD": abs(steer) if steer_req else 0,
    "STEER_DIR": steer <= 0,
    # Disable steering vibration for LDW if LKS set to Warn Only mode and Tactile feedback type
    "LKS_LDW": 0 if (not steer_req and (not lks_aux and lks_assist_mode) and (lks_tactile and not lks_audio)) else stock_ldw,
    "SET_ME_1_2": 1,
    "LKS_STATUS": 1,
    "STOCK_LKS_AUX": lks_aux,
    "LKS_WARNING_AUDIO": lks_audio,
    "LKS_WARNING_TACTILE": lks_tactile,
    "LKS_ASSIST_MODE": lks_assist_mode,
    "HAND_ON_WHEEL_WARNING": wheel_touch_warning,
    "WHEEL_WARNING_CHIME": wheel_touch_warning,
  }

  return packer.make_can_msg("ADAS_LKAS", 0, values)

def create_hud(packer, steer, steer_req, ldw, rlane, llane):
  steer_dir = steer >= 0
  values = {
    "LANE_DEPARTURE_WARNING_RIGHT": ldw and not steer_dir,
    "LANE_DEPARTURE_WARNING_LEFT": ldw and steer_dir,
    "LEFT_LANE_VISIBLE_DISENGAGE": 0,
    "RIGHT_LANE_VISIBLE_DISENGAGE": 0,
    "STEER_REQ_RIGHT": steer_req,
    "STEER_REQ_LEFT": steer_req,
    "STEER_REQ_MAJOR": 1 if steer_req else 0,
    "LLANE_CHAR": 0x91 if steer_req else 0x4b,
    "CURVATURE": 0x3f if steer_req else 0x3f,
    "RLANE_CHAR": 0xaa if steer_req else 0x3d,
  }

  return packer.make_can_msg("LKAS", 0, values)

def create_lead_detect(packer, is_lead, steer_req):
  values = {
    "LEAD_DISTANCE": 30,
    "NEW_SIGNAL_1": 0x7f,
    "NEW_SIGNAL_2": 0x7e,
    "IS_LEAD2": is_lead,
    "IS_LEAD1": is_lead,
    "LEAD_TOO_NEAR": 0,
  }

  return packer.make_can_msg("ADAS_LEAD_DETECT", 0, values)

def create_pcm(packer, steer, steer_req, raw_cnt):
  values = {
    "ACC_SET_SPEED": 0x23 if steer_req else 0,
    "SET_DISTANCE": 1 if steer_req else 0,
    "NEW_SIGNAL_1": 3,
    "ACC_SET": 1 if steer_req else 0,
    "ACC_ON_OFF_BUTTON": 1,
  }

  values["CHECKSUM"] = crc

  return packer.make_can_msg("PCM_BUTTONS", 0, values)

def create_acc_cmd(packer, accel, enabled, gas_override, standstill):
  # Does not work because it can keep car standstill but cannot make car move when lead car moves
  # keep_standstill = enabled and standstill and accel < 0.21
  keep_standstill = False
  accel_cmd = -31 if keep_standstill else accel * 10
  values = {
    "CMD": accel_cmd,
    "CMD_OFFSET1": accel_cmd,
    "CMD_OFFSET2": accel_cmd,
    "ACC_REQ": enabled,
    "NOT_ACC_REQ": not enabled,
    "SET_ME_1": 1,
    "CRUISE_ENABLE": enabled and not gas_override,

    # not sure
    "BRAKE_ENGAGED": 0,
    "SET_ME_X6A": 0x6A,
    "RISING_ENGAGE": gas_override,
    "UNKNOWN1": 0,
    "STATIONARY": keep_standstill,
    "STANDSTILL2": keep_standstill or gas_override,
    # 5 = Keep standstill, 3 = Accelerate, 4 = Brake, 1 = Maintain speed
    "MOTION_CONTROL": 5 if keep_standstill else 3 if accel > 0 else 4 if accel < 0 else 1
  }

  return packer.make_can_msg("ACC_CMD", 0, values)

def send_buttons(packer, count, send_cruise):
  """Spoof ACC Button Command."""

  if send_cruise:
   values = {
      "NEW_SIGNAL_1": 1,
      "CRUISE_BTN": 1,
      "SET_ME_BUTTON_PRESSED": 1,
    }
  else:
    values = {
      "SET_BUTTON": 0,
      "RES_BUTTON": 1,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 1,
      "SET_ME_BUTTON_PRESSED": 1,
    }

  return packer.make_can_msg("ACC_BUTTONS", 0, values)
