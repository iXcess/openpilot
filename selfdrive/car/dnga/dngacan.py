from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV

def lkc_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 1 + sum(dat)) & 0xFF

def dnga_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""

  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = lkc_checksum(0x1d0, dat[:-1])
  values["CHECKSUM"] = crc


  return packer.make_can_msg("STEERING_LKAS", 0, values)

def aeb_brake_command(packer, enabled, decel_cmd):

  decel_req = enabled

  values = {
    "AEB_PUMP_HOLD": 0xfe if (enabled and decel_req) else 0,
    "MAGNITUDE": 0x5a if (enabled and decel_req) else 0,
    "BRAKE_REQ": 0x10 if (enabled and decel_req) else 0,
    "SET_ME_XE5": 0x0 if (enabled and decel_req) else 0,
    "SET_ME_X1B": 0x0 if (enabled and decel_req) else 0,
  }

  dat = packer.make_can_msg("ADAS_AEB", 0, values)[2]
  crc = (dnga_checksum(680, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ADAS_AEB", 0, values)

def dnga_create_brake_command(packer, enabled, decel_req, pump, decel_cmd, aeb, idx):

  # Value overflow check
  # MAGNITUDE a max value 2.0 to prevent overflow, maximum seen on porto is 1.56
  # PUMP_REACTION{N} has a max value of 1.2, maximum seen on porto is 1.0
  decel_req = clip(decel_req, 0., 0.5)
  pump = clip(pump, 0., 1.0)

  values = {
    "COUNTER": idx,
    "PUMP_REACTION1": pump if enabled else 0,
    "BRAKE_REQ": decel_req and enabled,
    "MAGNITUDE": (-1* decel_cmd) if (enabled and decel_req) else 0,
    "SET_ME_1_WHEN_ENGAGE": 1 if enabled else 0,
    "PUMP_REACTION2": -1* pump if enabled else 0,
    "AEB_REQ1": 1 if aeb else 0,
    "AEB_REQ2": 1 if aeb else 0,
    "AEB_REQ3": 1 if aeb else 0,
    "AEB_1019": aeb,
  }

  dat = packer.make_can_msg("ACC_BRAKE", 0, values)[2]
  crc = (dnga_checksum(0x271, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_BRAKE", 0, values)

def dnga_create_accel_command(packer, set_speed, acc_rdy, enabled, is_lead, des_speed, brake_amt, brake_pump, distance_val):
  is_braking = (brake_amt > 0.0 or brake_pump > 0.0)

  values = {
    "SET_SPEED": set_speed * CV.MS_TO_KPH,
    "FOLLOW_DISTANCE": distance_val,
    "IS_LEAD": is_lead,
    "IS_ACCEL": (not is_braking) and enabled,
    "IS_DECEL": is_braking and enabled,
    "SET_ME_1_2": acc_rdy, #rdy buton
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": not enabled,
    "SET_1_WHEN_ENGAGE": enabled,
    "ACC_CMD": des_speed * CV.MS_TO_KPH if enabled else 0,
  }

  dat = packer.make_can_msg("ACC_CMD_HUD", 0, values)[2]
  crc = (dnga_checksum(0x273, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_CMD_HUD", 0, values)

def dnga_create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible, ldw, fcw, aeb, front_depart, ldp_off, fcw_off):

  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": ldw,
    "LDA_OFF": ldp_off,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
    "AEB_ALARM": fcw,
    "AEB_BRAKE": aeb,
    "FRONT_DEPART": front_depart,
    "FCW_DISABLE": fcw_off,
  }

  dat = packer.make_can_msg("LKAS_HUD", 0, values)[2]
  crc = (dnga_checksum(0x274, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("LKAS_HUD", 0, values)

def dnga_buttons(packer, set_button, res_button, counter):

  values = {
    "SET_MINUS": set_button,
    "RES_PLUS" : res_button,
    "ACC_RDY" : 1,
    "PEDAL_DEPRESSED": 1,
    "NEW_SIGNAL_1": 1,
    "NEW_SIGNAL_2": 1,
    "COUNTER" : counter,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[2]
  crc = (dnga_checksum(520, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("PCM_BUTTONS", 0, values)

