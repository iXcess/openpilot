def create_can_steer_command(packer, steer_angle, steer_req, is_standstill):

  set_me_xe = 0xB
  if is_standstill:
    set_me_xe = 0xE

  values = {
    "STEER_REQ": steer_req,
    "STEER_REQ_ACTIVE_LOW": not steer_req,
    "STEER_ANGLE": steer_angle * 1.02,     # desired steer angle
    "SET_ME_X01": 0x1 if steer_req else 0, # must be 0x1 to steer
    # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
    "SET_ME_XE": set_me_xe if steer_req else 0,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    }

  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)

def create_accel_command(packer, accel, enabled, brake_hold):
  accel = max(min(accel * 16.67, 30), -50)

  values = {
    "ACCEL_CMD": accel,
    "SET_ME_25_1": 25,                     # always 25
    "SET_ME_25_2": 25,                     # always 25
    "ACC_ON_1": enabled,
    "ACC_ON_2": enabled,
    "ACCEL_FACTOR": 14 if enabled else 0,   # the higher the value, the more powerful the accel
    "DECEL_FACTOR": 1 if enabled else 0,   # the lower the value, the more powerful the decel
    "SET_ME_X8": 8,
    "SET_ME_1": 1,
    "SET_ME_XF": 0xF,
    "CMD_REQ_ACTIVE_LOW": 0 if enabled else 1,
    "ACC_REQ_NOT_STANDSTILL": enabled,
    "ACC_CONTROLLABLE_AND_ON": enabled,
    "ACC_OVERRIDE_OR_STANDSTILL": 0,       # use this to apply brake hold
    "STANDSTILL_STATE": 0,                 # TODO integrate vEgo check
    "STANDSTILL_RESUME": 0,                # TODO integrate buttons
  }

  return packer.make_can_msg("ACC_CMD", 0, values)

# 50hz
def create_lkas_hud(packer, enabled, lss_state, lss_alert, tsr, ahb, passthrough,\
    hma, pt2, pt3, pt4, pt5, lka_on):

  values = {
    "STEER_ACTIVE_ACTIVE_LOW": lka_on, # not enabled,
    "STEER_ACTIVE_1_1": enabled and lka_on, # Left lane visible
    "STEER_ACTIVE_1_2": enabled and lka_on, # steering wheel between lanes icon, lkas active
    "STEER_ACTIVE_1_3": enabled and lka_on, # Right lane visible
    "LSS_STATE": lss_state,
    "SET_ME_1_2": 1,
    "SETTINGS": lss_alert,
    "SET_ME_X5F": ahb,
    "SET_ME_XFF": passthrough,
    "HAND_ON_WHEEL_WARNING": 0,           # TODO integrate warning signs when steer limited
    "TSR": tsr,
    "HMA": hma,
    "PT2": pt2,
    "PT3": pt3,
    "PT4": pt4,
    "PT5": pt5,
  }

  return packer.make_can_msg("LKAS_HUD_ADAS", 0, values)

def send_buttons(packer, state):
  """Spoof ACC Button Command."""
  values = {
      "SET_BTN": state,
      "RES_BTN": state,
      "SET_ME_1_1": 1,
      "SET_ME_1_2": 1,
  }
  return packer.make_can_msg("PCM_BUTTONS", 0, values)

