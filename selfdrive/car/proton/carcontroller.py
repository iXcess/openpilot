from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.proton.protoncan import create_can_steer_command, create_hud, create_lead_detect, send_buttons, create_acc_cmd
from selfdrive.car.proton.values import CAR, DBC, BRAKE_SCALE, GAS_SCALE
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from common.params import Params
import cereal.messaging as messaging

def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + driver_torque * 25
  driver_min_torque = -LIMITS.STEER_MAX + driver_torque * 25
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.num_cruise_btn_sent = 0
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    hud_control = CC.hudControl
    lat_active = CC.latActive

    if self.frame <= 1000 and CS.out.cruiseState.available and self.num_cruise_btn_sent <= 5:
      self.num_cruise_btn_sent += 1
      can_sends.append(send_buttons(self.packer, self.frame % 16, True))

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorque, self.params)

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    ts = self.frame * DT_CTRL

    # CAN controlled lateral running at 50hz
    if (self.frame % 2) == 0:
      if CS.stock_ldp:
        steer_dir = -1 if CS.steer_dir else 1
        apply_steer = CS.stock_ldp_cmd * steer_dir
        lat_active |= True
      can_sends.append(create_can_steer_command(self.packer, apply_steer, lat_active, CS.hand_on_wheel_warning and CS.is_icc_on, (self.frame/2) % 16, CS.stock_lks_settings,  CS.stock_lks_settings2))

      can_sends.append(create_acc_cmd(self.packer, actuators.accel, CC.longActive, (self.frame/2) % 16))

    if CS.out.standstill and CC.longActive and (self.frame % 50 == 0):
      # Spam resume button to resume from standstill at max freq of 10 Hz.
      can_sends.append(send_buttons(self.packer, self.frame % 16, False))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    self.frame += 1
    return new_actuators, can_sends
