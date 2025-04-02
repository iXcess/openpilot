#from cereal import car
from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.proton.protoncan import create_can_steer_command, send_buttons
from openpilot.selfdrive.car.proton.values import DBC
from openpilot.common.numpy_fast import clip

def apply_proton_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + driver_torque * 30
  driver_min_torque = -LIMITS.STEER_MAX + driver_torque * 30
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

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.params = CarControllerParams(self.CP)

    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    #latActive = enabled
    actuators = CC.actuators
    #lead_visible = CC.hudControl.leadVisible
    #rlane_visible = CC.hudControl.rightLaneVisible
    #llane_visible = CC.hudControl.leftLaneVisible
    #ldw = CC.hudControl.leftLaneDepart or CC.hudControl.rightLaneDepart

    lat_active = enabled

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    # TODO use openpilot's ready function
    apply_steer = apply_proton_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    # CAN controlled lateral running at 50hz
    if (self.frame % 2) == 0:
      can_sends.append(create_can_steer_command(self.packer, apply_steer, \
      lat_active, CS.hand_on_wheel_warning and CS.is_icc_on, \
      CS.lks_aux, CS.lks_audio, CS.lks_tactile, CS.lks_assist_mode, CS.lka_enable, CS.stock_ldw))

      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, rlane_visible, llane_visible))
      #can_sends.append(create_lead_detect(self.packer, lead_visible, enabled))
      #can_sends.append(create_acc_cmd(self.packer, actuators.accel, fake_enable))

    if CS.out.standstill and enabled and (self.frame % 29 == 0):
      # Spam resume button to resume from standstill at max freq of 34.48 Hz.
      if CS.acc_req:
        can_sends.append(send_buttons(self.packer, False))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends
