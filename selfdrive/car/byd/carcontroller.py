from opendbc.can.packer import CANPacker

from openpilot.selfdrive.car.interfaces import CarControllerBase

from openpilot.selfdrive.car.byd.bydcan import create_can_steer_command, send_buttons, create_lkas_hud
from openpilot.selfdrive.car.byd.values import DBC
from openpilot.common.numpy_fast import clip

RES_INTERVAL = 130
RES_LEN = 3
SNG_WAIT = 500

def apply_byd_steer_angle_limits(apply_angle, actual_angle, v_ego, LIMITS):
  # pick angle rate limits based on wind up/down
  steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
  rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN

  return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)

class CarControllerParams():
  def __init__(self, CP):
    self.ANGLE_RATE_LIMIT_UP = 3       # maximum allow 150 degree per second, 100Hz loop means 1.5
    self.ANGLE_RATE_LIMIT_DOWN = 3

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.params = CarControllerParams(self.CP)

    self.steer_rate_limited = False
    self.lka_active = False
    self.last_res_press_frame = 0 # The frame where the last resume press was finished
    self.resume_counter = 0       # Counter for tracking the progress of a resume press
    self.last_sng_stop_frame = 0
    self.is_sng_stop = False

  def update(self, CC, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators

    # steer
    apply_angle = apply_byd_steer_angle_limits(actuators.steeringAngleDeg, CS.out.steeringAngleDeg, CS.out.vEgo, self.params)
    self.steer_rate_limited = (abs(apply_angle - CS.out.steeringAngleDeg) > 2.5)

    # BYD CAN controlled lateral running at 50hz
    if (self.frame % 2) == 0:

      # logic to activate and deactivate lane keep, cannot tie to the lka_on state because it will occasionally deactivate itself
      if CS.lka_on:
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False

      if CS.out.steeringTorqueEps > 15:
        apply_angle = CS.out.steeringAngleDeg

      lat_active = enabled and abs(CS.out.steeringAngleDeg) < 90 and \
      self.lka_active and not CS.out.standstill # temporary hardcode 60 because if 90 degrees it will fault
      # brake_hold = False
      # TODO laneActive, used to check if ALC is off
      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame/2) % 16))
#      can_sends.append(create_accel_command(self.packer, actuators.accel, enabled, brake_hold, (self.frame/2) % 16))
      can_sends.append(create_lkas_hud(self.packer, enabled, CS.lss_state, CS.lss_alert, CS.tsr, \
      CS.abh, CS.passthrough, CS.HMA, CS.pt2, CS.pt3, CS.pt4, CS.pt5, self.lka_active, self.frame % 16))

    # For resume

    # If SNG stop
    if not self.is_sng_stop and (CS.out.standstill or CS.out.cruiseState.standstill):
      self.is_sng_stop = True
      self.last_sng_stop_frame = self.frame
    # If SNG moved
    if self.is_sng_stop and not (CS.out.standstill or CS.out.cruiseState.standstill):
      self.is_sng_stop = False

    if (CS.out.standstill or CS.out.cruiseState.standstill) and enabled and \
        self.resume_counter == 0 and self.frame > (self.last_res_press_frame + RES_INTERVAL) \
        and self.frame > (self.last_sng_stop_frame + SNG_WAIT):
      # Only start a new resume if the last one was finished, with an interval
      self.resume_counter = 1 # Start a new resume press

    if self.resume_counter > 0 and self.resume_counter <= RES_LEN and \
        (CS.out.standstill or CS.out.cruiseState.standstill):
      # Send resume press signal
      can_sends.append(send_buttons(self.packer, 1, (CS.counter_pcm_buttons + 1) % 16))
      self.resume_counter += 1

    if self.resume_counter > RES_LEN or not (CS.out.standstill or CS.out.cruiseState.standstill):
      # If resume press is finished or car is moving
      self.last_res_press_frame = self.frame # Store the frame where last resume press was finished
      self.resume_counter = 0 # Reset resume counter

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends