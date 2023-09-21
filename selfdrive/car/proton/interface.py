#!/usr/bin/env python3
from cereal import car
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.proton.values import CAR
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

from common.params import Params

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "proton"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.proton)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.enableDsu = False                  # driving support unit

    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.2          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.X50:
      ret.wheelbase = 2.6
      ret.steerRatio = 15.00
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1370. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.048

      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [550]]

      ret.lateralTuning.pid.kpBP = [0., 10., 30., 40.]
      ret.lateralTuning.pid.kpV = [0.05, 0.10, 0.15, 0.16]
      ret.lateralTuning.pid.kiBP = [0., 20., 30.]
      ret.lateralTuning.pid.kiV = [0.08, 0.30, 0.45]
      ret.lateralTuning.pid.kf = 0.00008

      ret.longitudinalTuning.kpBP = [0., 4., 20.]
      ret.longitudinalTuning.kpV = [1.2, 1.0, 0.4]
      ret.longitudinalActuatorDelayLowerBound = 0.2
      ret.longitudinalActuatorDelayUpperBound = 0.3

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kiV = [.32, .20, .07]

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.02 # reach stopping target smoothly

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid

    # events
    events = self.create_common_events(ret)
    ret.events = events.to_msg()

    return ret

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
