#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.byd.values import CAR

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "byd"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.01          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]
    ret.lateralTuning.pid.kiBP = [0., 5., 20.]
    ret.lateralTuning.pid.kpBP = [0., 5., 20.]

    if candidate == CAR.ATTO3:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.32, 0.23, 0.12], [1.5, 1.3, 1.0]]
      ret.lateralTuning.pid.kf = 0.00015

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0.15, 0.6, 0.7]
      ret.longitudinalTuning.kiBP = [5, 7, 28]
      ret.longitudinalTuning.kiV = [0.15, 0.26, 0.26]
      ret.longitudinalActuatorDelayLowerBound = 0.3
      ret.longitudinalActuatorDelayUpperBound = 0.4

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.05 # reach stopping target smoothly

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp)

    # events
    events = self.create_common_events(ret)

    ret.events = events.to_msg()

    return ret

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
