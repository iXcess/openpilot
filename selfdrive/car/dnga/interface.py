#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.dnga.values import CAR

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "dnga"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.dnga)]
    ret.safetyConfigs[0].safetyParam = 1   # TODO: add actual safetyParam

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]

    if candidate == CAR.ALZA:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]
      ret.lateralTuning.pid.kf = 0.00015

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0.15, 0.6, 0.7]
      ret.longitudinalTuning.kiBP = [5, 7, 28]
      ret.longitudinalTuning.kiV = [0.15, 0.26, 0.26]
      ret.longitudinalActuatorDelayLowerBound = 0.42
      ret.longitudinalActuatorDelayUpperBound = 0.60
      # TODO: ret.speedControlled = True

    elif candidate == CAR.ATIVA:

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.22]]
      ret.lateralTuning.pid.kf = 0.000188

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0.6, 0.5, 0.05]
      ret.longitudinalTuning.kiBP = [5, 7, 28]
      ret.longitudinalTuning.kiV = [0.15, 0.14, 0.01]
      ret.longitudinalActuatorDelayLowerBound = 0.32
      ret.longitudinalActuatorDelayUpperBound = 0.40
      # TODO: ret.speedControlled = True

    elif candidate == CAR.MYVI:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.20]]
      ret.lateralTuning.pid.kf = 0.00012

      ret.longitudinalTuning.kpBP = [0., 5., 20., 30.]
      ret.longitudinalTuning.kpV = [0.5, 0.5, 0.4, 0.3]
      ret.longitudinalTuning.kiBP = [5, 7, 20, 30]
      ret.longitudinalTuning.kiV = [0.11, 0.1, 0.08, 0.07]
      ret.longitudinalActuatorDelayLowerBound = 0.32
      ret.longitudinalActuatorDelayUpperBound = 0.40
      # TODO: ret.speedControlled = True

    elif candidate == CAR.VIOS:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]
      ret.lateralTuning.pid.kf = 0.00018

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0.65, 0.6, 0.6]
      ret.longitudinalTuning.kiBP = [5, 7, 28]
      ret.longitudinalTuning.kiV = [0.12, 0.12, 0.12]
      ret.longitudinalActuatorDelayLowerBound = 0.42
      ret.longitudinalActuatorDelayUpperBound = 0.60
      # TODO: ret.speedControlled = True

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.25 # reach stopping target smoothly

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
