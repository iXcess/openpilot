from dataclasses import dataclass, field
from cereal import car
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo

# Todo
HUD_MULTIPLIER = 1.035
Ecu = car.CarParams.Ecu

@dataclass
class ProtonPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('proton_general_pt', None))

class CANBUS:
  main_bus = 0
  cam_bus = 1

class CAR(Platforms):
  S70 = ProtonPlatformConfig(
    'PROTON S70',
    CarInfo("Proton S70", "All"),
    specs=CarSpecs(mass=1300., wheelbase=2.627, steerRatio=15.0)
  )
  X50 = ProtonPlatformConfig(
    "PROTON X50",
    CarInfo("Proton X50", "All"),
    specs=CarSpecs(mass=1370., wheelbase=2.6, steerRatio=15.0)
  )
  X90 = ProtonPlatformConfig(
    "PROTON X90",
    CarInfo("Proton X90", "All"),
    specs=CarSpecs(mass=1705., wheelbase=2.805, steerRatio=15.0)
  )

CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
