from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton

class VisionDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.inOutDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_VISION": False,
            },
            RobotTypes.Spires2026: {
                "HAS_VISION": True,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_VISION": True,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_VISION": False,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_VISION": False,
            },
        }
    def get(self, robotType: RobotTypes):
        return self.inOutDepConstants[robotType]

