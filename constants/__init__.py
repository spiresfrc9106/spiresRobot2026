"""

The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

from enum import Enum
import os

from wpilib import RobotBase

from utils.singleton import Singleton

kRobotUpdatePeriodMs: int = 50
kRobotUpdatePeriodS: float = kRobotUpdatePeriodMs / 1000
kRobotUpdateFrequency: float = 1 / kRobotUpdatePeriodS

kFieldWidthWeldedIn = 317.688
kFieldLengthWeldedIn = 651.22

kFieldWidthIn: float = kFieldWidthWeldedIn
kFieldLengthIn: float = kFieldLengthWeldedIn


class RobotModes(Enum):
    """Enum for robot modes."""

    REAL = 1
    SIMULATION = 2
    REPLAY = 3


class LoggerState(metaclass=Singleton):
    def __init__(self):
        self.logPath = (
            os.environ["LOG_PATH"]
            if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
            else None
        )

    @property
    def logPath(self):
        return self._logPath

    @logPath.setter
    def logPath(self, path):
        self._logPath = path
        self.update()

    def update(self):
        kSimMode = (
            RobotModes.REPLAY if self._logPath is not None else RobotModes.SIMULATION
        )
        self._kRobotMode = RobotModes.REAL if RobotBase.isReal() else kSimMode
        print(f"In LoggerState: pid={os.getpid()} kRobotMode={self.kRobotMode}")

    @property
    def kRobotMode(self):
        return self._kRobotMode


kTuningMode: bool = False
