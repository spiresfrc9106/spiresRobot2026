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

from wpilib import RobotBase, RobotController

from teamNumber import FRC_TEAM_NUMBER
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


"""
Specifc robots this codebase might run on.
"""
RobotTypes = Enum(
    "RobotTypes",
    [
        "Main",
        "Practice",
        "TestBoard",
        "Spires2023",
        "Spires2026",
        "Spires2026Sim",
        "SpiresMain",
        "SpiresPractice",
        "SpiresTestBoard",
        "SpiresRoboRioV1",
    ],
)


class RobotIdentification(metaclass=Singleton):
    """
    While we strive for our practice robot and main/competition robots to be as identical as possible,
    that's not always the case.
    The goal of this class is to identify which robot is currently running the code.
    The constants between practice and main robots may be different.
    """

    def __init__(self) -> None:
        self.robotType: RobotTypes | None = None

        # Deferred import to avoid circular dependency
        from utils.faults import Fault

        self.serialFault = Fault("RoboRIO serial number not recognized")
        self.serialNumber: str | None = None
        self._configureValue()

        simulationStr = "Simulation" if RobotBase.isSimulation() else "Real"
        print(":::::::::::")
        print(
            f"::::::::::: _RobotIdentification: {simulationStr} {self.getRobotType()} serialNumber:{self.serialNumber}"
        )
        print(":::::::::::")

    def _configureValue(self):

        self.serialFault.setNoFault()
        self.serialNumber = RobotController.getSerialNumber()

        if FRC_TEAM_NUMBER == 9106 and RobotBase.isSimulation():
            self.robotType = RobotTypes.Spires2026Sim
        elif self.serialNumber == "030e2cb0":
            # Test to see if the RoboRio serial number is the main/"Production" bot.
            self.robotType = RobotTypes.Main
        elif (
            self.serialNumber == "03064e3f"
            or FRC_TEAM_NUMBER == 1736
            and RobotBase.isSimulation()
        ):
            # Test to see if the RoboRio serial number is the practice bot.
            self.robotType = RobotTypes.Practice
        elif self.serialNumber == "0316b37c":
            # Test to see if the RoboRio serial number is our testboard's serial number.
            self.robotType = RobotTypes.TestBoard
        elif self.serialNumber == "032430C5":
            self.robotType = RobotTypes.Spires2023
        elif self.serialNumber == "032B1F4B":
            self.robotType = RobotTypes.Spires2026
        elif self.serialNumber == "032B1FBB":
            self.robotType = RobotTypes.SpiresTestBoard
        elif self.serialNumber == "03057ab7":
            self.robotType = RobotTypes.SpiresRoboRioV1
        else:
            # If the Robo Rio's serial number is not equal to any of our known serial numbers,
            # assume we are the main robot. But, throw a fault, since this is something software
            # team needs to fix.
            self.robotType = RobotTypes.SpiresTestBoard
            self.serialFault.setFaulted()
            assert False

    def getRobotType(self) -> RobotTypes | None:
        """
        Return which robot we're running on right now
        """
        return self.robotType

    def getRobotTypeStr(self) -> str:
        """
        Return which robot we're running on right now
        """
        if self.robotType is None:
            return ""
        return self.robotType.name


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
        self._update()

    def _update(self):
        kSimMode = (
            RobotModes.REPLAY if self._logPath is not None else RobotModes.SIMULATION
        )
        self._kRobotMode = RobotModes.REAL if RobotBase.isReal() else kSimMode
        print(f"In LoggerState: pid={os.getpid()} kRobotMode={self.kRobotMode}")

    @property
    def kRobotMode(self):
        return self._kRobotMode


kTuningMode: bool = False
