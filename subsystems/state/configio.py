from enum import Enum
from dataclasses import dataclass, field
from pykit.autolog import autolog

from wpilib import RobotBase, RobotController

from teamNumber import FRC_TEAM_NUMBER
from utils.singleton import Singleton

"""
Specifc robots this codebase might run on.
"""
RobotTypes = Enum('RobotTypes', [
    'Main',
    'Practice',
    'TestBoard',
    'Spires2023',
    'Spires2026',
    'Spires2026Sim',
    'SpiresMain',
    'SpiresPractice',
    'SpiresTestBoard',
    'SpiresRoboRioV1',
])

class _RobotIdentification(metaclass=Singleton):
    """
    While we strive for our practice robot and main/competition robots to be as identical as possible,
    that's not always the case.
    The goal of this class is to identify which robot is currently running the code.
    The constants between practice and main robots may be different.
    """

    def __init__(self):
        self.roboControl = RobotController
        self.robotType: RobotTypes|None = None

        # Deferred import to avoid circular dependency
        from utils.faults import Fault

        self.serialFault = Fault("RoboRIO serial number not recognized")
        self.serialNumber: str|None = None
        self._configureValue()

        simulationStr = "Simulation" if RobotBase.isSimulation() else "Real"
        print(f":::::::::::")
        print(f"::::::::::: _RobotIdentification: {simulationStr} {self.getRobotType()} serialNumber:{self.serialNumber}")
        print(f":::::::::::")


    def _configureValue(self):

        self.serialFault.setNoFault()
        self.serialNumber = RobotController.getSerialNumber()

        if FRC_TEAM_NUMBER == 9106 and RobotBase.isSimulation():
            self.robotType = RobotTypes.Spires2026Sim
        elif self.serialNumber == "030e2cb0":
            #Test to see if the RoboRio serial number is the main/"Production" bot.
            self.robotType = RobotTypes.Main
        elif self.serialNumber == "03064e3f" \
                or FRC_TEAM_NUMBER==1736 and RobotBase.isSimulation():
            #Test to see if the RoboRio serial number is the practice bot.
            self.robotType = RobotTypes.Practice
        elif self.serialNumber == "0316b37c":
            #Test to see if the RoboRio serial number is our testboard's serial number.
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

    def getRobotType(self)->RobotTypes:
        """
        Return which robot we're running on right now
        """
        return self.robotType

    def getRobotTypeStr(self)->RobotTypes:
        """
        Return which robot we're running on right now
        """
        return self.robotType.name

    @classmethod
    def isSpiresRobot(cls, robotType: RobotTypes)->bool:
        return str(robotType).startswith('RobotTypes.Spires')

class ConfigIO:
    """Process I/O data for the robot high-level state subsystem."""

    @autolog
    @dataclass
    class ConfigIOInputs:
        """Hold I/O data for the robot high-level state subsystem."""
        robotTypeStr: str = field(default_factory=lambda: _RobotIdentification().getRobotTypeStr())

    def updateInputs(self, inputs: ConfigIOInputs) -> None:
        """Update the robot high-level state I/O inputs.

        Args:
            inputs (ConfigIOInputs): The robot high-level state I/O inputs to update.
        """
        pass


    @classmethod
    def getRobotType(cls, inputs: ConfigIOInputs)->RobotTypes:
        return RobotTypes[inputs.robotTypeStr]

    @classmethod
    def isSpiresRobot(cls, inputs: ConfigIOInputs)->bool:
        return _RobotIdentification.isSpiresRobot(cls.getRobotType(inputs))
