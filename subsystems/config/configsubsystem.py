from enum import Enum
from typing_extensions import Self
from commands2 import Subsystem
from pykit.logger import Logger
from pykit.autolog import autolog_output, autologgable_output

from drivetrain.drivetrainDependentConstants import DrivetrainDependentConstants, CameraDependentConstants
from subsystems.config.configio import ConfigIO, RobotTypes
from utils.faults import Fault
from utils.singleton import _instances, Singleton
from westwood.constants import kRobotMode
from westwood.util.logtracer import LogTracer


from wpilib import RobotController, RobotBase
from teamNumber import FRC_TEAM_NUMBER




# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class ConfigSubsystem(Subsystem):
    _initalized = False

    def __new__(cls, *arg, **kwargs) -> Self:
        if cls not in _instances:
            instance = super().__new__(cls)
            _instances[cls] = instance
        return _instances[cls]

    def __init__(self) -> None:
        if self._initalized:
            return
        self.io = ConfigIO()
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.inputs = ConfigIO.ConfigIOInputs()


        # Call periodic early so we can load self.inputs.robotType before the other Subsytems
        # are created. So that in replay, they can replay as their configured type.
        self.periodic()
        self._robotType: RobotTypes = ConfigIO.getRobotType(self.inputs)
        self._isSpiresRobot: bool = ConfigIO.isSpiresRobot(self.inputs)
        self.drivetrainDepConstants = DrivetrainDependentConstants().get(self._robotType)
        self.cameraDepConstants = CameraDependentConstants().get(self._robotType)
        print(f":::::::::::")
        print(f"::::::::::: ConfigSubsystem: {kRobotMode} {self._robotType}")
        print(f":::::::::::")
        self._initalized = True

    @classmethod
    def getInstance(cls):
        return cls

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's configuration
        """

        LogTracer.resetOuter("ConfigSubsystemPeriodic")
        self.io.updateInputs(self.inputs)
        LogTracer.record("IOUpdate")
        """TODO put any configuation IOUpdates here"""
        LogTracer.record("StateUpdate")
        Logger.processInputs("Config", self.inputs)
        LogTracer.record("LoggerProcessInputs")

        """TODO put any configuation periodic here"""
        LogTracer.record("ModulesPeriodic")
        LogTracer.recordTotal()

    def getRobotType(self) -> RobotTypes:
        return self._robotType

    def isSpiresRobot(self) -> bool:
        return self._isSpiresRobot

    def useCasseroleSwerve(self) -> bool:
        return self.drivetrainDepConstants["HAS_DRIVETRAIN"] and DrivetrainDependentConstants().useCasseroleSwerve

    def useWestwoodSwerve(self) -> bool:
        return self.drivetrainDepConstants["HAS_DRIVETRAIN"] and DrivetrainDependentConstants().useWestwoodSwerve

