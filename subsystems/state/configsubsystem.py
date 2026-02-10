from typing_extensions import Self
from commands2 import Subsystem
from pykit.logger import Logger
from pykit.autolog import autolog_output, autologgable_output

from subsystems.state.configio import ConfigIO, RobotTypes
from utils.singleton import _instances
from westwood.constants import kRobotMode
from westwood.util.logtracer import LogTracer






# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class ConfigSubsystem(Subsystem):
    """
    A singleton class that records the robot's configuration.
    """
    _initalized = False

    # Because this is a singleton, we need to override __new__ to return the same instance every time.
    # We had to do this differently than utils.singleton.Singleton because we need to subclass
    # Subsystem, which doesn't allow us to use class ConfigSubsystem(Subsystem, metaclass=Singleton).
    # Note that utils.singleton.Singleton uses def __call__(self, *args, **kwargs) -> Self:, while we need __new__,
    # because Subsystem has a custom __new__ method.
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

        # Deferred import to avoid circular dependency
        from drivetrain.drivetrainDependentConstants import DrivetrainDependentConstants, CameraDependentConstants
        self.dpc = DrivetrainDependentConstants()
        self.drivetrainDepConstants = self.dpc.get(self._robotType)
        self.cameraDepConstants = CameraDependentConstants().get(self._robotType)

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
        return self.drivetrainDepConstants["HAS_DRIVETRAIN"] and self.dpc.useCasseroleSwerve

    def useWestwoodSwerve(self) -> bool:
        return self.drivetrainDepConstants["HAS_DRIVETRAIN"] and self.dpc.useWestwoodSwerve

    def getFPGATimeUS(self) -> int:
        """The time of the current robot periodic loop in microseconds."""
        return self.inputs.timeUSec

    def getFPGATimestampS(self) -> float:
        """The time of the current robot periodic loop in seconds."""
        return self.inputs.timeUSec / 1.0e6

