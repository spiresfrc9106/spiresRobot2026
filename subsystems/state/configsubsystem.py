from typing_extensions import Self
from commands2 import Subsystem
from pykit.autolog import autologgable_output

from constants import RobotTypes
from subsystems.state.robottopio import RobotTopDependentConstants
from subsystems.intakeOuttake.inout import InOutDependentConstants
from subsystems.vision.vision import VisionDependentConstants
from utils.robotLoggerSetup import RobotLoggerSetup
from utils.singleton import _instances
from constants import LoggerState


# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class ConfigSubsystem(Subsystem):
    """
    A singleton class that records the robot's configuration.
    """

    ## todo make this a normal singleton and not a subsystem.

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
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self._robotType = RobotLoggerSetup().robotType

        # Deferred import to avoid circular dependency
        from drivetrain.drivetrainDependentConstants import (
            DrivetrainDependentConstants,
            CameraDependentConstants,
        )

        self.dpc = DrivetrainDependentConstants()
        self.drivetrainDepConstants = self.dpc.get(self._robotType)

        self.robotTopDepConstants = RobotTopDependentConstants().get(self._robotType)
        self.cameraDepConstants = CameraDependentConstants().get(self._robotType)
        self.inoutDepConstants = InOutDependentConstants().get(self._robotType)
        self.visionDepConstants = VisionDependentConstants().get(self._robotType)
        print(":::::::::::")
        print(
            f"::::::::::: ConfigSubsystem: {LoggerState().kRobotMode} {self._robotType}"
        )
        print(":::::::::::")
        self._initalized = True

    @classmethod
    def getInstance(cls):
        return cls

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's configuration
        """
        pass

    def getRobotType(self) -> RobotTypes:
        return self._robotType

    def useCasseroleSwerve(self) -> bool:
        return (
            self.drivetrainDepConstants["HAS_DRIVETRAIN"]
            and self.dpc.useCasseroleSwerve
        )
