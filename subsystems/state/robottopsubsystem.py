from typing_extensions import Self
from commands2 import Subsystem
from wpimath.geometry import Pose2d, Rotation2d
from wpimath import angleModulus

from constants.field import kFarHubLocation, kCloseHubLocation
from pykit.logger import Logger
from pykit.autolog import autologgable_output

from subsystems.state.robottopio import RobotTopIO
from utils.allianceTransformUtils import onRed
from utils.singleton import _instances
from util.logtracer import LogTracer
from utils.units import m2in, rad2Deg


# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class RobotTopSubsystem(Subsystem):
    """
    A singleton class that records the robot's high-level state.
    """

    _initalized = False

    # Because this is a singleton, we need to override __new__ to return the same instance every time.
    # We had to do this differently than utils.singleton.Singleton because we need to subclass
    # Subsystem, which doesn't allow us to use class RobotTopSubsystem(Subsystem, metaclass=Singleton).
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
        self.io = RobotTopIO()
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.inputs = RobotTopIO.RobotTopIOInputs()

        self.robotPose: Pose2d | None = None

        self._initalized = True

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's configuration
        """

        LogTracer.resetOuter("RobotTopSubsystemPeriodic")
        self.io.updateInputs(self.inputs)
        LogTracer.record("IOUpdate")
        """put any RobotTop IOUpdates here"""
        LogTracer.record("StateUpdate")
        Logger.processInputs("RobotTop", self.inputs)
        LogTracer.record("LoggerProcessInputs")

        """put any RobotTop periodic here"""
        Logger.recordOutput("Robot/top/distanceToHubIn", m2in(self.getDistanceToHubM()))
        Logger.recordOutput("Robot/top/angleToHubDeg", rad2Deg(self.getAngleToHubRad()))
        LogTracer.record("ModulesPeriodic")
        LogTracer.recordTotal()

    def getFPGATimeUS(self) -> int:
        """The time of the current robot periodic loop in microseconds."""
        return self.inputs.timeUSec

    def getFPGATimestampS(self) -> float:
        """The time of the current robot periodic loop in seconds."""
        return self.inputs.timeUSec / 1.0e6

    def setRobotPose(self, pose: Pose2d) -> None:
        self.robotPose = pose

    def hubLocation(self):
        return kFarHubLocation if onRed() else kCloseHubLocation

    def getDistanceToHubM(self) -> float:
        result = 0.0
        if self.robotPose is not None:
            result = self.robotPose.translation().distance(self.hubLocation())
        return result

    def getRotationToHub(self) -> Rotation2d | None:
        rotationToTarget = None
        if self.robotPose is not None:
            bearingToTarget = (
                self.hubLocation() - self.robotPose.translation()
            ).angle()
            rotationToTarget = (
                bearingToTarget
                - self.robotPose.rotation()
                + Rotation2d.fromDegrees(180.0)
            )
        return rotationToTarget

    def getAngleToHubRad(self) -> float:
        rotation = self.getRotationToHub()
        angleToHubRad = 0.0
        if rotation is not None:
            angleToHubRad = angleModulus(rotation.radians())
        return angleToHubRad
