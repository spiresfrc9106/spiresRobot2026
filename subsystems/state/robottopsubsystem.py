from typing import Callable

from typing_extensions import Self
from commands2 import Subsystem
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Rotation2d, Pose3d, Transform3d, Rotation3d
from wpimath import angleModulus

from constants.field import kFarHubLocation, kCloseHubLocation
from constants.turret import kTurretLocation
from pykit.logger import Logger
from pykit.autolog import autologgable_output

from subsystems.state.robottopio import RobotTopIO
from util.convenientmath import pose3dFrom2d
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

    def __init__(self, io: RobotTopIO | None = None) -> None:
        if io is not None:
            self.io = io
        if self._initalized:
            return

        if io is None:
            # Default: created early (before factory runs); factory will swap in the real gyro
            from wrappers.wrapperedGyro import WrapperedNoGyro

            self.io = RobotTopIO(WrapperedNoGyro())

        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.inputs = RobotTopIO.RobotTopIOInputs()

        self.robotPose: Pose2d = Pose2d()
        # Deferred import to avoid circular dependency (faults.py imports RobotTopSubsystem)
        from utils.faults import Fault

        self._gyroDisconFault = Fault("Gyroscope not sending data")

        self.simResetPoseConsumers: list[Callable[[Pose2d], None]] = []
        self.simPoseReceiverConsumers: list[Callable[[], Pose2d]] = []

        self._initalized = True

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's configuration
        """

        LogTracer.resetOuter("RobotTopSubsystemPeriodic")
        self.io.updateInputs(self.inputs)
        LogTracer.record("IOUpdate")
        self._gyroDisconFault.set(not self.inputs.gyroConnected)
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
        #return self.inputs.timeUSec
        return Logger.getTimestamp()

    def getFPGATimestampS(self) -> float:
        """The time of the current robot periodic loop in seconds."""
        #return self.inputs.timeUSec / 1.0e6
        return Logger.getTimestamp() / 1.0e6

    def getRobotPose(self) -> Pose2d:
        return self.robotPose

    def setRobotPose(self, pose: Pose2d) -> None:
        self.robotPose = pose

    def resetRobotPose(self, pose: Pose2d) -> None:
        self.setRobotPose(pose)
        if RobotBase.isSimulation():
            self.resetSimPose(pose)

    def resetSimGyro(self, pose: Pose2d) -> None:
        """Reset the simulated gyro origin to the given pose. No-op on real hardware."""
        self.io.resetSimPose(pose)

    def resetSimPose(self, pose: Pose2d):
        if len(self.simResetPoseConsumers) > 0:
            for consumer in self.simResetPoseConsumers:
                consumer(pose)
            return
        print(
            f"resetSimPose: len={len(self.simResetPoseConsumers)} This is not supposed to happen"
        )

    def registerSimPoseResetConsumer(self, consumer: Callable[[Pose2d], None]) -> None:
        self.simResetPoseConsumers.append(consumer)

    def getSimPose(self) -> Pose2d:
        if len(self.simPoseReceiverConsumers) == 1:
            return self.simPoseReceiverConsumers[0]()
        print(
            f"getSimPose: len={len(self.simPoseReceiverConsumers)} This is not supposed to happen"
        )
        return self.getRobotPose()

    def getSimTurretPose(self) -> Pose3d:
        turretRotation = Rotation2d()
        return (
            pose3dFrom2d(self.getSimPose())
            + kTurretLocation
            + Transform3d(0, 0, 0, Rotation3d(0, 0, turretRotation.radians()))
        )

    def registerSimPoseReceiverConsumer(self, consumer: Callable[[], Pose2d]) -> None:
        self.simPoseReceiverConsumers.append(consumer)

    def hubLocation(self):
        return kFarHubLocation if onRed() else kCloseHubLocation

    def getDistanceToHubM(self) -> float:
        result = self.robotPose.translation().distance(self.hubLocation())
        return result

    def getRotationToHub(self) -> Rotation2d | None:
        bearingToTarget = (self.hubLocation() - self.robotPose.translation()).angle()
        rotationToTarget = (
            bearingToTarget - self.robotPose.rotation() + Rotation2d.fromDegrees(180.0)
        )
        return rotationToTarget

    def getAngleToHubRad(self) -> float:
        rotation = self.getRotationToHub()
        angleToHubRad = 0.0
        if rotation is not None:
            angleToHubRad = angleModulus(rotation.radians())
        return angleToHubRad


def RobotTopSubsystemFactory() -> RobotTopSubsystem:
    # Deferred import to avoid circular dependency (configsubsystem imports from robottopio)
    from constants import LoggerState, RobotModes
    from subsystems.state.configsubsystem import ConfigSubsystem
    from wrappers.wrapperedGyro import (
        WrapperedAdis16470ImuSingleton,
        WrapperedNavxSingleton,
        WrapperedNoGyro,
    )

    # ConfigSubsystem initialization may trigger early RobotTopSubsystem creation
    # (via WrapperedPoseEstPhotonCamera.__init__), so we replace the gyro after config loads.
    rtdc = ConfigSubsystem().robotTopDepConstants
    gyroType: str = rtdc["GYRO"]
    gyro: WrapperedNavxSingleton | WrapperedAdis16470ImuSingleton | WrapperedNoGyro
    if LoggerState().kRobotMode == RobotModes.REAL:
        if gyroType == "NAVX":
            gyro = WrapperedNavxSingleton()
        elif gyroType == "ADIS16470_IMU":
            gyro = WrapperedAdis16470ImuSingleton()
        else:
            gyro = WrapperedNoGyro()
    else:
        gyro = WrapperedNoGyro()

    io: RobotTopIO
    if LoggerState().kRobotMode == RobotModes.SIMULATION:
        from subsystems.state.robottopiosim import RobotTopIOSim

        io = RobotTopIOSim(gyro)
    else:
        io = RobotTopIO(gyro)
    robotTop = RobotTopSubsystem(io)
    return robotTop
