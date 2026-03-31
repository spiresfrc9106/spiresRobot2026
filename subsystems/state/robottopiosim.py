import random
from collections.abc import Callable

from wpimath.geometry import Pose2d, Rotation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds

from constants import kRobotUpdatePeriodS
from subsystems.state.robottopio import RobotTopIO


class RobotTopIOSim(RobotTopIO):
    """Simulation variant of RobotTopIO.

    Integrates chassis speeds each cycle to produce a simulated gyro angle that is
    written into inputs.gyroAngleRad so it flows through the IO logging layer and is
    replayable.
    """

    def __init__(self, gyro) -> None:
        super().__init__(gyro)
        self._simPose: Pose2d = Pose2d()
        self._chassisSpeedSupplier: Callable[[], ChassisSpeeds] | None = None

    def setChassisSpeedSupplier(self, supplier: Callable[[], ChassisSpeeds]) -> None:
        """Wire in the drivetrain chassis speed supplier after drivetrain is created."""
        self._chassisSpeedSupplier = supplier

    def resetSimPose(self, pose: Pose2d) -> None:
        """Reset the simulated robot pose (called when a known pose is set)."""
        self._simPose = pose

    def updateInputs(self, inputs: RobotTopIO.RobotTopIOInputs) -> None:
        super().updateInputs(inputs)  # populates timeUSec

        spds = (
            self._chassisSpeedSupplier()
            if self._chassisSpeedSupplier is not None
            else ChassisSpeeds()
        )

        self._simPose = self._simPose.exp(
            Twist2d(
                spds.vx * kRobotUpdatePeriodS,
                spds.vy * kRobotUpdatePeriodS,
                spds.omega * kRobotUpdatePeriodS,
            )
        )

        noise = Rotation2d.fromDegrees(random.uniform(-0.0, 0.0))
        inputs.gyroAngleRad = (self._simPose.rotation() + noise).radians()
        inputs.gyroConnected = True
        inputs.gyroYawRateRadPerSec = spds.omega
