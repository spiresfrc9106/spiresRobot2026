from wpimath.geometry import Pose2d
from dataclasses import dataclass, field

from wpimath.kinematics import ChassisSpeeds


@dataclass
class DrivetrainCommand:
    """
    Represents desired drivetrain motion which is currently desired.
    Usually comes from a human driver, but could be from an autonomous momde or assist feature.
    """

    velX: float = 0.0  # Field X velocity in meters/sec
    velY: float = 0.0  # Field Y velocity in meters/sec
    velT: float = 0.0  # Rotational speed in rad/sec
    robotRelative: bool = False
    desPose: Pose2d = field(
        default_factory=lambda: Pose2d()
    )  # Current desired pose of the drivetrain

    # as a hack in the 2026-03-14 competition to add auto's by embedding the desired chassis speeds
    # in the DriveTrain command. Use if not none. If none, use velX, velY, velT, and robotRelative
    desChassisSpeeds: ChassisSpeeds = None

    def scaleBy(self, scale: float):
        """
        Scale all motion xyzzy by a single factor
        """
        self.velX *= scale
        self.velY *= scale
        self.velT *= scale
