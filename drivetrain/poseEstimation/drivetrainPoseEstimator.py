from typing import Tuple
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d

from drivetrain.drivetrainPhysical import DrivetrainPhysical
from drivetrain.poseEstimation.drivetrainPoseTelemetry import DrivetrainPoseTelemetry
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from util.robotposeestimator import VisionObservation

# TODO-rms was:from navigation.autoDriveNavConstants import SCORE_DIST_FROM_REEF_CENTER
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
# from utils.constants import blueReefLocation, redReefLocation #2025 code

# Convienent abreviations for the types that we'll be passing around here.
# This is primarily driven by wpilib's conventions:
# 1) Swerve objects will be put into a tuple, with the length of the tuple equal to the number of modules
# 2) "State" refers to the speed of the wheel, plus the position of the azimuth
# 3) "Position" refers to the position of the wheel, plus the position of the azimuth
# Wheel position is preferred for odometry, since errors in velocity don't accumulate over time
# This is especially important with Rev motors which filter their velocity by a huge amount
# but the position is fairly accurate.
PosTupleType = Tuple[
    SwerveModulePosition,
    SwerveModulePosition,
    SwerveModulePosition,
    SwerveModulePosition,
]
StateTupleType = Tuple[
    SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
]


class DrivetrainPoseEstimator:
    """Wrapper class for all sensors and logic responsible for estimating where the robot is on the field"""

    def __init__(
        self,
        initialModulePositions: PosTupleType,
    ):

        self.robotTopSubsystem = RobotTopSubsystem()
        # Represents our current best-guess as to our location on the field.
        self._curEstPose = Pose2d()

        # The kalman filter to fuse all sources of information together into a single
        # best-estimate of the pose of the field
        self.kinematics = DrivetrainPhysical().kinematics
        self._poseEst = SwerveDrive4PoseEstimator(
            self.kinematics,
            Rotation2d(),
            initialModulePositions,
            self._curEstPose,
        )

        self._lastModulePositions = initialModulePositions

        # Logging and Telemetry
        self._telemetry = DrivetrainPoseTelemetry()
        self.limelightPoseOfTarget = None

        self.lastCamEstRobotPos = Pose2d()

    def setKnownPose(self, knownPose: Pose2d):
        """Reset the robot's estimated pose to some specific position. This is useful if we know with certanty
        we are at some specific spot (Ex: start of autonomous)

        Args:
            knownPose (Pose2d): The pose we know we're at
        """
        # Reset the simulated gyro origin (no-op on real hardware / replay).
        self.robotTopSubsystem.resetSimGyro(knownPose)

        self._poseEst.resetPosition(
            self.robotTopSubsystem.inputs.gyroAngleRotation,
            self._lastModulePositions,
            knownPose,
        )
        self.robotTopSubsystem.resetRobotPose(self._poseEst.getEstimatedPosition())

    def update(self, curModulePositions: PosTupleType, curModuleSpeeds: StateTupleType):
        """Periodic update, call this every 20ms.

        Args:
            curModulePositions  current module angle
            and wheel positions as read in from swerve module sensors
        """

        # Update the WPILib Pose Estimate
        self._poseEst.updateWithTime(
            self.robotTopSubsystem.getFPGATimestampS(),
            self.robotTopSubsystem.inputs.gyroAngleRotation,
            curModulePositions,
        )
        self._curEstPose = self._poseEst.getEstimatedPosition()
        self.robotTopSubsystem.setRobotPose(self._curEstPose)

        # Record the estimate to telemetry/logging-
        self._telemetry.update(self._curEstPose, [x.angle for x in curModulePositions])

        # Remember the module positions for next loop
        self._lastModulePositions = curModulePositions

    def getCurEstPose(self) -> Pose2d:
        """
        Returns:
            Pose2d: The most recent estimate of where the robot is at
        """
        return self._curEstPose

    def addVisionObservation(self, obs: VisionObservation) -> None:
        """Accept a vision observation from VisionSubsystem and feed it to the pose estimator."""
        self._poseEst.addVisionMeasurement(
            obs.visionPose,
            obs.timestamp,
            obs.std,
        )
        self._telemetry.addVisionObservation(obs)
