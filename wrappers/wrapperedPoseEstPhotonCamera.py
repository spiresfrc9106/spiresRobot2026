from dataclasses import dataclass
from typing import List

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d, Transform3d, Pose3d

from constants import kRobotUpdatePeriodS
from pykit.logger import Logger
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonCamera import setVersionCheckEnabled
from photonlibpy import PhotonPoseEstimator, PhotonCamera, EstimatedRobotPose

from utils.faults import Fault

MAX_CAMERA_TARGETS = 4
MAX_CAMERA_SOLUTIONS = 4


# Describes one on-field pose estimate from the acamera at a specific time.
@dataclass
class CameraPoseObservation:
    time: float
    estFieldPose: Pose2d
    xyStdDev: float = 1.0  # std dev of error in measurment, units of meters.
    rotStdDev: float = degreesToRadians(
        99999.0
    )  # std dev of measurement, in units of radians


EMPTY_TRANSFORM = Transform3d()


# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPoseEstPhotonCamera:
    def __init__(self, camName: str, robotToCam: Transform3d):
        setVersionCheckEnabled(False)

        self.name = camName
        self.cam = PhotonCamera(camName)

        self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates: list[CameraPoseObservation] = []
        self.robotToCam = robotToCam
        self.lastLatency = 0.0
        self.updateDuration = 0.0
        self.prevTimestampSec = 0.0
        self.singleTagModeTagList: list[int] | None = None  # not currently used

        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            self.robotToCam,
        )

        self.lastCaptureTime = RobotTopSubsystem().getFPGATimestampS()
        self.CAP_PERIOD_SEC = 0.025

    def setSingleTagMode(self, tag: list[int] | None):
        self.singleTagModeTagList = tag

    def update(self, prevEstPose: Pose2d):

        startTime = RobotTopSubsystem().getFPGATimestampS()

        self.poseEstimates = []
        self.lastLatency = 0.0

        if not self.cam.isConnected():
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return

        result = self.cam.getLatestResult()
        i = 0
        if (
            result.hasTargets()
            and RobotTopSubsystem().getFPGATimestampS() - result.getTimestampSeconds()
            < 2.0 * kRobotUpdatePeriodS
        ):
            targets: List[PhotonTrackedTarget] = result.getTargets()

            for target in targets[:MAX_CAMERA_TARGETS]:
                id = target.getFiducialId()
                poseAmbiguity = target.getPoseAmbiguity()
                bestCameraToTarget = target.getBestCameraToTarget()
                alternateCameraToTarget = target.getAlternateCameraToTarget()
                bestCameraFieldPose = Pose3d(
                    bestCameraToTarget.translation(), bestCameraToTarget.rotation()
                )
                alternateCameraFieldPose = Pose3d(
                    alternateCameraToTarget.translation(),
                    alternateCameraToTarget.rotation(),
                )
                bestRobotPredictedPose = bestCameraFieldPose.transformBy(
                    self.robotToCam.inverse()
                )
                alternateRobotPredictedPose = alternateCameraFieldPose.transformBy(
                    self.robotToCam.inverse()
                )

                # cameraFieldPose.transformBy(robotToCamera.inverse())
                yaw = target.getYaw()
                pitch = target.getPitch()
                area = target.getArea()
                Logger.recordOutput(f"{self.name}/target/{i}/id", id)
                Logger.recordOutput(
                    f"{self.name}/target/{i}/poseAmbiguity", poseAmbiguity
                )
                Logger.recordOutput(
                    f"{self.name}/target/{i}/bestCameraToTarget", bestCameraToTarget
                )
                Logger.recordOutput(
                    f"{self.name}/target/{i}/bestRobotField", bestRobotPredictedPose
                )
                Logger.recordOutput(
                    f"{self.name}/target/{i}/alternateCameraToTarget",
                    alternateCameraToTarget,
                )
                Logger.recordOutput(
                    f"{self.name}/target/{i}/alternateRobotField",
                    alternateRobotPredictedPose,
                )
                Logger.recordOutput(f"{self.name}/target/{i}/yaw", yaw)
                Logger.recordOutput(f"{self.name}/target/{i}/pitch", pitch)
                Logger.recordOutput(f"{self.name}/target/{i}/area", area)
                i += 1

            camEstPose: EstimatedRobotPose | None = (
                self.camPoseEst.estimateCoprocMultiTagPose(result)
            )
            if camEstPose is None:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)

            if camEstPose is not None:
                self.poseEstimates.append(
                    CameraPoseObservation(
                        time=camEstPose.timestampSeconds,
                        estFieldPose=camEstPose.estimatedPose.toPose2d(),
                        xyStdDev=3.0,
                        rotStdDev=degreesToRadians(60.0),
                    )
                )

        targets_found = i
        for i in range(MAX_CAMERA_TARGETS - targets_found):
            Logger.recordOutput(f"{self.name}/target/{targets_found + i}/id", 0)
            Logger.recordOutput(
                f"{self.name}/target/{targets_found + i}/poseAmbiguity", 0.0
            )
            Logger.recordOutput(
                f"{self.name}/target/{targets_found + i}/bestCameraToTarget",
                EMPTY_TRANSFORM,
            )
            Logger.recordOutput(
                f"{self.name}/target/{targets_found + i}/bestRobotField", Pose3d()
            )
            Logger.recordOutput(
                f"{self.name}/target/{targets_found + i}/alternateCameraToTarget",
                EMPTY_TRANSFORM,
            )
            Logger.recordOutput(
                f"{self.name}/target/{targets_found + i}/alternateRobotField", Pose3d()
            )
            Logger.recordOutput(f"{self.name}/target/{targets_found + i}/yaw", 0.0)
            Logger.recordOutput(f"{self.name}/target/{targets_found + i}/pitch", 0.0)
            Logger.recordOutput(f"{self.name}/target/{targets_found + i}/area", 0.0)

        posesFound = 0
        for pose in self.poseEstimates[:MAX_CAMERA_SOLUTIONS]:
            Logger.recordOutput(f"{self.name}/sol/{posesFound}/pose", pose.estFieldPose)
            posesFound += 1
        for i in range(MAX_CAMERA_SOLUTIONS - posesFound):
            Logger.recordOutput(f"{self.name}/sol/{posesFound + i}/pose", Pose2d())

        endTime = RobotTopSubsystem().getFPGATimestampS()
        self.updateDuration = (endTime - startTime) * 1000.0

    def getPoseEstimates(self):
        return self.poseEstimates

    def _toFieldPose(self, tgtPose, camToTarget):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = 0.0 <= y <= feetToMeters(27.0)
        inX = 0.0 <= x <= feetToMeters(54.0)
        return inX and inY

    def _closeEnoughToCamera(self, target: PhotonTrackedTarget):
        return target.getBestCameraToTarget().translation().norm() <= 2.0
