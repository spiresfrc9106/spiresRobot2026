from dataclasses import dataclass
from typing import List

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d, Transform3d, Pose3d

from constants import kRobotUpdatePeriodS
from constants.vision import ObservationType
from pykit.logger import Logger
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonCamera import setVersionCheckEnabled
from photonlibpy import PhotonPoseEstimator, PhotonCamera, EstimatedRobotPose

from subsystems.vision.visionio import (
    VisionSubsystemPoseObservation,
    condenseTagsListToUint32,
)
from utils.faults import Fault

MAX_CAMERA_TARGETS = 4
MAX_CAMERA_SOLUTIONS = 4

# Std dev scaling: xyStdDev = K_XY * avg_dist^2 / n_tags (multi-tag)
#                  xyStdDev = K_XY_SINGLE * dist^2       (single-tag)
# Rotation: multi-tag uses K_ROT * avg_dist^2 / n_tags; single-tag trusts gyro (very large).
_K_XY_MULTI = 0.01
_K_ROT_MULTI = 0.05
_K_XY_SINGLE = 0.1
_MAX_SINGLE_TAG_AMBIGUITY = 0.2
_ROT_STD_DEV_SINGLE_TAG = degreesToRadians(60.0)


# Describes one on-field pose estimate from the camera at a specific time.
@dataclass
class CameraPoseObservation:
    time: float
    estFieldPose: Pose2d
    xyStdDev: float = 1.0  # std dev of error in measurment, units of meters.
    rotStdDev: float = degreesToRadians(
        99999.0
    )  # std dev of measurement, in units of radians
    observationType: ObservationType = ObservationType.UNKNOWN
    averageDistance_m: float = 0.0
    nTags: int = 0


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
        self.poseEstimates: list[VisionSubsystemPoseObservation] = []
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
        logTargets = False
        if (
            result.hasTargets()
            and RobotTopSubsystem().getFPGATimestampS() - result.getTimestampSeconds()
            < 2.0 * kRobotUpdatePeriodS
        ):
            if logTargets:
                targets: List[PhotonTrackedTarget] = result.getTargets()
                for target in targets[:MAX_CAMERA_TARGETS]:
                    id = target.getFiducialId()
                    poseAmbiguity = target.getPoseAmbiguity()
                    bestCameraToTarget = target.getBestCameraToTarget()
                    alternateCameraToTarget = target.getAlternateCameraToTarget()
                    tagFieldPose = self.camPoseEst.fieldTags.getTagPose(id)
                    if tagFieldPose is not None:
                        bestCameraFieldPose = tagFieldPose.transformBy(
                            bestCameraToTarget.inverse()
                        )
                        alternateCameraFieldPose = tagFieldPose.transformBy(
                            alternateCameraToTarget.inverse()
                        )
                    else:
                        bestCameraFieldPose = Pose3d()
                        alternateCameraFieldPose = Pose3d()
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
            if camEstPose is not None:
                usedTargets = camEstPose.targetsUsed
                nTags = len(usedTargets)
                sumDist_m = sum(
                    t.getBestCameraToTarget().translation().norm() for t in usedTargets
                )
                avgDist_m = sumDist_m / nTags

                xyStdDev_m = _K_XY_MULTI * avgDist_m**2 / nTags
                rotStdDev_rad = _K_ROT_MULTI * avgDist_m**2 / nTags
                assert result.multitagResult is not None
                self.poseEstimates.append(
                    VisionSubsystemPoseObservation(
                        timestamp=camEstPose.timestampSeconds,
                        pose=camEstPose.estimatedPose,
                        ambiguity=0.0,
                        tagCount=nTags,
                        tagsList=condenseTagsListToUint32(
                            result.multitagResult.fiducialIDsUsed
                        ),
                        avgTagDist_m=avgDist_m,
                        observationType=ObservationType.PHOTON_MULTITAG.value,
                        xyStdDev_m=xyStdDev_m,
                        rotStdDev_rad=rotStdDev_rad,
                    )
                )
            else:
                camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)
                if camEstPose is not None and abs(camEstPose.estimatedPose.z) < 0.5:
                    singleTarget = camEstPose.targetsUsed[0]
                    poseAmbiguity = singleTarget.getPoseAmbiguity()
                    if True:  # if poseAmbiguity <= _MAX_SINGLE_TAG_AMBIGUITY:
                        """
                        targetPosition = self.camPoseEst._fieldTags.getTagPose(singleTarget.getFiducialId())
                        altCamEstPose = EstimatedRobotPose(
                            targetPosition.transformBy(
                            singleTarget.getBestCameraToTarget().inverse()
                            ).transformBy(self.robotToCam.inverse()),
                            result.getTimestampSeconds(),
                            result.targets,
                        )
                        if abs(camEstPose.estimatedPose.z) > 0.5 and abs(altCamEstPose.estimatedPose.z) < abs(camEstPose.estimatedPose.z):
                            camEstPose = altCamEstPose
                        """

                        avgDist_m = (
                            singleTarget.getBestCameraToTarget().translation().norm()
                        )
                        xyStdDev_m = _K_XY_SINGLE * avgDist_m**2
                        self.poseEstimates.append(
                            VisionSubsystemPoseObservation(
                                timestamp=camEstPose.timestampSeconds,
                                pose=camEstPose.estimatedPose,
                                ambiguity=poseAmbiguity,
                                tagCount=1,
                                tagsList=condenseTagsListToUint32(
                                    [singleTarget.getFiducialId()]
                                ),
                                avgTagDist_m=avgDist_m,
                                observationType=ObservationType.PHOTON_SINGLETAG.value,
                                xyStdDev_m=xyStdDev_m,
                                rotStdDev_rad=_ROT_STD_DEV_SINGLE_TAG,
                            )
                        )

        if logTargets:
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
                    f"{self.name}/target/{targets_found + i}/alternateRobotField",
                    Pose3d(),
                )
                Logger.recordOutput(f"{self.name}/target/{targets_found + i}/yaw", 0.0)
                Logger.recordOutput(
                    f"{self.name}/target/{targets_found + i}/pitch", 0.0
                )
                Logger.recordOutput(f"{self.name}/target/{targets_found + i}/area", 0.0)

        logSolutions = False
        if logSolutions:
            posesFound = 0
            for vo in self.poseEstimates[:MAX_CAMERA_SOLUTIONS]:
                Logger.recordOutput(f"{self.name}/sol/{posesFound}/pose", vo.pose)
                Logger.recordOutput(
                    f"{self.name}/sol/{posesFound}/xyStdDev", vo.xyStdDev_m
                )
                Logger.recordOutput(
                    f"{self.name}/sol/{posesFound}/rotStdDev_rad", vo.rotStdDev_rad
                )
                posesFound += 1
            for i in range(MAX_CAMERA_SOLUTIONS - posesFound):
                Logger.recordOutput(f"{self.name}/sol/{posesFound + i}/pose", Pose3d())
                Logger.recordOutput(f"{self.name}/sol/{posesFound}/xyStdDev", 0.0)
                Logger.recordOutput(f"{self.name}/sol/{posesFound}/rotStdDev_rad", 0.0)

        endTime = RobotTopSubsystem().getFPGATimestampS()
        self.updateDuration = (endTime - startTime) * 1000.0

    def getPoseEstimates(self) -> List[VisionSubsystemPoseObservation]:
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
