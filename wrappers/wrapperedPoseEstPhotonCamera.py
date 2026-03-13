from dataclasses import dataclass
import wpilib
from photonlibpy.targeting import PhotonPipelineResult
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d

from constants.vision import kMaxVisionAmbiguity
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from wrappers.casserolePhotonCamera import PhotonCamera
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonCamera import setVersionCheckEnabled
from photonlibpy import PhotonPoseEstimator, PhotonCamera, EstimatedRobotPose

from utils.fieldTagLayout import FieldTagLayout
from utils.faults import Fault

# Describes one on-field pose estimate from the acamera at a specific time.
@dataclass
class CameraPoseObservation:
    time:float
    estFieldPose:Pose2d
    xyStdDev:float=1.0 # std dev of error in measurment, units of meters.
    rotStdDev:float=degreesToRadians(99999.0) # std dev of measurement, in units of radians

# Sort Tags by location
REEF_TAG_IDS = [6,7,8,9,10,11,17,18,19,20,21,22]
BARGE_TAG_IDS = [4,5,14,15]
PROCESSOR_TAG_IDS = [3,16]
HUMAN_STATION_TAG_IDS = [1,2,12,13]

# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPoseEstPhotonCamera:
    def __init__(self, camName, robotToCam):
        setVersionCheckEnabled(False)

        self.name = camName
        self.cam = PhotonCamera(camName)

        self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates:list[CameraPoseObservation] = []
        self.robotToCam = robotToCam
        self.lastLatency = 0.0
        self.updateDuration = 0.0
        self.prevTimestampSec = 0.0
        self.singleTagModeTagList = None #not currently used

        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            self.robotToCam,
        )


        self.lastCaptureTime = RobotTopSubsystem().getFPGATimestampS()
        self.CAP_PERIOD_SEC = 0.025

    def setSingleTagMode(self, tag:list[int]|None):
        self.singleTagModeTagList = tag

    def update(self, prevEstPose:Pose2d):

        startTime = RobotTopSubsystem().getFPGATimestampS()

        self.poseEstimates = []
        self.lastLatency = 0.0

        if(not self.cam.isConnected()):
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return

        for result in self.cam.getAllUnreadResults():
            if RobotTopSubsystem().getFPGATimestampS()-result.getTimestampSeconds()<0.1:
                camEstPose: EstimatedRobotPose  = self.camPoseEst.estimateCoprocMultiTagPose(result)
                if camEstPose is None:
                    camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(result)

                if camEstPose is not None:
                    self.poseEstimates.append(
                        CameraPoseObservation(time=camEstPose.timestampSeconds,
                                              estFieldPose=camEstPose.estimatedPose.toPose2d(),
                                              xyStdDev=1.0,
                                              rotStdDev=degreesToRadians(60.0))
                    )

        endTime = RobotTopSubsystem().getFPGATimestampS()
        self.updateDuration = (endTime - startTime)*1000.0

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
    
