from wpimath.geometry import Pose2d, Transform3d

from subsystems.vision.visionio import (
    ObservationType,
    VisionSubsystemIO,
    VisionSubsystemPoseObservation,
)
from util.convenientmath import pose3dFrom2d
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera


class VisionSubsystemIOPhotonVision(VisionSubsystemIO):
    def __init__(
        self, name: str, robotToCamera: Transform3d, isTurreted: bool = False
    ) -> None:
        VisionSubsystemIO.__init__(self)
        self._wrapperedCam = WrapperedPoseEstPhotonCamera(name, robotToCamera)
        self.isTurreted = isTurreted

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self._wrapperedCam.robotToCam = transform

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs) -> None:
        self._wrapperedCam.update(Pose2d())
        inputs.connected = self._wrapperedCam.cam.isConnected()

        poseObservations = []
        for camObs in self._wrapperedCam.getPoseEstimates():
            poseObservations.append(
                VisionSubsystemPoseObservation(
                    timestamp=camObs.time,
                    pose=pose3dFrom2d(camObs.estFieldPose),
                    ambiguity=0.0,
                    tagCount=2,
                    tagsList=0,
                    averageTagDistance=0.0,
                    observationType=ObservationType.PHOTONVISION.value,
                    xyStdDev=camObs.xyStdDev,
                    rotStdDev=camObs.rotStdDev,
                )
            )
        inputs.poseObservations = poseObservations
        inputs.tagIds = []
        inputs.turretedObservations = []
