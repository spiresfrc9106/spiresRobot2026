from wpimath.geometry import Pose2d, Transform3d

from subsystems.vision.visionio import (
    VisionSubsystemIO,
)
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
        inputs.poseObservations = self._wrapperedCam.getPoseEstimates()
        inputs.tagIds = []
        inputs.turretedObservations = []
