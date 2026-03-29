from collections.abc import Callable
from typing import Optional
from pykit.logger import Logger
from wpimath.geometry import Pose2d, Pose3d, Transform3d
from photonlibpy.simulation import SimCameraProperties, PhotonCameraSim, VisionSystemSim

from subsystems.state.robottopsubsystem import RobotTopSubsystem
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniophoton import VisionSubsystemIOPhotonVision

from constants.vision import kApriltagFieldLayout
from util.convenientmath import pose3dFrom2d


class VisionSubsystemIOPhotonSim(VisionSubsystemIOPhotonVision):
    visionSim: Optional[VisionSystemSim] = None
    turretSim: Optional[VisionSystemSim] = None

    def __init__(
        self,
        name: str,
        robotToCamera: Transform3d,
        poseSupplier: Callable[[], Pose2d | Pose3d],
        isTurreted: bool = False,
    ) -> None:
        VisionSubsystemIOPhotonVision.__init__(self, name, robotToCamera, isTurreted)
        self.poseSupplier = poseSupplier

        if VisionSubsystemIOPhotonSim.visionSim is None:
            VisionSubsystemIOPhotonSim.visionSim = VisionSystemSim("main")
            VisionSubsystemIOPhotonSim.visionSim.addAprilTags(kApriltagFieldLayout)

        # The turret sim is separate from the main sim to account for the relative "base" transform changing
        # when the turret rotates. This is also offset from the main robot.
        if VisionSubsystemIOPhotonSim.turretSim is None:
            VisionSubsystemIOPhotonSim.turretSim = VisionSystemSim("turret")
            VisionSubsystemIOPhotonSim.turretSim.addAprilTags(kApriltagFieldLayout)

        cameraProperties = SimCameraProperties.OV9281_1920_1080()
        self.cameraSim = PhotonCameraSim(
            self._wrapperedCam.cam, cameraProperties, kApriltagFieldLayout
        )

        if isTurreted:
            VisionSubsystemIOPhotonSim.turretSim.addCamera(
                self.cameraSim, robotToCamera
            )
        else:
            VisionSubsystemIOPhotonSim.visionSim.addCamera(
                self.cameraSim, robotToCamera
            )

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):

        if self.isTurreted:
            assert VisionSubsystemIOPhotonSim.turretSim is not None
            VisionSubsystemIOPhotonSim.turretSim.update(self.poseSupplier())
            turretPose = RobotTopSubsystem().getSimTurretPose()
            camPose = turretPose + self._wrapperedCam.robotToCam
        else:
            assert VisionSubsystemIOPhotonSim.visionSim is not None
            VisionSubsystemIOPhotonSim.visionSim.update(self.poseSupplier())
            robotPose = pose3dFrom2d(RobotTopSubsystem().getSimPose())
            camPose = robotPose + self._wrapperedCam.robotToCam
        super().updateInputs(inputs)

        Logger.recordOutput(
            "VisionSim/Cam" + self._wrapperedCam.cam.getName() + "Pose", camPose
        )
