from dataclasses import dataclass
from typing import Callable

from wpimath.geometry import Transform3d, Pose3d, Rotation3d
from wpimath.units import inchesToMeters

from constants import LoggerState, RobotModes
from constants import RobotTypes
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniophoton import VisionSubsystemIOPhotonVision

if (
    LoggerState().kRobotMode == RobotModes.SIMULATION
):  # required since opencv can't go on rio
    # pylint:disable-next=ungrouped-imports
    from subsystems.vision.visioniophotonsim import VisionSubsystemIOPhotonSim
from utils.singleton import Singleton


@dataclass
class CameraConfiguration:
    cameraName: str
    realCameraIO: Callable[[str, Transform3d], VisionSubsystemIO]
    simCameraIO: Callable[..., VisionSubsystemIO] | None
    robotToCameraTransform: Transform3d


kRobotToBackCenterCamTransform = Transform3d(
    Pose3d(),
    Pose3d(
        inchesToMeters(-12.75),
        inchesToMeters(-2.25),
        inchesToMeters(8.0 + 3.0 / 16.0),
        Rotation3d.fromDegrees(0, -10.0, 180.0),
    ),
)

kRobotToBackHighCamTransform = Transform3d(
    Pose3d(),
    Pose3d(
        inchesToMeters(-(12.75 - 1.0)),
        inchesToMeters(+2.25),
        inchesToMeters(8.0 + 11.0 / 16.0),
        Rotation3d.fromDegrees(0, -40.0, 180.0),
    ),
)

kRobotToFrontLeftCamTransform = Transform3d(
    Pose3d(),
    Pose3d(
        inchesToMeters(+9.5),
        inchesToMeters(+11.5),
        inchesToMeters(+10.0),
        Rotation3d.fromDegrees(0, -20.0, -20.0),
    ),
)

kRobotToFrontRightCamTransform = Transform3d(
    Pose3d(),
    Pose3d(
        inchesToMeters(+9.5),
        inchesToMeters(-11.5),
        inchesToMeters(+10.0),
        Rotation3d.fromDegrees(0, -20.0, +20.0),
    ),
)


kBackCenterCamConfiguration = CameraConfiguration(
    cameraName="back_center_cam",
    realCameraIO=VisionSubsystemIOPhotonVision,
    simCameraIO=VisionSubsystemIOPhotonSim
    if LoggerState().kRobotMode == RobotModes.SIMULATION
    else None,
    robotToCameraTransform=kRobotToBackCenterCamTransform,
)

kBackHighCamConfiguration = CameraConfiguration(
    cameraName="back_high_cam",
    realCameraIO=VisionSubsystemIOPhotonVision,
    simCameraIO=VisionSubsystemIOPhotonSim
    if LoggerState().kRobotMode == RobotModes.SIMULATION
    else None,
    robotToCameraTransform=kRobotToBackHighCamTransform,
)

kFrontLeftCamConfiguration = CameraConfiguration(
    cameraName="front_left_cam",
    realCameraIO=VisionSubsystemIOPhotonVision,
    simCameraIO=VisionSubsystemIOPhotonSim
    if LoggerState().kRobotMode == RobotModes.SIMULATION
    else None,
    robotToCameraTransform=kRobotToFrontLeftCamTransform,
)

kFrontRightCamConfiguration = CameraConfiguration(
    cameraName="front_right_cam",
    realCameraIO=VisionSubsystemIOPhotonVision,
    simCameraIO=VisionSubsystemIOPhotonSim
    if LoggerState().kRobotMode == RobotModes.SIMULATION
    else None,
    robotToCameraTransform=kRobotToFrontRightCamTransform,
)


class VisionDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.visionDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_VISION": False,
                "CAMS": [],
                "back_center_cam": None,
            },
            RobotTypes.Spires2026: {
                "HAS_VISION": True,
                "CAMS": [
                    "back_center_cam",
                    "back_high_cam",
                    "front_left_cam",
                    "front_right_cam",
                ],
                "back_center_cam": kBackCenterCamConfiguration,
                "back_high_cam": kBackHighCamConfiguration,
                "front_left_cam": kFrontLeftCamConfiguration,
                "front_right_cam": kFrontRightCamConfiguration,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_VISION": True,
                "CAMS": [
                    "back_center_cam",
                    "back_high_cam",
                    "front_left_cam",
                    "front_right_cam",
                ],
                "back_center_cam": kBackCenterCamConfiguration,
                "back_high_cam": kBackHighCamConfiguration,
                "front_left_cam": kFrontLeftCamConfiguration,
                "front_right_cam": kFrontRightCamConfiguration,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_VISION": False,
                "CAMS": [],
                "back_center_cam": None,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_VISION": False,
                "CAMS": [],
                "back_center_cam": None,
            },
        }

    def get(self, robotType: RobotTypes):
        return self.visionDepConstants[robotType]
