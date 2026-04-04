from typing import Callable, List, Optional, cast, Any
from commands2 import Subsystem
from wpimath.geometry import Transform3d

from constants import LoggerState, RobotModes
from pykit.logger import Logger
from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from subsystems.vision.vision import CameraConfiguration

from subsystems.vision.visionio import (
    VisionSubsystemIO,
    VisionSubsystemTurretedPoseObservation,
    expandTagsUint32ToList,
)

from constants.vision import (
    kApriltagFieldLayout,
    kMaxVisionZError,
    kMaxVisionAmbiguity,
    kXyStdDevCoeff,
    kThetaStdDevCoeff,
)
from constants.turret import kTurretLocation
from util.convenientmath import pose3dFromTransform3d
from util.logtracer import LogTracer
from util.robotposeestimator import TurretedVisionObservation, VisionObservation


class VisionSubsystem(Subsystem):
    def __init__(
        self,
        visionConsumers: List[Callable[[VisionObservation], None]],
        turretedVisionConsumer: Callable[[TurretedVisionObservation], None],
        io: List[VisionSubsystemIO],
    ) -> None:
        self.consumers = visionConsumers
        self.turretedConsumer = turretedVisionConsumer
        self.io = io

        self.inputs: list[VisionSubsystemIO.VisionSubsystemIOInputs] = []
        for _ in io:
            self.inputs.append(VisionSubsystemIO.VisionSubsystemIOInputs())

    # pylint:disable-next=too-many-locals, too-many-statements, too-many-branches
    def periodic(self) -> None:
        LogTracer.resetOuter("VisionSubsystem")
        for idx, (i, inp) in enumerate(zip(self.io, self.inputs)):
            i.updateInputs(inp)
            Logger.processInputs(f"Vision/Camera{idx}", self.inputs[idx])
            LogTracer.record(f"Camera{idx} UpdateInputs")
        LogTracer.record("All Cameras UpdateInputs")

        allTagPoses: List[Any] = []
        allRobotPoses = []
        allRobotPosesAccepted = []
        allRobotPosesRejected = []

        allTurretedTransforms = []
        allTurretedTransformsRejected = []
        allTurretedTransformsAccepted = []

        LogTracer.reset()
        for idx, camera in enumerate(self.inputs):
            robotPoses = []
            robotPosesAccepted = []
            robotPosesRejected = []

            turretedTransforms = []
            turretedTransformsAccepted = []
            turretedTransformsRejected = []

            # if len(camera.poseObservations) or len(camera.turretedObservations):
            #    print(f"camera observations: {camera.poseObservations} {camera.turretedObservations}")

            for observation in camera.poseObservations:
                rejectPose = False
                """
                rejectPose = (
                    observation.tagCount == 0
                    or (
                        observation.tagCount == 1
                        and observation.ambiguity > kMaxVisionAmbiguity
                    )
                    or abs(observation.pose.Z()) > kMaxVisionZError
                    or observation.pose.X() < 0.0
                    or observation.pose.X() > kApriltagFieldLayout.getFieldLength()
                    or observation.pose.Y() < 0.0
                    or observation.pose.Y() > kApriltagFieldLayout.getFieldWidth()
                )
                """

                robotPoses.append(observation.pose)
                if rejectPose:
                    robotPosesRejected.append(observation.pose)
                else:
                    robotPosesAccepted.append(observation.pose)

                if rejectPose:
                    continue

                if observation.xyStdDev_m > 0.0:
                    # Pre-computed stddevs from WrapperedPoseEstPhotonCamera — use directly
                    linearStdDev = observation.xyStdDev_m
                    angularStdDev = observation.rotStdDev_rad
                else:
                    # Fallback: compute from distance/tagCount (limelight and other IO)
                    stdDevFactor = (
                        pow(observation.avgTagDist_m, 2.0) / observation.tagCount
                    )
                    linearStdDev = kXyStdDevCoeff * stdDevFactor
                    angularStdDev = kThetaStdDevCoeff * stdDevFactor

                # here you can also factor in per-camera weighting

                observedTags = expandTagsUint32ToList(observation.tagsList)

                visionObs = VisionObservation(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    [linearStdDev, linearStdDev, angularStdDev],
                    observedTags,
                )
                for consumer in self.consumers:
                    consumer(visionObs)
            LogTracer.record(f"Camera{idx} ProcessObservations")
            for tObs in camera.turretedObservations:
                tObsTyped: VisionSubsystemTurretedPoseObservation = cast(
                    VisionSubsystemTurretedPoseObservation, tObs
                )
                rejectPose = (
                    tObsTyped.tagCount == 0
                    or (
                        tObsTyped.tagCount == 1
                        and tObsTyped.ambiguity > kMaxVisionAmbiguity
                    )
                    or abs(
                        (tObsTyped.fieldToTurret + kTurretLocation.inverse())
                        .translation()
                        .Z()
                    )
                    > kMaxVisionZError  # work backwards onto what the robot pose would be
                    or tObsTyped.fieldToTurret.X() < 0.0
                    or tObsTyped.fieldToTurret.X()
                    > kApriltagFieldLayout.getFieldLength()
                    or tObsTyped.fieldToTurret.Y() < 0.0
                    or tObsTyped.fieldToTurret.Y()
                    > kApriltagFieldLayout.getFieldWidth()
                )
                turretPose = pose3dFromTransform3d(tObsTyped.fieldToTurret)
                turretedTransforms.append(turretPose)
                if rejectPose:
                    turretedTransformsRejected.append(turretPose)
                else:
                    turretedTransformsAccepted.append(turretPose)

                if rejectPose:
                    continue

                stdDevFactor = (
                    pow(tObsTyped.averageTagDistance, 2.0) / tObsTyped.tagCount
                )
                linearStdDev = kXyStdDevCoeff * stdDevFactor
                angularStdDev = kThetaStdDevCoeff * stdDevFactor
                # here you can also factor in per-camera weighting
                observedTags = []
                tagsList = tObsTyped.tagsList
                # extract tag list from bit masks
                for tagId in range(32):
                    if tagsList & (1 << tagId):
                        observedTags.append(tagId + 1)

                self.turretedConsumer(
                    TurretedVisionObservation(
                        tObsTyped.fieldToTurret,
                        tObsTyped.timestamp,
                        [linearStdDev, linearStdDev, angularStdDev],
                        observedTags,
                    )
                )

            Logger.recordOutput(f"Vision/Camera{idx}/RobotPoses", robotPoses)
            Logger.recordOutput(
                f"Vision/Camera{idx}/RobotPosesRejected", robotPosesRejected
            )
            Logger.recordOutput(
                f"Vision/Camera{idx}/RobotPosesAccepted", robotPosesAccepted
            )
            Logger.recordOutput(
                f"Vision/Camera{idx}/TurretedTransforms", turretedTransforms
            )
            Logger.recordOutput(
                f"Vision/Camera{idx}/TurretedTransformsRejected",
                turretedTransformsRejected,
            )
            Logger.recordOutput(
                f"Vision/Camera{idx}/TurretedTransformsAccepted",
                turretedTransformsAccepted,
            )
            allRobotPoses.extend(robotPoses)
            allRobotPosesAccepted.extend(robotPosesAccepted)
            allRobotPosesRejected.extend(robotPosesRejected)
            allTurretedTransforms.extend(turretedTransforms)
            allTurretedTransformsAccepted.extend(turretedTransformsAccepted)
            allTurretedTransformsRejected.extend(turretedTransformsRejected)
        LogTracer.record("All Cameras ProcessObservations")

        Logger.recordOutput("Vision/Summary/TagPose", allTagPoses)
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses
        )  # Problems: In simulation replay this field differs
        # The convert to csv tool and find differences tool does not seem to handle this list of poses.
        # Perhaps the check for errors part of the test does not look at lists of poses.
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected)
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted)
        Logger.recordOutput("Vision/Summary/TurretedTransforms", allTurretedTransforms)
        Logger.recordOutput(
            "Vision/Summary/TurretedTransformsRejected",
            allTurretedTransformsRejected,
        )
        Logger.recordOutput(
            "Vision/Summary/TurretedTransformsAccepted",
            allTurretedTransformsAccepted,
        )
        LogTracer.recordTotal()


def VisionSubsystemFactory(
    visionConsumers: List[Callable[[VisionObservation], None]] | None = None,
) -> VisionSubsystem | None:
    VDC = ConfigSubsystem().visionDepConstants
    vision: Optional[VisionSubsystem] = None
    if VDC["HAS_VISION"]:
        io: List[VisionSubsystemIO] = []

        for cam in VDC["CAMS"]:
            config: CameraConfiguration = VDC[cam]
            t: Transform3d = config.robotToCameraTransform
            print(
                f"{cam} {t.X()} {t.Y()} {t.Z()} {t.rotation().X()} {t.rotation().Y()} {t.rotation().Z()}"
            )

            match LoggerState().kRobotMode:
                case RobotModes.REAL:
                    io.append(
                        config.realCameraIO(
                            config.cameraName, config.robotToCameraTransform
                        )
                    )
                case RobotModes.SIMULATION:
                    io.append(
                        config.simCameraIO(  # type: ignore[misc]
                            config.cameraName,
                            config.robotToCameraTransform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotTopSubsystem().getSimPose(),
                        )
                    )
                case _:
                    io.append(VisionSubsystemIO())

        vision = VisionSubsystem(
            visionConsumers if visionConsumers is not None else [],
            lambda _: None,
            io=io,
        )

    return vision
