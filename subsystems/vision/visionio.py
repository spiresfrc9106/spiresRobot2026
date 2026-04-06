from dataclasses import dataclass, field
from typing import List

from constants.vision import ObservationType
from pykit.autolog import autolog
from wpimath.geometry import Pose3d, Transform3d
from wpiutil.wpistruct import uint64, make_wpistruct


tagsIntType = uint64


@make_wpistruct(name="visionobservation")
@autolog
@dataclass
class VisionSubsystemPoseObservation:
    timestamp: float = 0.0
    pose: Pose3d = field(default_factory=Pose3d)
    ambiguity: float = 0.0
    tagCount: int = 0
    tagsList: tagsIntType = tagsIntType(0)
    avgTagDist_m: float = 0.0
    observationType: int = ObservationType.PHOTONVISION.value
    xyStdDev_m: float = 0.0
    rotStdDev_rad: float = 0.0


def condenseTagsListToUint32(tagsAsList: List[int]) -> tagsIntType:
    idsCondensed = 0
    # this is stored in a 32 bit unsigned integer, this is OK since only 32 tags exist on the field
    for tagId in tagsAsList:
        idsCondensed |= (
            1 << tagId - 1
        )  # condense the tag IDs into a single integer using bitwise OR. This allows us to store up to 32 tag IDs in a single integer, which is more efficient than storing a list of integers.
    return tagsIntType(idsCondensed)


def expandTagsUint32ToList(tagsAsUint32: tagsIntType) -> List[int]:
    observedTags = []
    for tagId in range(32):
        if tagsAsUint32 & (1 << tagId):
            observedTags.append(
                tagId + 1
            )  # need to add 1 since tag IDs are 1 indexed but our bitmask is 0 indexed
    return observedTags


@make_wpistruct(name="turretedobservation")
@autolog
@dataclass
class VisionSubsystemTurretedPoseObservation:
    timestamp: float = 0
    fieldToTurret: Transform3d = field(default_factory=Transform3d)
    ambiguity: float = 0
    tagCount: int = 0
    tagsList: tagsIntType = tagsIntType(0)
    averageTagDistance: float = 0
    observationType: int = ObservationType.PHOTONVISION.value


class VisionSubsystemIO:
    @autolog
    @dataclass
    class VisionSubsystemIOInputs:
        connected: bool = False
        poseObservations: list[VisionSubsystemPoseObservation] = field(
            default_factory=list
        )
        turretedObservations: list[VisionSubsystemTurretedPoseObservation] = field(
            default_factory=list
        )

    def updateInputs(self, inputs: VisionSubsystemIOInputs):
        pass

    def updateCameraPosition(self, transform: Transform3d) -> None:
        raise NotImplementedError("Must be implemented by subclass")
