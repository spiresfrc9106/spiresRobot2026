
from AutoSequencerV2.command import Command
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from subsystems.state.configsubsystem import ConfigSubsystem


class DisableAprilTagCommand(Command):
    def __init__(self):
        if ConfigSubsystem().useCasseroleSwerve() or ConfigSubsystem().useWestwoodSwerve():
            self.drivetrainPoseEstimator = DrivetrainPoseEstimator(DrivetrainControl().getModulePositions())

    def execute(self):
        self.drivetrainPoseEstimator.setUseAprilTags(False)

    def end(self, interrupted):
        self.drivetrainPoseEstimator.setUseAprilTags(True)

    def isDone(self):
        return False
        #return if we're done. Which should be never? It should be controlled by other things
