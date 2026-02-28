
from subsystems.drivetrain.drivetrainsubsystemio import DrivetrainSubsystemIO

class DrivetrainSubsystemIOReal(DrivetrainSubsystemIO):

    def __init__(
            self,
            name:str,
    ) -> None:
        self.name = name


    def updateInputs(self, inputs: DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs):
        """Update state of motor per the appropriate specifc API."""
        """
        inputs.turretPosition = Rotation2d(self.position.value * kRadiansPerRevolution)
        inputs.turretSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.turretAppliedVolts = self.applied.value
        inputs.turretSupplyAmps = self.supply.value
        """

