
from wpimath.geometry import Rotation2d

from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO

class InOutSubsystemIOReal(InOutSubsystemIO):

    def __init__(
            self,
            name:str,
    ) -> None:
        self.name = name


    def updateInputs(self, inputs: InOutSubsystemIO.InOutSubsystemIOInputs):
        """Update state of motor per the appropriate specifc API."""
        """
        inputs.turretPosition = Rotation2d(self.position.value * kRadiansPerRevolution)
        inputs.turretSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.turretAppliedVolts = self.applied.value
        inputs.turretSupplyAmps = self.supply.value
        """

