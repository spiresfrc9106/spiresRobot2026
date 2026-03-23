from subsystems.drivetrain.drivetrainsubsystemio import DrivetrainSubsystemIO
from subsystems.drivetrain.drivetrainsubsystemioreal import DrivetrainSubsystemIOReal



class DrivetrainSubsystemIORealSim(DrivetrainSubsystemIOReal):
    """Simulate the motors"""

    def __init__(self,
        name: str,
    ) -> None:
        super().__init__(name)  # Initialize the Sim motor the same way as the actual Talon motor

    def updateInputs(self, inputs: DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs) -> None:
        """Simulate the motor behavior, then update TalonIO inputs."""
        super().updateInputs(inputs)  # Call the TalonIO updateInputs method
