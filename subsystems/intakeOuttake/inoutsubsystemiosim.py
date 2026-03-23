from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO
from subsystems.intakeOuttake.inoutsubsystemioreal import InOutSubsystemIOReal


class InOutSubsystemIORealSim(InOutSubsystemIOReal):
    """Simulate the motors"""

    def __init__(
        self,
        name: str,
    ) -> None:
        super().__init__(
            name
        )  # Initialize the Sim motor the same way as the actual Talon motor
        pass

    def updateInputs(self, inputs: InOutSubsystemIO.InOutSubsystemIOInputs) -> None:
        """Simulate the motor behavior, then update TalonIO inputs."""

        # do simulated motor behavior here

        super().updateInputs(inputs)  # Call the TalonIO updateInputs method
