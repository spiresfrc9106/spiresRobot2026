from dataclasses import dataclass
from pykit.autolog import autolog


class DrivetrainSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class DrivetrainSubsystemIOInputs:
        """Initialize required TODO drivetrain inputs."""

    def updateInputs(self, inputs: DrivetrainSubsystemIOInputs):
        pass
