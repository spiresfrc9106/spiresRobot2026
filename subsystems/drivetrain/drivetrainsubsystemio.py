from dataclasses import dataclass, field
from enum import Enum
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d



class DrivetrainSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class DrivetrainSubsystemIOInputs:
        """Initialize required TODO drivetrain inputs."""

        exampleTODO: float = 0.0

    def updateInputs(self, inputs: DrivetrainSubsystemIOInputs):
        pass



