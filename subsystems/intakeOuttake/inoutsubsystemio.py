from dataclasses import dataclass, field
from enum import Enum
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d



class InOutSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class InOutSubsystemIOInputs:
        """Initialize required TODO motor controls."""

        groundTargetIPS: float = 0.0
        hopperTargetIPS: float = 0.0
        flywheelTargetIPS: float = 0.0

    def updateInputs(self, inputs: InOutSubsystemIOInputs):
        pass



