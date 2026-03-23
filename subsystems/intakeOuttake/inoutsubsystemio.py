from dataclasses import dataclass
from pykit.autolog import autolog


class InOutSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class InOutSubsystemIOInputs:
        """Initialize required TODO motor controls."""

        groundTargetIPS: float = 0.0
        hopperTargetIPS: float = 0.0
        flywheelTargetIPS: float = 0.0
        agitatorTargetHz: float = 0.0
        groundIPS: float = 0.0
        hopperIPS: float = 0.0
        flywheelIPS: float = 0.0
        agitatorHz: float = 0.0

    def updateInputs(self, inputs: InOutSubsystemIOInputs):
        pass
