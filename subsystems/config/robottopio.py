from dataclasses import dataclass, field
from pykit.autolog import autolog

from wpilib import RobotController




class RobotTopIO:
    """Process I/O data for the robot high-level state subsystem."""

    @autolog
    @dataclass
    class RobotTopIOInputs:
        """Hold I/O data for the robot high-level state subsystem."""
        timeUSec: int = 0

    def updateInputs(self, inputs: RobotTopIOInputs) -> None:
        """Update the robot high-level state I/O inputs.

        Args:
            inputs (RobotTopIOInputs): The robot high-level state I/O inputs to update.
        """
        inputs.timeUSec = RobotController.getFPGATime()


