from dataclasses import dataclass
from pykit.autolog import autolog

class EncoderModuleIO:
    @autolog
    @dataclass
    class EncoderModuleIOInputs:
        faulted: bool = False
        curAngleRad: float = 0.0

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: EncoderModuleIOInputs) -> None:
        """Update the encoder module I/O inputs.

        Args:
            inputs (EncoderModuleIOInputs): The module I/O inputs to update.
        """

    def getAngleRad(self) -> float:
        pass

    def isFaulted(self) -> bool:
        pass




