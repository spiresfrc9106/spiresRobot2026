from subsystems.common.encodermoduleio import EncoderModuleIO


class EncoderModuleIOSim(EncoderModuleIO):
    """Stateful simulation IO for an absolute encoder — no real hardware."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._simAngleRad: float = 0.0

    def setSimAngleRad(self, angleRad: float) -> None:
        self._simAngleRad = angleRad

    def updateInputs(self, inputs: EncoderModuleIO.EncoderModuleIOInputs) -> None:
        inputs.faulted = False
        inputs.curAngleRad = self._simAngleRad
