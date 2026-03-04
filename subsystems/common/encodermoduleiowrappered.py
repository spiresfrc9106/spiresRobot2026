from subsystems.common.encodermoduleio import EncoderModuleIO
from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder


class EncoderModuleIOWrappered(EncoderModuleIO):
    def __init__(self, name: str, encoder: WrapperedPulseWidthEncoder) -> None:
        super().__init__(name)
        self.encoder = encoder


    def updateInputs(self, inputs: EncoderModuleIO.EncoderModuleIOInputs) -> None:
        self.encoder.update()
        inputs.faulted = self.encoder.isFaulted()
        inputs.curAngleRad = self.encoder.getAngleRad()

    def getAngleRad(self) -> float:
        return self.encoder.getAngleRad()

    def isFaulted(self) -> bool:
        return self.encoder.isFaulted()


