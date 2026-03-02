from subsystems.common.encodermoduleio import EncoderModuleIO
from subsystems.common.encodermoduleiowrappered import EncoderModuleIOWrappered
from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder


class EncoderModuleIOWrapperedSim(EncoderModuleIOWrappered):
    def __init__(self, name: str, encoder: WrapperedPulseWidthEncoder) -> None:
        super().__init__(name, encoder)

    def updateInputs(self, inputs: EncoderModuleIO.EncoderModuleIOInputs) -> None:
        return super().updateInputs(inputs)

