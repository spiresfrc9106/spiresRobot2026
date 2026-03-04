
from pykit.logger import Logger

from subsystems.common.encodermoduleio import EncoderModuleIO

from westwood.util.logtracer import LogTracer


class EncoderModule:
    def __init__(self, name: str, io: EncoderModuleIO) -> None:
        self.name = name
        self.io = io
        self.inputs = EncoderModuleIO.EncoderModuleIOInputs()

    def periodic(self) -> None:
        LogTracer.resetOuter("EncoderModule/" + self.name)
        self.io.updateInputs(self.inputs)
        LogTracer.record("UpdateInputs")
        Logger.processInputs(self.name, self.inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()

    def getAngleRad(self) -> float:
        return self.inputs.curAngleRad

    def isFaulted(self) -> bool:
        return self.inputs.faulted