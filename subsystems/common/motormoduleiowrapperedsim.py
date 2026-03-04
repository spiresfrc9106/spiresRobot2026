from subsystems.common.motormoduleio import MotorModuleIO
from subsystems.common.motormoduleiowrappered import MotorModuleIOWrappered
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

class MotorModuleIOWrapperedSim(MotorModuleIOWrappered):
    def __init__(self, name: str, motor: WrapperedMotorSuper) -> None:
        super().__init__(name, motor)

    def updateInputs(self, inputs: MotorModuleIO.MotorModuleIOInputs) -> None:
        return super().updateInputs(inputs)

