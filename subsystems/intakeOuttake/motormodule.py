from pykit.logger import Logger
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from subsystems.intakeOuttake.motormoduleio import MotorModuleIO

from westwood.constants.drive import kMinWheelLinearVelocity, kWheelRadius
from westwood.util.logtracer import LogTracer


class MotorModule:
    def __init__(self, name: str, io: MotorModuleIO) -> None:
        self.name = name
        self.io = io
        self.inputs = MotorModuleIO.MotorModuleIOInputs()

    def periodic(self) -> None:
        LogTracer.resetOuter("MotorModule/" + self.name)
        self.io.updateInputs(self.inputs)
        LogTracer.record("UpdateInputs")
        Logger.processInputs(self.name, self.inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        self.io.setPosCmd(posCmdRad, arbFF)

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        self.io.setVelCmd(velCmdRadps, arbFF)

    def getMotorPositionRad(self)->float:
        return self.inputs.positionRad

    def getMotorVelocityRadPerSec(self)->float:
        return self.inputs.velocityRadps

    def getAppliedOutput(self)->float:
        return self.inputs.appliedV

    def getOutputTorqueCurrentA(self)->float:
        return self.inputs.torqueCurrentA




