from dataclasses import dataclass
from typing import TYPE_CHECKING

from pykit.logger import Logger

from subsystems.common.motormoduleio import MotorModuleIO

from utils.calibration import Calibration
from westwood.util.logtracer import LogTracer

if TYPE_CHECKING:
    from subsystems.common.motormodulecontroller import MotorModuleController


@dataclass
class MotorModuleCals:
    kP: Calibration
    kD: Calibration
    kS: Calibration
    kV: Calibration
    kA: Calibration
    maxAccUserUnitsPerS2: Calibration


class MotorModule:
    def __init__(self, name: str, io: MotorModuleIO,
                 controller: 'MotorModuleController',
                 ffCharacterizationRampVPerS=0.1) -> None:
        self.name = name
        self.io = io
        self.controller = controller
        self.ffCharacterizationRampVPerS = ffCharacterizationRampVPerS
        self.inputs = MotorModuleIO.MotorModuleIOInputs()

    def periodic(self) -> None:
        LogTracer.resetOuter("MotorModule/" + self.name)
        self.io.updateInputs(self.inputs)
        LogTracer.record("UpdateInputs")
        Logger.processInputs(self.name, self.inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()

    def updatePIDandFF(self) -> None:
        self.controller.updatePIDandFF(self.io)

    def updateClosedLoopOutput(self, targetRadPerS: float) -> None:
        self.controller.updateClosedLoopOutput(self.io, targetRadPerS)

    def reset(self) -> None:
        self.controller.reset(self.io)

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        self.io.setPosCmd(posCmdRad, arbFF)

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        self.io.setVelCmd(velCmdRadps, arbFF)

    def setMaxMotionVelCmd(self, velCmdRadps: float):
        self.io.setMaxMotionVelCmd(velCmdRadps)

    def setVoltage(self, outputVoltageVolts:float)->None:
        self.io.setVoltage(outputVoltageVolts)

    def getMotorPositionRad(self)->float:
        return self.inputs.posRad

    def getMotorVelocityRadPerSec(self)->float:
        return self.inputs.velRadps

    def getAppliedOutput(self)->float:
        return self.inputs.appliedV

    def getdesVoltsOrFfVolts(self)->float:
        return self.inputs.desVoltsOrFfVolts

    def getOutputTorqueCurrentA(self)->float:
        return self.inputs.torqueCurrentA