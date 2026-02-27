from subsystems.common.motormoduleio import MotorModuleIO
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

class MotorModuleIOWrappered(MotorModuleIO):
    def __init__(self, name: str, motor: WrapperedMotorSuper) -> None:
        super().__init__(name)
        self.motor = motor

    def setPID(self, kP: float, kI: float, kD: float) -> None:
        self.motor.setPID(kP, kI, kD)

    def setPIDFF(self, kP: float, kI: float, kD: float, kS: float, kV: float, kA: float) -> None:
        self.motor.setPIDFF(kP, kI, kD, kS, kV, kA)

    def setMaxMotionVelParams(self, maxAccRadps2: float) -> None:
        self.motor.setMaxMotionVelParams(maxAccRadps2)

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        self.motor.setPosCmd(posCmdRad, arbFF)

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        self.motor.setVelCmd(velCmdRadps, arbFF)

    def setMaxMotionVelCmd(self, velCmdRadps: float) -> None:
        self.motor.setMaxMotionVelCmd(velCmdRadps)

    def setVoltage(self, outputVoltageVolts:float)->None:
        self.motor.setVoltage(outputVoltageVolts)

    def setFeedForwardKA(self, kA: float) -> None:
        self.motor.setFeedForwardKA(kA)

    def updateInputs(self, inputs: MotorModuleIO.MotorModuleIOInputs) -> None:
        inputs.desVoltsOrFfVolts = self.motor.getDesiredVoltageOrFF()
        inputs.posRad = self.motor.getMotorPositionRad()
        inputs.desVelRadps = self.motor.desVelRadps
        inputs.velRadps = self.motor.getMotorVelocityRadPerSec()
        inputs.appliedV = self.motor.getAppliedOutput()
        inputs.drive_supply_current = 0.0
        inputs.torqueCurrentA = self.motor.getOutputTorqueCurrentA()

