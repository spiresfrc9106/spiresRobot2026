from wrappers.wrapperedMotorCommon import MotorControlStates

class WrapperedMotorSuper:

    def setFollow(self, leaderCanID:int, invert:bool=False)->None:
        assert False, "Not implemented"

    def setInverted(self, isInverted:bool)->None:
        assert False, "Not implemented"

    def setPID(self, kP:float, kI:float, kD:float)->None:
        assert False, "Not implemented"

    def setPIDFF(self, kP: float, kI: float, kD: float, kS: float, kV: float, kA: float) -> None:
        assert False, "Not implemented"

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        assert False, "Not implemented"

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        assert False, "Not implemented"

    def setVoltage(self, outputVoltageVolts:float)->None:
        assert False, "Not implemented"

    def getMotorPositionRad(self)->float:
        assert False, "Not implemented"

    def getMotorVelocityRadPerSec(self)->float:
        assert False, "Not implemented"

    def getAppliedOutput(self)->float:
        assert False, "Not implemented"

    def getDesiredVoltageOrFF(self) -> float:
        assert False, "Not implemented"

    def getCurrentLimitA(self)->int:
        assert False, "Not implemented"

    def getControlState(self)->MotorControlStates:
        assert False, "Not implemented"

    def setSmartCurrentLimit(self, currentLimitA: int)->None:
        assert False, "Not implemented"

    def getOutputTorqueCurrentA(self)->float:
        assert False, "Not implemented"
