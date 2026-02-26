from dataclasses import field
from math import isclose
from typing import Optional

from rev import ClosedLoopSlot
from wpimath.controller import SimpleMotorFeedforwardRadians

from constants import kRobotUpdatePeriodS
from subsystems.intakeOuttake.motormodule import MotorModuleCals
from subsystems.intakeOuttake.motormoduleio import MotorModuleIO
from utils.units import radPerSec2RPM


class MotorModuleController:
    def __init__(self, cals: MotorModuleCals) -> None:
        self.cals = cals

    def updatePIDandFF(self, io: MotorModuleIO) -> None:
        raise NotImplementedError

    def updateClosedLoopOutput(self, io: MotorModuleIO, targetRadPerS: float) -> None:
        raise NotImplementedError

    def reset(self, io: MotorModuleIO) -> None:
        io.setVoltage(0.0)


class NullController(MotorModuleController):
    """No-op controller used for replay mode."""
    def __init__(self) -> None:
        pass

    def updatePIDandFF(self, io: MotorModuleIO) -> None:
        pass

    def updateClosedLoopOutput(self, io: MotorModuleIO, targetRadPerS: float) -> None:
        pass

    def reset(self, io: MotorModuleIO) -> None:
        pass


def _secPerRadToMinPerRev(k):
    """Convert kV (V·s/rad) and kA (V·s²/rad) to Spark RPM-based units."""
    k_minPerRev = 0.0 if isclose(k, 0.0) else 1.0 / radPerSec2RPM(1.0 / k)
    return k_minPerRev


class MaxMotionController(MotorModuleController):
    """MaxMotion velocity control with Spark built-in feedforward."""

    def __init__(self, cals: MotorModuleCals) -> None:
        super().__init__(cals)
        self.oldTargetRadPerS: float = 0.0
        self.kA_VoltsMinSecsPerRev: float = field(init=False, default_factory=lambda:_secPerRadToMinPerRev(self.cals.kA.get()))

    def updatePIDandFF(self, io: MotorModuleIO) -> None:
        kV_perRPM = _secPerRadToMinPerRev(self.cals.kV.get())
        self.kA_VoltsMinSecsPerRev = _secPerRadToMinPerRev(self.cals.kA.get())
        io.setPIDFF(self.cals.kP.get(), 0.0, self.cals.kD.get(),
                    self.cals.kS.get(), kV_perRPM, self.kA_VoltsMinSecsPerRev)
        io.setMaxMotionVelParams(self.cals.maxAccIPS2.get())

    def updateClosedLoopOutput(self, io: MotorModuleIO, targetRadPerS: float) -> None:
        if targetRadPerS != self.oldTargetRadPerS:
            if self.oldTargetRadPerS == 0.0:
                io.setFeedForwardKA(self.kA_VoltsMinSecsPerRev)
            if isclose(targetRadPerS, 0.0):
                targetRadPerS = 0.0
                io.setFeedForwardKA(0.0)
                io.setMaxMotionVelCmd(0.0)
                io.setVoltage(0.0)
            else:
                io.setMaxMotionVelCmd(targetRadPerS)
            self.oldTargetRadPerS = targetRadPerS

    def reset(self, io: MotorModuleIO) -> None:
        self.oldTargetRadPerS = 0.0
        io.setFeedForwardKA(0.0)
        io.setVoltage(0.0)


class SparkVelocityController(MotorModuleController):
    """Spark velocity control with Spark built-in feedforward."""

    def updatePIDandFF(self, io: MotorModuleIO) -> None:
        kV_RPM, kA_RPM = _secPerRadToMinPerRev(self.cals.kV.get(), self.cals.kA.get())
        io.setPIDFF(self.cals.kP.get(), 0.0, self.cals.kD.get(),
                    self.cals.kS.get(), kV_RPM, kA_RPM)

    def updateClosedLoopOutput(self, io: MotorModuleIO, targetRadPerS: float) -> None:
        if isclose(targetRadPerS, 0.0):
            io.setVoltage(0.0)
        else:
            io.setVelCmd(targetRadPerS)


class WPILibFFController(MotorModuleController):
    """Spark velocity control with WPILib SimpleMotorFeedforward."""

    def __init__(self, cals: MotorModuleCals) -> None:
        super().__init__(cals)
        self.ff: Optional[SimpleMotorFeedforwardRadians] = None

    def updatePIDandFF(self, io: MotorModuleIO) -> None:
        io.setPID(self.cals.kP.get(), 0.0, self.cals.kD.get())
        self.ff = SimpleMotorFeedforwardRadians(
            self.cals.kS.get(), self.cals.kV.get(), self.cals.kA.get(),
            kRobotUpdatePeriodS)

    def updateClosedLoopOutput(self, io: MotorModuleIO, targetRadPerS: float) -> None:
        if isclose(targetRadPerS, 0.0):
            io.setVoltage(0.0)
        else:
            ffV = self.ff.calculate(targetRadPerS) if self.ff else 0.0
            io.setVelCmd(targetRadPerS, ffV)