from typing import Callable, Tuple

from wpilib.sysid import State

from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from wpilib import XboxController

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from subsystems.common.motormodule import MotorModule

@autologgable_output
class SysIdMotorModules:
    def __init__(self, controller:XboxController, preinit: Callable[[],None], postinit: Callable[[],None], *requirements: Subsystem ):
        self.controller = controller
        self.preinit = preinit
        self.postinit = postinit
        self.requirements = requirements
        self.loggedStateStr = "none"

    def sysIdRoutine(self,
                     name:str,
                     motorModules: Tuple[MotorModule],
                     voltsPerSec:float=0.5,
                     stepVolts:float=6.0,
                     timeoutS:float=10.0) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

        self.loggedStateStr = "none"

        def logOutputs(_)->None:

            for motorModule in motorModules:
                volts = motorModule.io.motor.getDesiredVoltageOrFF()
                Logger.recordOutput(f"inout {motorModule.name} SysId/voltage", volts)
                radsPerSec = motorModule.io.motor.getMotorVelocityRadPerSec()
                Logger.recordOutput(f"inout {motorModule.name} SysId/radsPerSec", radsPerSec)
                rad = motorModule.io.motor.getMotorPositionRad()
                Logger.recordOutput(f"inout {motorModule.name} SysId/rad", rad)

                #print(f"{Timer.getTimestamp():.6} {name} {self.loggedStateStr} SysID: volts={volts}, radsPerSec={radsPerSec}, rad={rad}")
                assert type(volts) == float, f"volts is not a float: {volts}"
                assert type(radsPerSec) == float, f"radsPerSec is not a float: {radsPerSec}"
                assert type(rad) == float, f"rad is not a float: {rad}"
                assert -1.0e10 <= volts <= 1.0e10, f"volts is out of bounds: {volts}"
                assert -1.0e10 <= radsPerSec <= 1.0e10, f"radsPerSec is out of bounds: {radsPerSec}"
                assert -1.0e10 <= rad <= 1.0e10, f"rad is out of bounds: {rad}"

        def logState(state: State) -> None:
            match state:
                case State.kQuasistaticForward:
                    self.loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    self.loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    self.loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    self.loggedStateStr = "dynamic-reverse"
                case State.kNone|_:
                    self.loggedStateStr = "none"
            Logger.recordOutput(f"inout {name} SysId/state", self.loggedStateStr)

        def setVoltage(v:float)->None:
            for motorModule in motorModules:
                motorModule.setVoltage(v)

        def allMotorModulesAreStopped()->bool:
            stopped = True
            for motorModule in motorModules:
                if abs(motorModule.getMotorVelocityRadPerSec()) >= 0.1:
                    stopped = False
            return stopped

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                voltsPerSec, # ramp voltage rate in V/sec
                stepVolts, # step voltage in V
                timeoutS, # timeout in seconds
                logState),
            SysIdRoutine.Mechanism(
                setVoltage,
                logOutputs,
                self.requirements[0]
            ),
        )

        sysIdQFWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper() and allMotorModulesAreStopped())
        sysIdQFCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        sysIdQFWithWaits = sysIdQFWaitCmd.andThen(sysIdQFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdQRWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper() and allMotorModulesAreStopped())
        sysIdQRCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        sysIdQRWithWaits = sysIdQRWaitCmd.andThen(sysIdQRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdDFWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper()  and allMotorModulesAreStopped())
        sysIdDFCmd = charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward)
        sysIdDFWithWaits = sysIdDFWaitCmd.andThen(sysIdDFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdDRWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper()  and allMotorModulesAreStopped())
        sysIdDRCmd = charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        sysIdDRWithWaits = sysIdDRWaitCmd.andThen(sysIdDRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        def beforeTests()->None:
            self.preinit()

        return cmd.sequence(
            cmd.runOnce(beforeTests, self),
            sysIdQFWithWaits,
            sysIdQRWithWaits,
            sysIdDFWithWaits,
            sysIdDRWithWaits,
            cmd.runOnce(lambda: self.postinit(), self),
        )
