from typing import Callable

from wpilib.sysid import State

from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from wpilib import XboxController, Timer

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from subsystems.common.motormodule import MotorModule

@autologgable_output
class SysIdMotorModule:
    def __init__(self, controller:XboxController, preinit: Callable[[],None], postinit: Callable[[],None], *requirements: Subsystem ):
        self.controller = controller
        self.preinit = preinit
        self.postinit = postinit
        self.requirements = requirements
        self.loggedStateStr = "none"


    def feedForwardCharacterization(self, motorModule: MotorModule) -> Command:
        velocitySamples: list[float] = []
        voltageSamples: list[float] = []
        timer = Timer()

        def setup():
            self.preinit()
            velocitySamples.clear()
            voltageSamples.clear()
            timer.restart()

        def run():
            voltage = timer.get() * motorModule.ffCharacterizationRampVPerS
            motorModule.setVoltage(voltage)
            velocitySamples.append(motorModule.getMotorVelocityRadPerSec())
            voltageSamples.append(voltage)

        def end(_interrupted: bool):
            n = len(velocitySamples)
            sumX = sum(velocitySamples)
            sumY = sum(voltageSamples)
            sumXY = sum(velocitySamples[i] * voltageSamples[i] for i in range(n))
            sumX2 = sum(v**2 for v in velocitySamples)
            kS = 0.0
            kV = 0.0
            divisor = (n * sumX2 - sumX * sumX)
            if divisor != 0.0:
                kS = (sumY * sumX2 - sumX * sumXY) / divisor
                kV = (n * sumXY - sumX * sumY) / divisor

            print("************************************************************")
            print(f"{motorModule.name} Feed Forward Characterization Results: \nkS = {kS}\nkV = {kV}")
            self.postinit()

        ffWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        ffCmd = (cmd.runOnce(setup, self.requirements)
                 .andThen(cmd.run(run, self.requirements).withName(f"{motorModule.name} FeedForwardCharacterization"))
                 .finallyDo(end))
        ffWithWaits = ffWaitCmd.andThen(ffCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        return ffWithWaits


    def sysIdRoutine(self,
                     name:str,
                     motorModule: MotorModule,
                     voltsPerSec:float=0.5,
                     stepVolts:float=6.0,
                     timeoutS:float=10.0) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

        self.loggedStateStr = "none"

        def logOutputs(_)->None:

            volts = motorModule.io.motor.getDesiredVoltageOrFF()
            Logger.recordOutput(f"inout {name} SysId/voltage", volts)
            radsPerSec = motorModule.io.motor.getMotorVelocityRadPerSec()
            Logger.recordOutput(f"inout {name} SysId/radsPerSec", radsPerSec)
            rad = motorModule.io.motor.getMotorPositionRad()
            Logger.recordOutput(f"inout {name} SysId/rad", rad)

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

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                voltsPerSec, # ramp voltage rate in V/sec
                stepVolts, # step voltage in V
                timeoutS, # timeout in seconds
                logState),
            SysIdRoutine.Mechanism(
                motorModule.setVoltage,
                logOutputs,
                self.requirements[0]
            ),
        )

        sysIdQFWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper() and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdQFCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        sysIdQFWithWaits = sysIdQFWaitCmd.andThen(sysIdQFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdQRWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper() and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdQRCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        sysIdQRWithWaits = sysIdQRWaitCmd.andThen(sysIdQRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdDFWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdDFCmd = charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward)
        sysIdDFWithWaits = sysIdDFWaitCmd.andThen(sysIdDFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        sysIdDRWaitCmd = cmd.waitUntil(
            lambda: self.controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
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
