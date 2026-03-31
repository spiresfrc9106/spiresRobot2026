#!/usr/bin/env python3

import sys

from commands2.commandscheduler import CommandScheduler

from pathplannerlib.commands import PathPlannerLogging
import commands2

from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger
from utils.robotLoggerSetup import RobotLoggerSetup

import constants

from robotcontainer import RobotContainer
from utils.calibration import CalibrationWrangler

from util.logtracer import LogTracer

from testingMotors.motorCtrl import motorDepConstants, MotorControl

# from drivetrain.controlStrategies.trajectory import Trajectory

from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from humanInterface.operatorInterface import OperatorInterface
from utils.singleton import destroyAllSingletonInstances
from wpilib import Timer
import wpilib

LoggedRobot.default_period = constants.kRobotUpdatePeriodS


class MyRobot(LoggedRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def __init__(self):
        super().__init__()
        self.loggerSetup = RobotLoggerSetup(type(self).__name__)
        self.useTiming = self.loggerSetup.useTiming

        self.container = RobotContainer()

    #########################################################
    ## Common init/update for all modes
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.count = 0
        # print(f"{self.count} robotInit has run")
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        wpilib.LiveWindow.disableAllTelemetry()
        CommandScheduler.getInstance().setPeriod(
            1
        )  # 1s period for command scheduler watchdog

        commandCount: dict[str, int] = {}

        def logCommandFunction(command: commands2.Command, active: bool) -> None:
            name = command.getName()
            count = commandCount.get(name, 0) + (1 if active else -1)
            commandCount[name] = count
            Logger.recordOutput(f"Commands/{name}", count > 0)

        CommandScheduler.getInstance().onCommandInitialize(
            lambda c: logCommandFunction(c, True)
        )
        CommandScheduler.getInstance().onCommandFinish(
            lambda c: logCommandFunction(c, False)
        )
        CommandScheduler.getInstance().onCommandInterrupt(
            lambda c: logCommandFunction(c, False)
        )

        PathPlannerLogging.setLogCurrentPoseCallback(
            lambda pose: Logger.recordOutput("PathPlanner/CurrentPose", pose)
        )
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda pose: Logger.recordOutput("PathPlanner/TargetPose", pose)
        )
        PathPlannerLogging.setLogActivePathCallback(
            lambda poses: Logger.recordOutput("PathPlanner/CurrentPath", poses)
        )

        # self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.

        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()

        self.ledCtrl = LEDControl()

        if motorDepConstants["HAS_MOTOR_TEST"]:
            self.motorCtrlFun = MotorControl()

        # self.shooterCtrl = ShooterController()

        # self.rioMonitor = RIOMonitor()
        # self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        # if self.pwrMon is not None:
        #    self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        # self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        # self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.cw = CalibrationWrangler()
        # self.addPeriodic(FaultWrangler().update, 0.06, 0.0)

    def robotPeriodic(self) -> None:
        # print(f"{self.count} robotPeriodic")

        # if self.count == 10:
        #    gc.freeze()

        LogTracer.resetOuter("RobotPeriodic")
        self.container.robotPeriodic()
        LogTracer.record("ContainerPeriodic")

        self.dInt.update()

        self.oInt.update()
        self.cw.update()

        self.ledCtrl.update()

        self.count += 1
        LogTracer.record("OtherUpdates")
        LogTracer.recordTotal()

        # We need to close log tracer with LogTracer.recordTotal(), above,
        # because it is a singleton and
        # the next line will open new top level tracers.
        commands2.CommandScheduler.getInstance().run()

    #########################################################
    ## Disabled-Specific init and update

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        # Trajectory().trajHDC.updateCals()

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        # print("autonomousInit has run")

        self.container.autonomousInit()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def autonomousExit(self):
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self) -> None:
        self.container.teleopInit()
        # print(f"{self.count} teleopInit")

    def teleopPeriodic(self) -> None:
        """This function is called periodically when in teleop"""
        # print(f"{self.count} teleopPeriodic")

        if self.dInt.getGyroResetCmd():
            if self.container.drivetrainSubsystem is not None:
                self.container.drivetrainSubsystem.casseroleDrivetrain.resetGyro()

        # No trajectory in Teleop
        # Trajectory().setCmd(None)
        if motorDepConstants["HAS_MOTOR_TEST"]:
            self.motorCtrlFun.update(100.0)

    def teleopExit(self):
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Test-Specific init and update
    def testInit(self) -> None:
        self.container.testInit()

    def testPeriodic(self):
        pass

    def testExit(self) -> None:
        # CTREMusicPlayback().stop()
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Cleanup
    def endCompetition(self):

        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
        from utils.singleton import _instances

        print(f"Goodbye! utils.singleton.instances={_instances}")

        super().endCompetition()


def printOverrunMessage(self):
    print(f"{Timer.getFPGATimestamp():.3f} Loop overrun detected!")


def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()


if __name__ == "__main__":
    wpilib.run(MyRobot)
