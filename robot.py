#!/usr/bin/env python3

import os
import typing
import sys

from commands2.commandscheduler import CommandScheduler

from pathplannerlib.commands import PathPlannerLogging
import commands2

from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger

import constants
from drivetrain.drivetrainDependentConstants import DrivetrainDependentConstants

from robotcontainer import RobotContainer
from utils.calibration import CalibrationWrangler

if DrivetrainDependentConstants().useWestwoodSwerve:
    from westwood.westwoodrobotcontainer import WestwoodRobotContainer
from westwood.util.logtracer import LogTracer
from westwood.util.phoenixutil import PhoenixUtil

from AutoSequencerV2.autoSequencer import AutoSequencer
from testingMotors.motorCtrl import motorDepConstants, MotorControl

from drivetrain.controlStrategies.trajectory import Trajectory

from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from humanInterface.operatorInterface import OperatorInterface
from utils.singleton import destroyAllSingletonInstances
from subsystems.state.configsubsystem import ConfigSubsystem
from wpilib import Timer
import wpilib

LoggedRobot.default_period = constants.kRobotUpdatePeriodS
class MyRobot(LoggedRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def __init__(self):
        super().__init__()
        Logger.recordMetadata("Robot", type(self).__name__)
        match constants.kRobotMode:
            case constants.RobotModes.REAL|constants.RobotModes.SIMULATION:
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                    Logger.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                Logger.addDataReciever(NT4Publisher(True))
                Logger.addDataReciever(WPILOGWriter())
            case constants.RobotModes.REPLAY:
                self.useTiming = (
                    False  # Disable timing in replay mode, run as fast as possible
                    #True # replay with timing
                )
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                replaySource = WPILOGReader(log_path)
                Logger.setReplaySource(replaySource)
                Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
        Logger.start()

        self.container = RobotContainer()
        self.westwoodContainer = None
        if ConfigSubsystem().useWestwoodSwerve():
            self.westwoodContainer = WestwoodRobotContainer()

    #########################################################
    ## Common init/update for all modes
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.count=0
        #print(f"{self.count} robotInit has run")
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


        #self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.

        self.driveTrain = None
        #print(f"useCasseroleSwerve()={ConfigSubsystem().useCasseroleSwerve()}")
        if ConfigSubsystem().useCasseroleSwerve():
            self.driveTrain = DrivetrainControl()


        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()

        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        if motorDepConstants['HAS_MOTOR_TEST']:
            self.motorCtrlFun = MotorControl()

        #self.shooterCtrl = ShooterController()

        #self.rioMonitor = RIOMonitor()
        #self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        #if self.pwrMon is not None:
        #    self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        #self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        #self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.cw = CalibrationWrangler()
        #self.addPeriodic(FaultWrangler().update, 0.06, 0.0)

        self.autoHasRun = False

    def robotPeriodic(self) -> None:
        #print(f"{self.count} robotPeriodic")

        #if self.count == 10:
        #    gc.freeze()

        LogTracer.resetOuter("RobotPeriodic")
        if self.westwoodContainer is not None:
            PhoenixUtil.updateSignals()
            LogTracer.record("PhoenixUpdate")
            self.westwoodContainer.robotPeriodic()
            LogTracer.record("WestwoodContainerPeriodic")
        self.container.robotPeriodic()
        LogTracer.record("ContainerPeriodic")

        self.dInt.update()

        if self.driveTrain is not None:
            self.driveTrain.update()

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


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        #print("autonomousInit has run")

        if self.westwoodContainer is not None:
            self.autonomousCommand = self.westwoodContainer.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        self.container.autonomousInit()

        # Use the autonomous rouines starting pose to init the pose estimator
        startPose = self.autoSequencer.getStartingPose()
        if startPose is not None:
            if self.driveTrain is not None:
                # Use the autonomous routines starting pose to init the pose estimator
                self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())  # position set.

        # Mark we at least started autonomous
        self.autoHasRun = True # pylint: disable=attribute-defined-outside-init

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

        # Do not run autosteer in autonomous
        #self.autosteer.setAutoSteerActiveCmd(False)

        self.autoSequencer.update()

        # Operators cannot control in autonomous
        #self.driveTrain.setManualCmd(DrivetrainCommand())

    def autonomousExit(self):
        self.autoSequencer.end()
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self) -> None:
        self.container.teleopInit()
        #print(f"{self.count} teleopInit")
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # clear existing telemetry trajectory
        if self.driveTrain is not None:
            self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)
        # Ensure auto-steer starts disabled, no motion without driver command
        #self.autosteer.setInhibited()


    def teleopPeriodic(self) -> None:
        """This function is called periodically when in teleop"""
        #print(f"{self.count} teleopPeriodic")

        # TODO - this is technically one loop delayed, which could induce lag
        if self.driveTrain is not None:
            self.driveTrain.setManualCmd(self.dInt.getCmd(), self.dInt.getRobotRelative())


            # We're enabled as long as the driver is commanding it, and we're _not_ trying to control robot relative.
            enableAutoSteer = not self.dInt.getRobotRelative() and self.dInt.getAutoSteerEnable()
            #self.autosteer.setAutoSteerActiveCmd(enableAutoSteer)
            #self.autosteer.setAlignToProcessor(self.dInt.getAutoSteerToAlgaeProcessor())
            #self.autosteer.setAlignDownfield(self.dInt.getAutoSteerDownfield())
        
            #self.autodrive.setRequest(self.dInt.getAutoDrive())

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()

        # No trajectory in Teleop
        Trajectory().setCmd(None)
        if motorDepConstants['HAS_MOTOR_TEST']:
            self.motorCtrlFun.update(100.0)

    def teleopExit(self):
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    #########################################################
    ## Test-Specific init and update
    def testInit(self) -> None:
        self.container.testInit()

    def testPeriodic(self):
        pass

    def testExit(self) -> None:
        #CTREMusicPlayback().stop()
        self.container.quietRobotOnExitFromActiveMode()

    #########################################################
    ## Cleanup
    def endCompetition(self):
        print("Goodbye!")

        # Stop robot code exectuion first
        super().endCompetition()

        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
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
