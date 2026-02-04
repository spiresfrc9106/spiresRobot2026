#!/usr/bin/env python3

import os
import typing
import sys
import gc
import wpilib
import ntcore as nt

from commands2.commandscheduler import CommandScheduler
from pathplannerlib.commands import PathPlannerLogging
import wpilib
import commands2
from phoenix6.signal_logger import SignalLogger

from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger

import westwood.constants
from westwood.robotcontainer import RobotContainer
from westwood.util.logtracer import LogTracer
from westwood.util.phoenixutil import PhoenixUtil
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboard import Dashboard
from testingMotors.motorCtrl import MotorControl, motorDepConstants
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.autoSteer import AutoSteer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.drivetrainDependentConstants import drivetrainDepConstants
from memes.ctreMusicPlayback import CTREMusicPlayback
from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from humanInterface.operatorInterface import OperatorInterface
from navigation.forceGenerators import PointObstacle
from utils.robotIdentification import RobotIdentification
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate, getNowLogger
from utils.calibration import CalibrationWrangler
from utils.crashLogger import CrashLogger
from utils.faults import FaultWrangler
from utils.powerMonitor import PowerMonitor
from utils.rioMonitor import RIOMonitor
from utils.robotIdentification import RobotIdentification
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from webserver.webserver import Webserver
#from fuelSystems.shooterControl import ShooterController
import wpilib

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
        match westwood.constants.kRobotMode:
            case westwood.constants.RobotModes.REAL:
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
            case westwood.constants.RobotModes.SIMULATION:
                Logger.addDataReciever(WPILOGWriter())
                Logger.addDataReciever(NT4Publisher(True))
            case westwood.constants.RobotModes.REPLAY:
                self.useTiming = (
                    False  # Disable timing in replay mode, run as fast as possible
                )
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                Logger.setReplaySource(WPILOGReader(log_path))
                Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
        Logger.start()
        self.container = RobotContainer()

    #########################################################
    ## Common init/update for all modes
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        print("robotInit has run")
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        SignalLogger.enable_auto_logging(False)
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

        print(f"robot type = {RobotIdentification().getRobotType()} serialNumber={RobotIdentification().serialNumber}")

        #self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.
        wpilib.LiveWindow.disableAllTelemetry()

        self.webserver = Webserver()

        self.driveTrain = None
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            print(f"drivetrainDepConstants['HAS_DRIVETRAIN']={drivetrainDepConstants['HAS_DRIVETRAIN']}")
            self.driveTrain = DrivetrainControl()


        self.stt = SegmentTimeTracker(longLoopPrintEnable=False, epochTracerEnable=False)

        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()

        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        #self.shooterCtrl = ShooterController()

        #self.rioMonitor = RIOMonitor()
        #self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        #if self.pwrMon is not None:
        #    self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        #self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        #self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        #self.addPeriodic(FaultWrangler().update, 0.06, 0.0)

        self.autoHasRun = False

        self.logger1 = getNowLogger('now1', 'sec')
        self.logger2 = getNowLogger('now2', 'sec')
        self.logger3 = getNowLogger('now3', 'sec')
        self.count=0

    def robotPeriodic(self) -> None:
        self.logger1.logNow(nt._now())
        self.stt.start()

        #if self.count == 10:
        #    gc.freeze()

        LogTracer.resetOuter("RobotPeriodic")
        PhoenixUtil.updateSignals()
        LogTracer.record("PhoenixUpdate")
        self.container.robotPeriodic()
        LogTracer.record("ContainerPeriodic")
        commands2.CommandScheduler.getInstance().run()
        LogTracer.record("CommandsPeriodic")
        LogTracer.recordTotal()
        self.dInt.update()
        self.stt.mark("Driver Interface")

        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.update()
            self.stt.mark("Drivetrain")

        self.oInt.update()
        self.stt.mark("Operator Interface")

        #self.shooterCtrl.update()
        #self.stt.mark("Shooter Update")

        #self.autodrive.updateTelemetry()
        #self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(self.autodrive.getWaypoints())
        #self.driveTrain.poseEst._telemetry.setCurObstacles(self.autodrive.rfp.getObstacleStrengths())
        self.stt.mark("Telemetry")
        self.logger2.logNow(nt._now())


        #self.ledCtrl.setAutoDriveActive(self.autodrive.isRunning())
        #self.ledCtrl.setAutoSteerActive(self.autosteer.isRunning())
        #self.ledCtrl.setStuck(self.autodrive.rfp.isStuck())
        self.ledCtrl.update()
        self.stt.mark("LED Ctrl")

        logUpdate()
        self.count += 1
        self.stt.end()
        self.logger3.logNow(nt._now())

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        print("autonomousInit has run")

        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            # Use the autonomous routines starting pose to init the pose estimator
            self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())  #position set.

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous rouines starting pose to init the pose estimator
        startPose = self.autoSequencer.getStartingPose()
        if(startPose is not None):
            self.driveTrain.poseEst.setKnownPose(startPose)

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

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)
        # Ensure auto-steer starts disabled, no motion without driver command
        #self.autosteer.setInhibited()


    def teleopPeriodic(self) -> None:
        """This function is called periodically when in teleop"""

        # TODO - this is technically one loop delayed, which could induce lag
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.setManualCmd(self.dInt.getCmd(), self.dInt.getRobotRelative())


            # We're enabled as long as the driver is commanding it, and we're _not_ trying to control robot relative.
            enableAutoSteer = not self.dInt.getRobotRelative() and self.dInt.getAutoSteerEnable()
            #self.autosteer.setAutoSteerActiveCmd(enableAutoSteer)
            #self.autosteer.setAlignToProcessor(self.dInt.getAutoSteerToAlgaeProcessor())
            #self.autosteer.setAlignDownfield(self.dInt.getAutoSteerDownfield())
        
            #self.autodrive.setRequest(self.dInt.getAutoDrive())

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()

        #if self.dInt.getCreateObstacle():
        #    # For test purposes, inject a series of obstacles around the current pose
        #    ct = self.driveTrain.poseEst.getCurEstPose().translation()
        #    tfs = [
        #        #Translation2d(1.7, -0.5),
        #        #Translation2d(0.75, -0.75),
        #        #Translation2d(1.7, 0.5),
        #        Translation2d(0.75, 0.75),
        #        Translation2d(2.0, 0.0),
        #        Translation2d(0.0, 1.0),
        #        Translation2d(0.0, -1.0),
        #    ]
        #    for tf in tfs:
        #        obs = PointObstacle(location=(ct+tf), strength=0.5)
        #        self.autodrive.rfp.addObstacleObservation(obs)

        # No trajectory in Teleop
        Trajectory().setCmd(None)
        if motorDepConstants['HAS_MOTOR_TEST']:
            self.motorCtrlFun.update(self.dInt.getMotorTestPowerRpm())

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
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

        wpilib.LiveWindow.setEnabled(False)
        #CTREMusicPlayback().play()

    def testPeriodic(self):
        pass

    def testExit(self) -> None:
        #CTREMusicPlayback().stop()
        pass

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
