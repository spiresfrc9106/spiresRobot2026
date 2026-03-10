from typing import Optional

from commands2 import Command, cmd, CommandScheduler
from wpimath.geometry import Pose2d, Rotation2d

from constants import kFieldLengthIn, kFieldWidthIn
from humanInterface.driverInterface import DriverInterface
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from subsystems.drivetrain.drivetrainsubsystem import DrivetrainSubsystemFactory, DrivetrainSubsystem
from subsystems.intakeOuttake.inoutsubsystem import InOutSubsystem, inoutSubsystemFactory

from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from utils.allianceTransformUtils import onRed
from utils.units import deg2Rad, in2m


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, xyzzy, and button mappings) should be declared here.
    """
    """
    Spires addendum to RobotContainer design, the robot container should know very little about the robot's subsystems,
    or how to build them or what they connect to. That information should be in the subsystem classes.
    """
    autoOrTestCommand: Optional[Command] = None

    def __init__(self) -> None:
        # The robot's subsystems
        self.config = ConfigSubsystem()
        self.robotop = RobotTopSubsystem()
        self.inout: InOutSubsystem|None = inoutSubsystemFactory()
        self.drivetrainSubsystem: DrivetrainSubsystem|None = None
        if ConfigSubsystem().useCasseroleSwerve():
            self.drivetrainSubsystem = DrivetrainSubsystemFactory()

        self.autoHasRun = False


        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Move Auto Choices Here"
        )
        self.autoChooser.setDefaultOption("Do Nothing Once", cmd.none())

        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        if self.inout is not None:
            self.testChooser.addOption(
                "inout FF ground",
                self.inout.makeCommandFeedForwardCharacterizationGroundMotor()
            )
            self.testChooser.addOption(
                "inout FF hopper",
                self.inout.makeCommandFeedForwardCharacterizationHopperMotor()
            )
            self.testChooser.addOption(
                "inout FF flywheel",
                self.inout.makeCommandFeedForwardCharacterizationFlywheelMotor()
            )
            self.testChooser.addOption(
                f"inout SysId ground",
                self.inout.makeSysIdCommandGroundMotor()
            )
            self.testChooser.addOption(
                f"inout SysId hopper",
                self.inout.makeSysIdCommandHopperMotor()
            )
            self.testChooser.addOption(
                f"inout SysId flywheel",
                self.inout.makeSysIdCommandFlywheelMotor()
            )

        self.testChooser.setDefaultOption("Do Nothing Once", cmd.none())

    def robotPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.autoOrTestCommand = self.autoChooser.getSelected()
        self.autonomousOrTestCommonInit()
        self.autoHasRun = True


    def teleopInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
        if self.inout is not None:
           CommandScheduler.getInstance().schedule(self.inout.aOperatorRunsInoutCommand())
        # clear existing telemetry trajectory
        if self.drivetrainSubsystem is not None:
            # xyzzy todo
            # we always have a default autonoumous pose?
            # that if auto hasn't run, we set our default poss to the default, or selected autonoumous pose?
            # -Thanks Coach Mike
            if not self.autoHasRun:
                robotStartXIn = 40.0
                robotStartYIn = 80.0
                if onRed():
                    self.drivetrainSubsystem.casseroleDrivetrain.poseEst.setKnownPose(
                        Pose2d(in2m(kFieldLengthIn-robotStartXIn), in2m(kFieldWidthIn-robotStartYIn), Rotation2d(deg2Rad(180)))
                    )
                else:
                    self.drivetrainSubsystem.casseroleDrivetrain.poseEst.setKnownPose(
                        Pose2d(in2m(robotStartXIn), in2m(robotStartYIn), Rotation2d(deg2Rad(0)))
                    )
            self.drivetrainSubsystem.casseroleDrivetrain.poseEst._telemetry.setCurAutoTrajectory(None)
            self.drivetrainSubsystem.setDefaultCommand(
                self.drivetrainSubsystem.arcadeDriveClosedLoop(DriverInterface().getCmd)
            )

    def testInit(self) -> None:
        self.autoOrTestCommand = self.testChooser.getSelected()
        self.autonomousOrTestCommonInit()

    def autonomousOrTestCommonInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
        if self.autoOrTestCommand is not None:
            CommandScheduler.getInstance().schedule(self.autoOrTestCommand)

    def quietRobotOnExitFromActiveMode(self) -> None:
        if self.autoOrTestCommand is not None:
            self.autoOrTestCommand.cancel()
            self.autoOrTestCommand = None

        if self.inout is not None:
            self.inout.initialize()

