import os
from typing import Optional

import commands2
import wpilib
from commands2 import Command, cmd, CommandScheduler
from pathplannerlib.auto import PathPlannerAuto, AutoBuilder, NamedCommands
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition

from constants import kFieldLengthIn, kFieldWidthIn, kRobotMode, RobotModes
from constants.field import poseTransformedForAlliance
from drivetrain.drivetrainPhysical import DrivetrainPhysical
from humanInterface.driverInterface import DriverInterface
from pykit.logger import Logger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from robotstate import RobotState
from subsystems.drivetrain.drivetrainsubsystem import DrivetrainSubsystemFactory, DrivetrainSubsystem
from subsystems.intakeOuttake.inoutsubsystem import InOutSubsystem, inoutSubsystemFactory

from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from subsystems.vision.visionsubsystem import VisionSubsystem, VisionSubsystemFactory
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
        self.visionSubsystem: VisionSubsystem|None = VisionSubsystemFactory()

        if self.inout is not None:
            NamedCommands.registerCommand("spinUpFlywheel", self.inout.spinUpFlywheelCommand())
            NamedCommands.registerCommand("shoot", self.inout.shootCommand())
            NamedCommands.registerCommand("spinUpAndShoot", self.inout.spinUpAndShootCommand())
        if self.drivetrainSubsystem is not None:
            p = DrivetrainPhysical()
            AutoBuilder.configure(
                self.drivetrainSubsystem.casseroleDrivetrain.getCurEstPose,
                self.resetPose,
                self.drivetrainSubsystem.casseroleDrivetrain.getRobotRelativeChassisSpeeds,
                self.drivetrainSubsystem.drivePathPlanned,
                PPHolonomicDriveController(
                    p.kPathFollowingTranslationConstantsAuto,
                    p.kPathFollowingRotationConstants,
                ),
                # controller
                RobotConfig.fromGUISettings(),
                # robot_config
                onRed,
                self.drivetrainSubsystem,
            )

        self.autoHasRun = False


        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Auto Choices"
        )
        if self.drivetrainSubsystem is not None:
            pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
            for file in os.listdir(pathsPath):
                relevantName = file.split(".")[0]
                print(f"Adding auto {relevantName}")
                autonLeft = PathPlannerAuto(relevantName)
                leftPose =autonLeft._startingPose
                autonRight = PathPlannerAuto(relevantName, mirror=True)
                rightPose = autonRight._startingPose
                name = f"left -{relevantName}"
                self.autoChooser.addOption(name, autonLeft)
                name = f"right-{relevantName}"
                self.autoChooser.addOption(name, autonRight)

                name = f"sh-left -{relevantName}"
                self.autoChooser.addOption(
                        name,
                        commands2.SequentialCommandGroup(
                            cmd.runOnce(lambda p=leftPose: self.resetPose(poseTransformedForAlliance(p)),self.drivetrainSubsystem),
                            self.inout.spinUpAndShootCommand(),
                            autonLeft),
                )
                name = f"sh-right-{relevantName}"
                self.autoChooser.addOption(
                        name,
                        commands2.SequentialCommandGroup(
                            cmd.runOnce(lambda p=rightPose: self.resetPose(poseTransformedForAlliance(p)),self.drivetrainSubsystem),
                            self.inout.spinUpAndShootCommand(),
                            autonRight),
                )

        self.autoChooser.setDefaultOption("Do Nothing Once", cmd.none())

        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        if self.drivetrainSubsystem is not None:
            self.testChooser.addOption(
                "drivetrain SysId wheel",
                self.drivetrainSubsystem.makeSysIdCommandWheelMotors()
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
        if self.visionSubsystem is not None:
            if self.drivetrainSubsystem is None:
                RobotState.periodic(
                    Rotation2d().fromDegrees(0.0),  # self.drive.getRawRotation(),
                    Logger.getTimestamp()/1e6,
                    0.0,  # self.drive.getAngularVelocity(),
                    ChassisSpeeds(),  # self.drive.getFieldRelativeSpeeds(),
                    (SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition()),
                    # self.drive.getModulePositions(),
                    Rotation2d().fromDegrees(0.0),  # self.turret.position,
                    Rotation2d().fromDegrees(0.0),  # self.intake.position,
                )
            else:
                RobotState.periodic(
                    self.drivetrainSubsystem.getRawRotation(),
                    Logger.getTimestamp() / 1e6,
                    self.drivetrainSubsystem.getAngularVelocity(),
                    self.drivetrainSubsystem.getFieldRelativeChassisSpeeds(),
                    self.drivetrainSubsystem.getModulePositions(),
                    Rotation2d().fromDegrees(0.0),  # self.turret.position,
                    Rotation2d().fromDegrees(0.0),  # self.intake.position,
                )


    def autonomousInit(self) -> None:
        self.setDefaultStartpose()
        self.autoOrTestCommand = self.autoChooser.getSelected()
        self.autonomousOrTestCommonInit()
        self.autoHasRun = True


    def teleopInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
        DriverInterface().initiatialize()
        if self.inout is not None:
           CommandScheduler.getInstance().schedule(self.inout.aOperatorRunsInoutCommand())
        # clear existing telemetry trajectory
        if self.drivetrainSubsystem is not None:
            # xyzzy todo
            # we always have a default autonoumous pose?
            # that if auto hasn't run, we set our default poss to the default, or selected autonoumous pose?
            # -Thanks Coach Mike
            self.setDefaultStartpose()
            #self.drivetrainSubsystem.casseroleDrivetrain.poseEst._telemetry.setCurAutoTrajectory(None)
            self.drivetrainSubsystem.setDefaultCommand(
                self.drivetrainSubsystem.arcadeDriveClosedLoop(DriverInterface().getCmd)
            )

    def setDefaultStartpose(self) -> None:
        if self.drivetrainSubsystem is not None:
            if not self.autoHasRun:
                robotStartXIn = 40.0
                robotStartYIn = 80.0
                if onRed():
                    startPose = Pose2d(in2m(kFieldLengthIn - robotStartXIn), in2m(kFieldWidthIn - robotStartYIn),
                                       Rotation2d(deg2Rad(0)))
                    print(f"onRed startPose: {startPose}")
                else:
                    startPose = Pose2d(in2m(robotStartXIn), in2m(robotStartYIn), Rotation2d(deg2Rad(180)))
                    print(f"onBlue startPose: {startPose}")
                self.resetPose(startPose)

    def resetPose(self, pose: Pose2d) -> None:
            self.drivetrainSubsystem.casseroleDrivetrain.poseEst.setKnownPose(pose)
            print(f"resetPose: {pose}")
            if self.visionSubsystem is not None:
                RobotState.resetPose(pose)

    def testInit(self) -> None:
        self.autoOrTestCommand = self.testChooser.getSelected()
        self.autonomousOrTestCommonInit()

    def autonomousOrTestCommonInit(self) -> None:
        DriverInterface().initiatialize()
        CommandScheduler.getInstance().cancelAll()
        if self.autoOrTestCommand is not None:
            CommandScheduler.getInstance().schedule(self.autoOrTestCommand)

    def quietRobotOnExitFromActiveMode(self) -> None:
        if self.autoOrTestCommand is not None:
            self.autoOrTestCommand.cancel()
            self.autoOrTestCommand = None

        if self.inout is not None:
            self.inout.initialize()

