import os
from typing import Callable, Optional

import commands2
import wpilib
from commands2 import Command, cmd, CommandScheduler
from pathplannerlib.auto import PathPlannerAuto, AutoBuilder, NamedCommands
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d

from constants import kFieldLengthIn, kFieldWidthIn
from constants.field import poseTransformedForAlliance
from drivetrain.drivetrainPhysical import DrivetrainPhysical
from humanInterface.driverInterface import DriverInterface
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from subsystems.drivetrain.drivetrainsubsystem import (
    DrivetrainSubsystemFactory,
    DrivetrainSubsystem,
)
from subsystems.intakeOuttake.inoutsubsystem import (
    InOutSubsystem,
    inoutSubsystemFactory,
)

from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystemFactory
from subsystems.vision.visionsubsystem import VisionSubsystem, VisionSubsystemFactory
from util.robotposeestimator import VisionObservation
from utils.allianceTransformUtils import onRed
from utils.units import deg2Rad, in2m, m2in


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

    def __init__(self) -> None:
        # The robot's subsystems
        self.autoOrTestCommand: Optional[Command] = None
        self.config = ConfigSubsystem()
        self.robotop = RobotTopSubsystemFactory()
        self.inout: InOutSubsystem | None = inoutSubsystemFactory()
        self.drivetrainSubsystem: DrivetrainSubsystem | None = None
        if ConfigSubsystem().useCasseroleSwerve():
            self.drivetrainSubsystem = DrivetrainSubsystemFactory()
        if self.drivetrainSubsystem is not None:
            from subsystems.state.robottopiosim import RobotTopIOSim

            if isinstance(self.robotop.io, RobotTopIOSim):
                self.robotop.io.setChassisSpeedSupplier(
                    self.drivetrainSubsystem.casseroleDrivetrain.getRobotRelativeChassisSpeeds
                )
        visionConsumers: list[Callable[[VisionObservation], None]] = []
        if self.drivetrainSubsystem is not None:
            visionConsumers.append(
                self.drivetrainSubsystem.casseroleDrivetrain.poseEst.addVisionObservation
            )
        self.visionSubsystem: VisionSubsystem | None = VisionSubsystemFactory(
            visionConsumers
        )
        if self.inout is not None:
            NamedCommands.registerCommand(
                "spinUpFlywheel", self.inout.spinUpFlywheelCommand()
            )
            NamedCommands.registerCommand("shoot", self.inout.shootCommand())
            NamedCommands.registerCommand(
                "spinUpAndShoot", self.inout.spinUpAndShootCommand()
            )
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
        # self.autoTrajsDict = {}
        if self.drivetrainSubsystem is not None:
            pathsPath = os.path.join(
                wpilib.getDeployDirectory(), "pathplanner", "autos"
            )
            for file in os.listdir(pathsPath):
                relevantName = file.split(".")[0]
                print(f"Adding auto {relevantName}")
                if relevantName.startswith("side_"):
                    autonLeft = PathPlannerAuto(relevantName)
                    leftPose = autonLeft._startingPose
                    autonRight = PathPlannerAuto(relevantName, mirror=True)
                    rightPose = autonRight._startingPose
                    name = f"L-{relevantName}"
                    self.autoChooser.addOption(name, autonLeft)
                    name = f"R-{relevantName}"
                    self.autoChooser.addOption(name, autonRight)

                    autonLeft = PathPlannerAuto(relevantName)
                    autonRight = PathPlannerAuto(relevantName, mirror=True)

                    name = f"sh-L-{relevantName}"
                    self.autoChooser.addOption(
                        name,
                        commands2.SequentialCommandGroup(
                            cmd.runOnce(
                                lambda p=leftPose: self.resetPose(  # type: ignore[misc]
                                    poseTransformedForAlliance(p)
                                ),
                                self.drivetrainSubsystem,
                            ),
                            self.inout.spinUpAndShootCommand(),  # type: ignore[union-attr]
                            autonLeft,
                        ),
                    )
                    name = f"sh-R-{relevantName}"
                    self.autoChooser.addOption(
                        name,
                        commands2.SequentialCommandGroup(
                            cmd.runOnce(
                                lambda p=rightPose: self.resetPose(  # type: ignore[misc]
                                    poseTransformedForAlliance(p)
                                ),
                                self.drivetrainSubsystem,
                            ),
                            self.inout.spinUpAndShootCommand(),  # type: ignore[union-attr]
                            autonRight,
                        ),
                    )
                else:
                    auton = PathPlannerAuto(relevantName)
                    cPose = auton._startingPose
                    name = f"C-{relevantName}"
                    self.autoChooser.addOption(name, auton)
                    auton = PathPlannerAuto(relevantName)
                    name = f"sh-C-{relevantName}"
                    self.autoChooser.addOption(
                        name,
                        commands2.SequentialCommandGroup(
                            cmd.runOnce(
                                lambda p=cPose: self.resetPose(  # type: ignore[misc]
                                    poseTransformedForAlliance(p)
                                ),
                                self.drivetrainSubsystem,
                            ),
                            self.inout.spinUpAndShootCommand(),  # type: ignore[union-attr]
                            auton,
                        ),
                    )

        self.autoChooser.setDefaultOption("Do Nothing Once", cmd.none())

        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        if self.drivetrainSubsystem is not None:
            self.testChooser.addOption(
                "drivetrain SysId wheel",
                self.drivetrainSubsystem.makeSysIdCommandWheelMotors(),
            )

        if self.inout is not None:
            self.testChooser.addOption(
                "inout FF ground",
                self.inout.makeCommandFeedForwardCharacterizationGroundMotor(),
            )
            self.testChooser.addOption(
                "inout FF hopper",
                self.inout.makeCommandFeedForwardCharacterizationHopperMotor(),
            )
            self.testChooser.addOption(
                "inout FF flywheel",
                self.inout.makeCommandFeedForwardCharacterizationFlywheelMotor(),
            )
            self.testChooser.addOption(
                "inout SysId ground", self.inout.makeSysIdCommandGroundMotor()
            )
            self.testChooser.addOption(
                "inout SysId hopper", self.inout.makeSysIdCommandHopperMotor()
            )
            self.testChooser.addOption(
                "inout SysId flywheel", self.inout.makeSysIdCommandFlywheelMotor()
            )

        self.testChooser.setDefaultOption("Do Nothing Once", cmd.none())

    def robotPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        self.setDefaultStartpose()
        self.autoOrTestCommand = self.autoChooser.getSelected()
        self.autonomousOrTestCommonInit()
        self.autoHasRun = True

    def teleopInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
        DriverInterface().initiatialize()
        if self.inout is not None:
            CommandScheduler.getInstance().schedule(
                self.inout.aOperatorRunsInoutCommand()
            )
        # clear existing telemetry trajectory
        if self.drivetrainSubsystem is not None:
            # xyzzy todo
            # we always have a default autonoumous pose?
            # that if auto hasn't run, we set our default poss to the default, or selected autonoumous pose?
            # -Thanks Coach Mike
            self.setDefaultStartpose()
            # self.drivetrainSubsystem.casseroleDrivetrain.poseEst._telemetry.setCurAutoTrajectory(None)
            self.drivetrainSubsystem.setDefaultCommand(
                self.drivetrainSubsystem.arcadeDriveClosedLoop(DriverInterface().getCmd)
            )

    def setDefaultStartpose(self) -> None:
        if self.drivetrainSubsystem is not None:
            if not self.autoHasRun:
                robotStartXIn = m2in(2.5)
                robotStartYIn = kFieldWidthIn / 2
                if onRed():
                    startPose = Pose2d(
                        in2m(kFieldLengthIn - robotStartXIn),
                        in2m(kFieldWidthIn - robotStartYIn),
                        Rotation2d(deg2Rad(0)),
                    )
                    print(f"onRed startPose: {startPose}")
                else:
                    startPose = Pose2d(
                        in2m(robotStartXIn),
                        in2m(robotStartYIn),
                        Rotation2d(deg2Rad(180)),
                    )
                    print(f"onBlue startPose: {startPose}")
                self.resetPose(startPose)

    def resetPose(self, pose: Pose2d) -> None:
        print(f"resetPose: {pose}")
        if self.drivetrainSubsystem is not None:
            self.drivetrainSubsystem.casseroleDrivetrain.setKnownPose(pose)
        else:
            self.robotop.resetRobotPose(pose)

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
