import os

from pykit.logger import Logger
from pykit.alertlogger import AlertLogger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser

import wpilib
from wpimath.geometry import Pose2d, Rotation2d
import commands2
import commands2.cmd as Commands
from pathplannerlib.auto import PathPlannerAuto

from subsystems.state.robottopsubsystem import RobotTopSubsystem
from subsystems.state.configsubsystem import ConfigSubsystem

from westwood.commands.drive.fieldrelativedrive import FieldRelativeDrive
from westwood.commands.drive.anglealign import AngleAlignDrive
from westwood.commands.defensestate import DefenseState

from westwood.commands.resetgyro import ResetGyro
from westwood.robotmechanism import RobotMechanism
from westwood.robotstate import RobotState
from westwood.subsystems.drive.driveiopigeon import DriveIOPigeon
from westwood.subsystems.drive.drivesubsystem import DriveSubsystem
from westwood.subsystems.drive.swervemoduleio  import SwerveModuleConfigParams, SwerveModuleIO
from westwood.subsystems.drive.swervemoduleiosim import SwerveModuleIOSim
from westwood.subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
#from westwood.subsystems import TurretSubsystem
#from westwood.subsystems import TurretSubsystemIO
#from westwood.subsystems.turret.turretsubsystemiosim import TurretSubsystemIOSim
#from westwood.subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from westwood.subsystems.vision.visionio import VisionSubsystemIO
from westwood.subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from westwood.subsystems.vision.visioniophotonsim import VisionSubsystemIOPhotonSim
from westwood.subsystems.vision.visionsubsystem import VisionSubsystem
from westwood.subsystems.drive.driveio import DriveIO

from westwood.operatorinterface import OperatorInterface

from westwood.constants.vision import (
    kRobotToCamera1Transform,
    kRobotToCamera2Transform,
)
from westwood.constants.field import kAutoDuration
from westwood.constants.drive import (
    kTurboSpeedMultiplier,
    kNormalSpeedMultiplier,
    kFrontLeftModuleName,
    kFrontLeftDriveMotorId,
    kFrontLeftDriveInverted,
    kFrontLeftSteerMotorId,
    kFrontLeftSteerInverted,
    kFrontLeftSteerEncoderId,
    kFrontLeftAbsoluteEncoderOffset,
    kCANivoreName,
    kFrontRightModuleName,
    kFrontRightDriveMotorId,
    kFrontRightDriveInverted,
    kFrontRightSteerMotorId,
    kFrontRightSteerInverted,
    kFrontRightSteerEncoderId,
    kFrontRightAbsoluteEncoderOffset,
    kBackLeftModuleName,
    kBackLeftDriveMotorId,
    kBackLeftDriveInverted,
    kBackLeftSteerMotorId,
    kBackLeftSteerInverted,
    kBackLeftSteerEncoderId,
    kBackLeftAbsoluteEncoderOffset,
    kBackRightModuleName,
    kBackRightDriveMotorId,
    kBackRightDriveInverted,
    kBackRightSteerMotorId,
    kBackRightSteerInverted,
    kBackRightSteerEncoderId,
    kBackRightAbsoluteEncoderOffset,
    kDriveGearingRatio,
    kSteerGearingRatioMk5i,
    kSteerGearingRatioMk5n,
)
from constants import RobotModes, kRobotMode
from westwood.util.fliputil import FlipUtil


class WestwoodRobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, xyzzy, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive = None
        self.vision = None
        match kRobotMode:
            case RobotModes.REAL:
                if ConfigSubsystem().useWestwoodSwerve():
                    self.drive = DriveSubsystem(
                        DriveIOPigeon(),
                        (
                            SwerveModuleIOCTRE(
                                kFrontLeftModuleName,
                                SwerveModuleConfigParams(
                                    kFrontLeftDriveMotorId,
                                    kFrontLeftDriveInverted,
                                    kDriveGearingRatio,
                                    kFrontLeftSteerMotorId,
                                    kFrontLeftSteerInverted,
                                    kSteerGearingRatioMk5i,
                                    kFrontLeftSteerEncoderId,
                                    kFrontLeftAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOCTRE(
                                kFrontRightModuleName,
                                SwerveModuleConfigParams(
                                    kFrontRightDriveMotorId,
                                    kFrontRightDriveInverted,
                                    kDriveGearingRatio,
                                    kFrontRightSteerMotorId,
                                    kFrontRightSteerInverted,
                                    kSteerGearingRatioMk5i,
                                    kFrontRightSteerEncoderId,
                                    kFrontRightAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOCTRE(
                                kBackLeftModuleName,
                                SwerveModuleConfigParams(
                                    kBackLeftDriveMotorId,
                                    kBackLeftDriveInverted,
                                    kDriveGearingRatio,
                                    kBackLeftSteerMotorId,
                                    kBackLeftSteerInverted,
                                    kSteerGearingRatioMk5n,
                                    kBackLeftSteerEncoderId,
                                    kBackLeftAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOCTRE(
                                kBackRightModuleName,
                                SwerveModuleConfigParams(
                                    kBackRightDriveMotorId,
                                    kBackRightDriveInverted,
                                    kDriveGearingRatio,
                                    kBackRightSteerMotorId,
                                    kBackRightSteerInverted,
                                    kSteerGearingRatioMk5n,
                                    kBackRightSteerEncoderId,
                                    kBackRightAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                        ),
                    )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    None, #RobotState.addTurretedVisionMeasurement,
                    [
                        VisionSubsystemIOLimelight(
                            "limelight-br",
                            kRobotToCamera1Transform,
                            RobotState.getRotation,
                        ),
                        VisionSubsystemIOLimelight(
                            "limelight-fl",
                            kRobotToCamera2Transform,
                            RobotState.getRotation,
                        ),
                    ],
                )
                #self.turrent = TurretSubsystem(TurretSubsystemIOTalon())

            case RobotModes.SIMULATION:
                if ConfigSubsystem().useWestwoodSwerve:
                    self.drive = DriveSubsystem(
                        DriveIO(),
                        (
                            SwerveModuleIOSim(
                                kFrontLeftModuleName,
                                SwerveModuleConfigParams(
                                    kFrontLeftDriveMotorId,
                                    kFrontLeftDriveInverted,
                                    kDriveGearingRatio,
                                    kFrontLeftSteerMotorId,
                                    kFrontLeftSteerInverted,
                                    kSteerGearingRatioMk5i,
                                    kFrontLeftSteerEncoderId,
                                    kFrontLeftAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOSim(
                                kFrontRightModuleName,
                                SwerveModuleConfigParams(
                                    kFrontRightDriveMotorId,
                                    kFrontRightDriveInverted,
                                    kDriveGearingRatio,
                                    kFrontRightSteerMotorId,
                                    kFrontRightSteerInverted,
                                    kSteerGearingRatioMk5i,
                                    kFrontRightSteerEncoderId,
                                    kFrontRightAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOSim(
                                kBackLeftModuleName,
                                SwerveModuleConfigParams(
                                    kBackLeftDriveMotorId,
                                    kBackLeftDriveInverted,
                                    kDriveGearingRatio,
                                    kBackLeftSteerMotorId,
                                    kBackLeftSteerInverted,
                                    kSteerGearingRatioMk5n,
                                    kBackLeftSteerEncoderId,
                                    kBackLeftAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                            SwerveModuleIOSim(
                                kBackRightModuleName,
                                SwerveModuleConfigParams(
                                    kBackRightDriveMotorId,
                                    kBackRightDriveInverted,
                                    kDriveGearingRatio,
                                    kBackRightSteerMotorId,
                                    kBackRightSteerInverted,
                                    kSteerGearingRatioMk5n,
                                    kBackRightSteerEncoderId,
                                    kBackRightAbsoluteEncoderOffset,
                                    kCANivoreName,
                                ),
                            ),
                        ),
                    )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    None, #RobotState.addTurretedVisionMeasurement,
                    [
                        VisionSubsystemIOPhotonSim(
                            "camera-br",
                            kRobotToCamera1Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                        VisionSubsystemIOPhotonSim(
                            "camera-fl",
                            kRobotToCamera2Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                    ],
                )
                #self.turrent = TurretSubsystem(TurretSubsystemIOSim())

            case _:
                if ConfigSubsystem().useWestwoodSwerve():
                    self.drive = DriveSubsystem(
                        DriveIO(),
                        (
                            SwerveModuleIO("Front Left"),
                            SwerveModuleIO("Front Right"),
                            SwerveModuleIO("Back Left"),
                            SwerveModuleIO("Back Right"),
                        ),
                    )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    None, #RobotState.addTurretedVisionMeasurement,
                    [VisionSubsystemIO(), VisionSubsystemIO()],
                )
                #self.turrent = TurretSubsystem(TurretSubsystemIO())

        # Alerts
        AlertLogger.registerGroup("Alerts")
        self.driverDisconnected = wpilib.Alert(
            "Driver controller disconnected (port 0)", wpilib.Alert.AlertType.kWarning
        )
        self.operatorDisconnected = wpilib.Alert(
            "Operator controller disconnected (port 1)", wpilib.Alert.AlertType.kWarning
        )
        self.deadInTheWaterAlert = wpilib.Alert(
            "No auto selected!!!", wpilib.Alert.AlertType.kWarning
        )

        self.shiftActiveAlert = wpilib.Alert(
            "SHIFT ACTIVE!", wpilib.Alert.AlertType.kInfo
        )
        self.shiftActiveAlert.set(True)
        # Initialize as active at startup;
        # this initial value may be updated later based on the actual shift state

        # Autonomous routines

        self.nothingAuto = commands2.WaitCommand(kAutoDuration)

        # Chooser
        self.chooser: LoggedDashboardChooser[commands2.Command] = (
            LoggedDashboardChooser("Autonomous")
        )
        #self.chooser.addOption("Turret SysID", self.turret.sysIdRoutine(self.turret))

        if self.drive is not None:
            pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
            for file in os.listdir(pathsPath):
                relevantName = file.split(".")[0]
                auton = PathPlannerAuto(relevantName)
                self.chooser.addOption(relevantName, auton)

        self.chooser.setDefaultOption("Do Nothing Auto", self.nothingAuto)

        def changeStart(newAuto: commands2.Command):
            if isinstance(newAuto, PathPlannerAuto):
                # pylint: disable-next=protected-access
                startingLocation = FlipUtil.fieldPose(newAuto._startingPose)
                RobotState.setAutonomousStartingLogation(startingLocation)

        self.chooser.onChange(changeStart)

        # Put the chooser on the dashboard
        self.configureButtonBindings()

        if self.drive is not None:
            self.drive.setDefaultCommand(
                FieldRelativeDrive(
                    self.drive,
                    lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                    * kTurboSpeedMultiplier,
                    lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                    * kTurboSpeedMultiplier,
                    OperatorInterface.Drive.ChassisControls.Rotation.x,
                )
            )
        #self.turret.setDefaultCommand(turretcommands.trackedTurret(self.turret))

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def robotPeriodic(self) -> None:
        if self.drive is not None:
            RobotState.periodic(
                self.drive.getRawRotation(),
                RobotTopSubsystem().getFPGATimeS(),
                self.drive.getAngularVelocity(),
                self.drive.getFieldRelativeSpeeds(),
                self.drive.getModulePositions(),
                Rotation2d(
                    RobotTopSubsystem().getFPGATimeS() / 20
                ),  # Simulated turret rotation, just go spin
            )
        self.updateAlerts()
        Logger.recordOutput("Component Poses", RobotMechanism.getPoses())

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        if self.drive is not None:
            OperatorInterface.Drive.align_angle().whileTrue(
                AngleAlignDrive(
                    self.drive,
                    lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                    * kNormalSpeedMultiplier,
                    lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                    * kNormalSpeedMultiplier,
                ).repeatedly()
            )

            OperatorInterface.Drive.reset_gyro().onTrue(
                ResetGyro(self.drive, Pose2d(0, 0, 0)).andThen(
                    OperatorInterface.rumbleControllers().withTimeout(0.5)
                )
            )

            OperatorInterface.Drive.defense_state().whileTrue(DefenseState(self.drive))

        RobotState.shiftTrigger().onTrue(
            Commands.runOnce(lambda: self.shiftActiveAlert.set(True))
        ).onFalse(Commands.runOnce(lambda: self.shiftActiveAlert.set(False)))

    def updateAlerts(self):
        self.driverDisconnected.set(not wpilib.DriverStation.isJoystickConnected(0))
        self.operatorDisconnected.set(not wpilib.DriverStation.isJoystickConnected(1))
        self.deadInTheWaterAlert.set(self.chooser.getSelected() == self.nothingAuto)

    def getAutonomousCommand(self) -> commands2.Command:
        if self.drive is None:
            return self.nothingAuto
        selected = self.chooser.getSelected()
        if selected is None:
            return self.nothingAuto
        assert isinstance(selected, commands2.Command)
        return selected
