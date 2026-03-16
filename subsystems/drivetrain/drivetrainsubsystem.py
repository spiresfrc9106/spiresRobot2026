from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple, Callable

from commands2 import Command, Subsystem, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import RobotConfig
from wpilib import XboxController, RobotBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition

from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.drivetrainPhysical import wrapperedSwerveDriveAzmthEncoder, DrivetrainPhysical
from pykit.autolog import autologgable_output, autolog_output
from pykit.logger import Logger
from rev import SparkBase, SparkSim
from wpilib.simulation import LinearSystemSim_1_1_1, FlywheelSim, RoboRioSim, BatterySim

from wpimath.system.plant import DCMotor, LinearSystemId

from constants import kRobotMode, RobotModes, kRobotUpdatePeriodS
from humanInterface.operatorInterface import OperatorInterface, FlywheelCommand, InOutCommand
from subsystems.common.encodermodule import EncoderModule
from subsystems.common.encodermoduleio import EncoderModuleIO
from subsystems.common.encodermoduleiowrappered import EncoderModuleIOWrappered
from subsystems.common.encodermoduleiowrapperedsim import EncoderModuleIOWrapperedSim
from subsystems.common.sysidmotormodule import SysIdMotorModule
from subsystems.common.sysidmotormodules import SysIdMotorModules
from subsystems.drivetrain.drivetrainsubsystemio import DrivetrainSubsystemIO

from subsystems.drivetrain.drivetrainsubsystemioreal import DrivetrainSubsystemIOReal
from subsystems.drivetrain.drivetrainsubsystemiosim import DrivetrainSubsystemIORealSim
from subsystems.common.motormodule import MotorModule
from subsystems.common.motormodulecontroller import NullController, SparkSlewRateLimitedVelocityController
from subsystems.common.motormoduleio import MotorModuleIO
from subsystems.common.motormoduleiowrappered import MotorModuleIOWrappered
from subsystems.common.motormoduleiowrapperedsim import MotorModuleIOWrapperedSim
from subsystems.state.configsubsystem import ConfigSubsystem
from utils.allianceTransformUtils import onRed
from utils.constants import DT_FL_AZMTH_CANID, DT_FL_AZMTH_ENC_PORT, DT_FL_WHEEL_CANID, DT_FR_WHEEL_CANID, \
    DT_FR_AZMTH_CANID, DT_FR_AZMTH_ENC_PORT, DT_BL_AZMTH_CANID, DT_BL_AZMTH_ENC_PORT, DT_BL_WHEEL_CANID, \
    DT_BR_WHEEL_CANID, DT_BR_AZMTH_CANID, DT_BR_AZMTH_ENC_PORT

from utils.units import radPerSec2RPM, rad2Deg
from util.logtracer import LogTracer
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor



DTDC = ConfigSubsystem().drivetrainDepConstants
HAS_DRIVETRAIN = DTDC["HAS_DRIVETRAIN"]
@autologgable_output
class DrivetrainSubsystem(Subsystem):

    def __init__(
        self,
        io: DrivetrainSubsystemIO,
        motorModuleIOsAndEncoderIOSets: List[Tuple[str, MotorModuleIO, MotorModuleIO, EncoderModuleIO]]
    ) -> None:
        Subsystem.__init__(self)
        self.name = type(self).__name__
        self.setName(self.name)
        self.io = io
        self.inputs = DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs()
        self.wheelModules = []
        self.azmthModules = []
        self.azmthEncoderModules = []
        self.motorAndEncoderModules = []
        for moduleName, wheelModuleIO, azmthModuleIO, azmthEncoderIO in motorModuleIOsAndEncoderIOSets:
            wheelMotorModule = MotorModule(name=f"wheelMotor{moduleName}Module", io=wheelModuleIO, controller=NullController)
            azmuthMotorModule = MotorModule(name=f"azmthMotor{moduleName}Module", io=azmthModuleIO, controller=NullController)
            azmuthEncoderModule = EncoderModule(name=f"azmthEncoder{moduleName}Module", io=azmthEncoderIO)
            self.wheelModules.append(wheelMotorModule)
            self.azmthModules.append(azmuthMotorModule)
            self.azmthEncoderModules.append(azmuthEncoderModule)
            self.motorAndEncoderModules.append((moduleName, wheelMotorModule, azmuthMotorModule, azmuthEncoderModule))

        self.casseroleDrivetrain = DrivetrainControl(self.motorAndEncoderModules)
        self.initialize()
        self.setDefaultCommand(self.aDoNothingCommand())

        self.sysIdWheelMotors = SysIdMotorModules(
            XboxController(1),
            self.sysIdMotorModulePreInit,
            self.sysIdMotorModulePostInit,
            self,
        )

    def initialize(self):
        self.casseroleDrivetrain.setManualCmd(self.casseroleDrivetrain.DO_NOTHING_CMD)

    def getRawRotation(self) -> Rotation2d:
        return self.casseroleDrivetrain.getRawRotation()

    @autolog_output(key="Robot/velocity")
    def getAngularVelocity(self) -> float:
        """radians per sec"""
        #TODO to get playback to work this needs to become an input/output module.
        #if RobotBase.isSimulation() and not Logger.isReplay():
        #    # todo value: ChassisSpeeds = self.simVelocityGetter.get()
        #    #value: ChassisSpeeds = self.simVelocityGetter.get()
        #    #return value.omega
        #    #return self.casseroleDrivetrain.gyro.getRate()
        return self.casseroleDrivetrain.gyro.getRate()


    def getFieldRelativeChassisSpeeds(self)->ChassisSpeeds:
        return self.casseroleDrivetrain.getFieldRelativeChassisSpeeds()

    def getRobotRelativeChassisSpeeds(self)->ChassisSpeeds:
        return self.casseroleDrivetrain.getRobotRelativeChassisSpeeds()

    def getModulePositions(self)->Tuple[SwerveModulePosition,SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]:
        return self.casseroleDrivetrain.getModulePositions()

    def sysIdMotorModulePreInit(self) -> None:
        self.initialize()
        self.casseroleDrivetrain.wheelMotorsAreExternallyControlled()
        cmd = DrivetrainCommand(velX=1.0, velY=0.0, velT=0.0, robotRelative=True)
        self.casseroleDrivetrain.setManualCmd(cmd)

    def sysIdMotorModulePostInit(self) -> None:
        self.casseroleDrivetrain.wheelMotorsAreClosedLoop()

    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        for module in self.wheelModules:
            module.periodic()
        for module in self.azmthModules:
            module.periodic()
        for module in self.azmthEncoderModules:
            module.periodic()
        # We start LogTracing here because the above modules do their own periodic logging.
        LogTracer.resetOuter("drivetrainSubsystem periodic")
        self.io.updateInputs(self.inputs)  # update state of the ionout subsystem
        Logger.processInputs("drivetrain", self.inputs)
        LogTracer.record("UpdateInputs")
        self.casseroleDrivetrain.update()
        LogTracer.record("casseroleDrivetrain.update")


    def _updateAllCals(self):
        pass

    def makeSysIdCommandWheelMotors(self) -> Command:
        return self.sysIdWheelMotors.sysIdRoutine("wheel", self.wheelModules)

    def drivePathPlanned(self, chassisSpeeds: ChassisSpeeds, _feedForward):
        self.casseroleDrivetrain.setManualCmdViaChassisSpeeds(chassisSpeeds)

    def doNothing(self):
        pass


    def aDoNothingCommand(self) -> Optional[Command]:
        return cmd.sequence(
            cmd.runOnce(lambda: self.initialize(), self),
            cmd.run(lambda: self.doNothing(), self),
        )

    def arcadeDriveClosedLoop(self, getCmd: Callable[[], DrivetrainCommand]) -> Command:
        def run():
            cmd = getCmd()
            self.casseroleDrivetrain.setManualCmd(cmd)
        return cmd.run(run, self).withName("Arcade Drive Closed Loop")


def makeNameAndWrapperedMotorsAndEncoder(
        subsystemName: str,
        moduleName: str,
        wheelMotorWrapper: WrapperedMotorSuper,
        wheelMotorCanID: int,
        azmthMotorCanID: int,
        azmthEncoderPortIdx: int,
        azmthOffset: float,
        invertWheelMotor: bool,
        invertAzmthMotor: bool,
        invertAzmthEncoder: bool) -> Tuple[str, MotorModuleIO, MotorModuleIO, WrapperedRevThroughBoreEncoder]:
    """
    Make motors for one swerve drive module.

    Convention Reminders:
    The **module** refers to the whole assembly, including two motors, their built-in sensors, 
    the azimuth angle sensor, the hardware, everything.

    The **azimuth** is the motor, sensor, and mechanism to point the wheel in a specific direction.

    Positive azimuth rotation is counter-clockwise when viewed top-down. By the right hand rule, this is
    rotation in the positive-Z direction. Zero degrees is toward the front of the robot

    The **wheel** is the motor and mechanism to apply a force in that direction.

    Positive wheel rotation causes the robot to move forward if the azimuth is pointed forward.

    Uses WPILib convention for names:
    1) "State" refers to the speed of the wheel, plus the position of the azimuth
    2) "Position" refers to the position of the wheel, plus the position of the azimuth



        Make motors for one swerve drive module

        Args:
            moduleName (str): Name Prefix for the module (IE, "FL", or "BR"). For logging purposes mostly
            wheelMotorCanID (int): CAN Id for the wheel motor for this module
            azmthMotorCanID (int): CAN Id for the azimuth motor for this module
            azmthEncoderPortIdx (int): RIO Port for the azimuth absolute encoder for this module
            azmthOffset (float): Mounting offset of the azimuth encoder in Radians.
            invertWheelMotor (bool): Inverts the drive direction of the wheel motor
            invertAzmthMotor (bool): Inverts the steering direction of the azimuth motor
            invertAzmthEncoder (bool): Inverts the direction of the steering azimuth encoder
    """

    print(f"{moduleName} azmthOffset={rad2Deg(azmthOffset):7.1f} deg")
    wheelMotor = wheelMotorWrapper(
        wheelMotorCanID, subsystemName + moduleName + "/wheelMotor", brakeMode=False, currentLimitA=60
    )
    azmthMotor = WrapperedSparkMax(
        azmthMotorCanID, subsystemName + moduleName + "/azmthMotor", True
    )

    # Note the azimuth encoder inversion should be fixed, based on the physical design of the encoder itself,
    # plus the swerve module physical construction. It might need to be tweaked here though if we change 
    # module brands or sensor brands.
    azmthEnc = wrapperedSwerveDriveAzmthEncoder(
        azmthEncoderPortIdx, subsystemName + moduleName + "/azmthEnc", azmthOffset, invertAzmthEncoder
    )

    wheelMotor.setInverted(invertWheelMotor)
    azmthMotor.setInverted(invertAzmthMotor)
    return (moduleName, wheelMotor, azmthMotor, azmthEnc)


def DrivetrainSubsystemFactory() -> DrivetrainSubsystem|None:
    drivetrain: Optional[DrivetrainSubsystem] = None
    if HAS_DRIVETRAIN:
        match kRobotMode:
            case RobotModes.REAL | RobotModes.SIMULATION:
                wrapperedMotorsAndEncoderSets = []
                p = DrivetrainPhysical()

                wrapperedMotorsAndEncoderSets.append(
                    makeNameAndWrapperedMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "FL", p.WHEEL_MOTOR_WRAPPER, DT_FL_WHEEL_CANID,
                                                         DT_FL_AZMTH_CANID,
                                                         DT_FL_AZMTH_ENC_PORT,
                                                         p.FL_ENCODER_MOUNT_OFFSET_RAD,
                                                         p.FL_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                wrapperedMotorsAndEncoderSets.append(
                    makeNameAndWrapperedMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "FR", p.WHEEL_MOTOR_WRAPPER, DT_FR_WHEEL_CANID,
                                                         DT_FR_AZMTH_CANID,
                                                         DT_FR_AZMTH_ENC_PORT,
                                                         p.FR_ENCODER_MOUNT_OFFSET_RAD,
                                                         p.FR_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                wrapperedMotorsAndEncoderSets.append(
                    makeNameAndWrapperedMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "BL", p.WHEEL_MOTOR_WRAPPER, DT_BL_WHEEL_CANID,
                                                         DT_BL_AZMTH_CANID,
                                                         DT_BL_AZMTH_ENC_PORT,
                                                         p.BL_ENCODER_MOUNT_OFFSET_RAD,
                                                         p.BL_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                wrapperedMotorsAndEncoderSets.append(
                    makeNameAndWrapperedMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "BR", p.WHEEL_MOTOR_WRAPPER, DT_BR_WHEEL_CANID,
                                                         DT_BR_AZMTH_CANID,
                                                         DT_BR_AZMTH_ENC_PORT,
                                                         p.BR_ENCODER_MOUNT_OFFSET_RAD,
                                                         p.BR_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )


        motorModuleIOsAndEncoderIOSets = []
        match kRobotMode:
            case RobotModes.REAL:
                io=DrivetrainSubsystemIOReal(name="drivetrainIO")
                for moduleName, wheelMotor, azmthMotor, azmthEncoder in wrapperedMotorsAndEncoderSets:
                    wheelMotor_io=MotorModuleIOWrappered(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                    azmthMotor_io=MotorModuleIOWrappered(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                    azmthEncoder_io=EncoderModuleIOWrappered(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthEncoderModuleIO", encoder=azmthEncoder)
                    motorModuleIOsAndEncoderIOSets.append(
                        (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder_io))
            case RobotModes.SIMULATION:
                    io=DrivetrainSubsystemIORealSim(name="drivetrainIO")
                    for moduleName, wheelMotor, azmthMotor, azmthEncoder in wrapperedMotorsAndEncoderSets:
                        wheelMotor_io=MotorModuleIOWrapperedSim(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                        azmthMotor_io=MotorModuleIOWrapperedSim(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                        azmthEncoder_io = EncoderModuleIOWrapperedSim(
                            name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthEncoderModuleIO", encoder=azmthEncoder)
                        motorModuleIOsAndEncoderIOSets.append(
                            (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder_io))
            case RobotModes.REPLAY | _:
                    io=DrivetrainSubsystemIO()
                    for moduleName, wheelMotor, azmthMotor, azmthEncoder in wrapperedMotorsAndEncoderSets:
                        wheelMotor_io=MotorModuleIO(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                        azmthMotor_io=MotorModuleIO(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                        azmthEncoder_io = EncoderModuleIO(
                            name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthEncoderModuleIO", encoder=azmthEncoder)
                        motorModuleIOsAndEncoderIOSets.append(
                            (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder_io))
        drivetrain = DrivetrainSubsystem(io=io, motorModuleIOsAndEncoderIOSets=motorModuleIOsAndEncoderIOSets)

    return drivetrain