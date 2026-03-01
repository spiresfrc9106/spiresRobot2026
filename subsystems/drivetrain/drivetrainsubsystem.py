from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple

from commands2 import Command, Subsystem, cmd
from wpilib import XboxController

from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.drivetrainPhysical import wrapperedSwerveDriveAzmthEncoder, DrivetrainPhysical
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import SparkBase, SparkSim
from wpilib.simulation import LinearSystemSim_1_1_1, FlywheelSim, RoboRioSim, BatterySim

from wpimath.system.plant import DCMotor, LinearSystemId

from constants import kRobotMode, RobotModes, kRobotUpdatePeriodS
from humanInterface.operatorInterface import OperatorInterface, FlywheelCommand, InOutCommand
from subsystems.common.sysidmotormodule import SysIdMotorModule
from subsystems.drivetrain.drivetrainsubsystemio import DrivetrainSubsystemIO

from subsystems.drivetrain.drivetrainsubsystemioreal import DrivetrainSubsystemIOReal
from subsystems.drivetrain.drivetrainsubsystemiosim import DrivetrainSubsystemIORealSim
from subsystems.common.motormodule import MotorModule
from subsystems.common.motormodulecontroller import NullController, SparkSlewRateLimitedVelocityController
from subsystems.common.motormoduleio import MotorModuleIO
from subsystems.common.motormoduleiowrappered import MotorModuleIOWrappered
from subsystems.common.motormoduleiowrapperedsim import MotorModuleIOWrapperedSim
from subsystems.state.configsubsystem import ConfigSubsystem
from utils.constants import DT_FL_AZMTH_CANID, DT_FL_AZMTH_ENC_PORT, DT_FL_WHEEL_CANID, DT_FR_WHEEL_CANID, \
    DT_FR_AZMTH_CANID, DT_FR_AZMTH_ENC_PORT, DT_BL_AZMTH_CANID, DT_BL_AZMTH_ENC_PORT, DT_BL_WHEEL_CANID, \
    DT_BR_WHEEL_CANID, DT_BR_AZMTH_CANID, DT_BR_AZMTH_ENC_PORT

from utils.units import radPerSec2RPM, rad2Deg
from westwood.util.logtracer import LogTracer
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor



DTDC = ConfigSubsystem().drivetrainDepConstants
HAS_DRIVETRAIN = DTDC["HAS_DRIVETRAIN"]
@autologgable_output
class DrivetrainSubsystem(Subsystem):

    def __init__(
        self,
        io: DrivetrainSubsystemIO,
        motorModulesAndEncoderSets: List[Tuple[str, MotorModuleIO, MotorModuleIO, WrapperedRevThroughBoreEncoder]]
    ) -> None:
        Subsystem.__init__(self)
        self.name = type(self).__name__
        self.setName(self.name)
        self.io = io
        self.inputs = DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs()
        self.casseroleDrivetrain = DrivetrainControl(motorModulesAndEncoderSets)
        self.initialize()

        self.isClosedLoop = True

    def initialize(self):
        self.setDefaultCommand(self.aDoNothingCommand())

    def sysIdMotorModulePreInit(self) -> None:
        self.initialize()
        self.setClosedLoop(False)

    def sysIdMotorModulePostInit(self) -> None:
        self.setClosedLoop(True)


    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        self.casseroleDrivetrain.update()

    def _updateAllCals(self):
        pass


    def _updatePIDGainsAndFeedForward(self) -> None:
        """
        for module in motorsAndEncoderSets:
            module.updatePIDandFF()
        """

    def periodicUpdateClosedLoopOutputs(self) -> None:
        """
        self.groundModule.updateClosedLoopOutput(
            self.groundInPerSToRadPerS(self.inputs.groundTargetIPS))
        self.hopperModule.updateClosedLoopOutput(
            self.hopperInPerSToRadPerS(self.inputs.hopperTargetIPS))
        self.flywheelModule.updateClosedLoopOutput(
            self.flywheelInPerSToRadPerS(self.inputs.flywheelTargetIPS))
        """

    def setClosedLoop(self, closedLoop: bool) -> None:
        self.isClosedLoop = closedLoop

    """
    def makeCommandFeedForwardCharacterizationGroundMotor(self) -> Command:
        return self.sysIdMotorModule.feedForwardCharacterization(self.groundModule)

    def makeCommandFeedForwardCharacterizationHopperMotor(self) -> Command:
        return self.sysIdMotorModule.feedForwardCharacterization(self.hopperModule)

    def makeCommandFeedForwardCharacterizationFlywheelMotor(self) -> Command:
        return self.sysIdMotorModule.feedForwardCharacterization(self.flywheelModule)

    def makeSysIdCommandGroundMotor(self) -> Command:
        return self.sysIdMotorModule.sysIdRoutine("ground", self.groundModule)

    def makeSysIdCommandHopperMotor(self) -> Command:
        return self.sysIdMotorModule.sysIdRoutine("hopper", self.hopperModule)

    def makeSysIdCommandFlywheelMotor(self) -> Command:
        return self.sysIdMotorModule.sysIdRoutine("flywheel", self.flywheelModule)
    """

    def doNothing(self):
        pass


    def aDoNothingCommand(self) -> Optional[Command]:
        return cmd.sequence(
            cmd.runOnce(lambda: self.initialize(), self),
            cmd.run(lambda: self.doNothing(), self),
        )

    def aOperatorRunsInoutCommand(self) -> Optional[Command]:
        return cmd.sequence(
            cmd.runOnce(lambda: self.initialize(), self),
            cmd.run(lambda: self.useOperatorControls(), self),
        )


def makeNameAndModuleMotorsAndEncoder(
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
        wheelMotorCanID, subsystemName + moduleName + "/wheelMotor", False
    )
    azmthMotor = WrapperedSparkMotor.makeSparkMax(
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
                motorsAndEncoderSets = []
                p = DrivetrainPhysical()

                motorsAndEncoderSets.append(
                    makeNameAndModuleMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "FL", p.WHEEL_MOTOR_WRAPPER, DT_FL_WHEEL_CANID,
                                                      DT_FL_AZMTH_CANID,
                                                      DT_FL_AZMTH_ENC_PORT,
                                                      p.FL_ENCODER_MOUNT_OFFSET_RAD,
                                                      p.FL_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                motorsAndEncoderSets.append(
                    makeNameAndModuleMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "FR", p.WHEEL_MOTOR_WRAPPER, DT_FR_WHEEL_CANID,
                                                      DT_FR_AZMTH_CANID,
                                                      DT_FR_AZMTH_ENC_PORT,
                                                      p.FR_ENCODER_MOUNT_OFFSET_RAD,
                                                      p.FR_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                motorsAndEncoderSets.append(
                    makeNameAndModuleMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "BL", p.WHEEL_MOTOR_WRAPPER, DT_BL_WHEEL_CANID,
                                                      DT_BL_AZMTH_CANID,
                                                      DT_BL_AZMTH_ENC_PORT,
                                                      p.BL_ENCODER_MOUNT_OFFSET_RAD,
                                                      p.BL_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )
                motorsAndEncoderSets.append(
                    makeNameAndModuleMotorsAndEncoder(f"{p.DRIVETRAIN_NAME}/", "BR", p.WHEEL_MOTOR_WRAPPER, DT_BR_WHEEL_CANID,
                                                      DT_BR_AZMTH_CANID,
                                                      DT_BR_AZMTH_ENC_PORT,
                                                      p.BR_ENCODER_MOUNT_OFFSET_RAD,
                                                      p.BR_INVERT_WHEEL_MOTOR, p.INVERT_AZMTH_MOTOR, p.INVERT_AZMTH_ENCODER)
                )


        motorModulesAndEncoderSets = []
        match kRobotMode:
            case RobotModes.REAL:
                io=DrivetrainSubsystemIOReal(name="inoutIO")
                for moduleName, wheelMotor, azmthMotor, azmthEncoder in motorsAndEncoderSets:
                    wheelMotor_io=MotorModuleIOWrappered(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                    azmthMotor_io=MotorModuleIOWrappered(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                    motorModulesAndEncoderSets.append(
                        (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder))
            case RobotModes.SIMULATION:
                    io=DrivetrainSubsystemIORealSim(name="inoutIO")
                    for moduleName, wheelMotor, azmthMotor, azmthEncoder in motorsAndEncoderSets:
                        wheelMotor_io=MotorModuleIOWrapperedSim(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                        azmthMotor_io=MotorModuleIOWrapperedSim(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                        motorModulesAndEncoderSets.append(
                            (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder))
            case RobotModes.REPLAY | _:
                    io=DrivetrainSubsystemIO()
                    for moduleName, wheelMotor, azmthMotor, azmthEncoder in motorsAndEncoderSets:
                        wheelMotor_io=MotorModuleIO(name=f"{p.DRIVETRAIN_NAME}/{moduleName}WheelMotorModuleIO", motor=wheelMotor)
                        azmthMotor_io=MotorModuleIO(name=f"{p.DRIVETRAIN_NAME}/{moduleName}AzmthMotorModuleIO", motor=azmthMotor)
                        motorModulesAndEncoderSets.append(
                            (moduleName, wheelMotor_io, azmthMotor_io, azmthEncoder))
        drivetrain = DrivetrainSubsystem(io=io, motorModulesAndEncoderSets=motorModulesAndEncoderSets)

    return drivetrain