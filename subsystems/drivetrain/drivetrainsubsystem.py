from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from wpilib import XboxController

from drivetrain.drivetrainControl import DrivetrainControl
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

from utils.units import radPerSec2RPM
from westwood.util.logtracer import LogTracer
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor



DTDC = ConfigSubsystem().drivetrainDepConstants
HAS_DRIVETRAIN = DTDC["HAS_DRIVETRAIN"]
@autologgable_output
class DrivetrainSubsystem(Subsystem):

    def __init__(
        self,
        io: DrivetrainSubsystemIO
    ) -> None:
        Subsystem.__init__(self)
        self.name = type(self).__name__
        self.setName(self.name)
        self.io = io
        self.inputs = DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs()
        self.casseroleDrivetrain = DrivetrainControl()
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
        for module in self.modules:
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




def DrivetrainSubsystemFactory() -> DrivetrainSubsystem|None:
    drivetrain: Optional[DrivetrainSubsystem] = None
    if HAS_DRIVETRAIN:
        match kRobotMode:
            case RobotModes.REAL | RobotModes.SIMULATION:

                """
                groundMotorGearBox: Optional[DCMotor] = None
                hopperMotorGearBox: Optional[DCMotor] = None
                flywheelMotorGearBox: Optional[DCMotor] = None
                """

                if kRobotMode == RobotModes.SIMULATION:
                    """
                    groundMotorGearBox = DCMotor.NEO(1)
                    hopperMotorGearBox = DCMotor.NEO(1)
                    flywheelMotorGearBox = DCMotor.neoVortex(1)
                    """

                """
                groundMotor = WrapperedSparkMotor.makeSparkMax(name="groundMotor",
                                                canID=config.drivetrainDepConstants["GROUND_MOTOR_CANID"],
                                                gearBox=groundMotorGearBox)
                groundMotor.setInverted(config.drivetrainDepConstants["GROUND_MOTOR_INVERTED"])
                hopperMotor = WrapperedSparkMotor.makeSparkMax(name="hopperMotor",
                                                canID=config.drivetrainDepConstants["HOPPER_MOTOR_CANID"],
                                                gearBox=hopperMotorGearBox)
                hopperMotor.setInverted(config.drivetrainDepConstants["HOPPER_MOTOR_INVERTED"])
                flywheelMotor = WrapperedSparkMotor.makeSparkFlex(name="flywheelMotor",
                                                   canID=config.drivetrainDepConstants["FLYWHEEL_MOTOR_CANID"],
                                                   gearBox=flywheelMotorGearBox)
                flywheelMotor.setInverted(config.drivetrainDepConstants["FLYWHEEL_MOTOR_INVERTED"])


                
                groundController = SparkSlewRateLimitedVelocityController(
                    cals=drivetrainCals.groundCals,
                    userUnitsToRadPerSec=DrivetrainSubsystem.groundInPerSToRadPerS
                )
                hopperController = SparkSlewRateLimitedVelocityController(
                    cals=drivetrainCals.hopperCals,
                    userUnitsToRadPerSec=DrivetrainSubsystem.hopperInPerSToRadPerS
                )
                flywheelController = SparkSlewRateLimitedVelocityController(
                    cals=drivetrainCals.flywheelCals,
                    userUnitsToRadPerSec=DrivetrainSubsystem.flywheelInPerSToRadPerS
                )
                """

                drivetrainSim = None
                if kRobotMode == RobotModes.SIMULATION:
                    """
                    drivetrainSim = DrivetrainSubsystemSimulation(groundMotor, hopperMotor, flywheelMotor)
                    """
            case _:
                pass

        match kRobotMode:
            case RobotModes.REAL:
                drivetrain = DrivetrainSubsystem(
                    io=DrivetrainSubsystemIOReal(name="drivetrainIO"),
                )

            case RobotModes.SIMULATION:
                drivetrain = DrivetrainSubsystem(
                    io=DrivetrainSubsystemIORealSim(name="drivetrainIO"),
                )
            case RobotModes.REPLAY | _:
                drivetrain = DrivetrainSubsystem(
                    io=DrivetrainSubsystemIO(),
                )

        match kRobotMode:
            case RobotModes.SIMULATION:
                """
                groundMaxFreeSpeedRadps = groundMotorGearBox.freeSpeed
                groundMaxFreeSpeedRPM = radPerSec2RPM(groundMaxFreeSpeedRadps)
                groundMaxFreeSpeedIPS =  drivetrain.groundRadPerSToInPerS(groundMaxFreeSpeedRadps)
                print(f"groundMaxFreeSpeedRPM={groundMaxFreeSpeedRPM:.0f} groundMaxFreeSpeedIPS={groundMaxFreeSpeedIPS:.0f}"
                      f" calGroundIntakeTargetSpeedIPS={drivetrain.calGroundIntakeTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={drivetrain.calGroundIntakeTargetSpeedIPS/groundMaxFreeSpeedIPS:.2f}")
                hopperMaxFreeSpeedRadps = hopperMotorGearBox.freeSpeed
                hopperMaxFreeSpeedRPM = radPerSec2RPM(hopperMaxFreeSpeedRadps)
                hopperMaxFreeSpeedIPS =  drivetrain.hopperRadPerSToInPerS(hopperMaxFreeSpeedRadps)
                print(f"hopperMaxFreeSpeedRPM={hopperMaxFreeSpeedRPM:.0f} hopperMaxFreeSpeedIPS={hopperMaxFreeSpeedIPS:.0f}"
                      f" calHopperIntakeTargetSpeedIPS={drivetrain.calHopperIntakeTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={drivetrain.calHopperIntakeTargetSpeedIPS/hopperMaxFreeSpeedIPS:.2f}")
                flywheelMaxFreeSpeedRadps = flywheelMotorGearBox.freeSpeed
                flywheelMaxFreeSpeedRPM = radPerSec2RPM(flywheelMaxFreeSpeedRadps)
                flywheelMaxFreeSpeedIPS =  drivetrain.flywheelRadPerSToInPerS(flywheelMaxFreeSpeedRadps)
                print(f"flywheelMaxFreeSpeedRPM={flywheelMaxFreeSpeedRPM:.0f} flywheelMaxFreeSpeedIPS={flywheelMaxFreeSpeedIPS:.0f}"
                      f" calFlywheelIntakeTargetSpeedIPS={drivetrain.calFlywheelTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={drivetrain.calFlywheelTargetSpeedIPS/flywheelMaxFreeSpeedIPS:.2f}")
                """
            case RobotModes.REAL|RobotModes.REPLAY | _:
                pass

    return drivetrain