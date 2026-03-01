from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from wpilib import XboxController

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import SparkBase, SparkSim
from wpilib.simulation import LinearSystemSim_1_1_1, FlywheelSim, RoboRioSim, BatterySim

from wpimath.system.plant import DCMotor, LinearSystemId

from constants import kRobotMode, RobotModes, kRobotUpdatePeriodS
from humanInterface.operatorInterface import OperatorInterface, FlywheelCommand, InOutCommand
from subsystems.common.sysidmotormodule import SysIdMotorModule
from subsystems.intakeOuttake.inoutcalset import InOutCalSet
from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO

from subsystems.intakeOuttake.inoutsubsystemioreal import InOutSubsystemIOReal
from subsystems.intakeOuttake.inoutsubsystemiosim import InOutSubsystemIORealSim
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

class FlywheelState(Enum):
    kOff = 0
    kSpinningUp = 1


class InOutState(Enum):
    kOff = 0
    kIntaking = 1
    kOutaking = 2
    kShooting = 3

IODC = ConfigSubsystem().inoutDepConstants
HAS_INOUT = IODC["HAS_INOUT"]

@autologgable_output
class InOutSubsystem(Subsystem):
    GROUND_GEAR_REDUCTION = IODC["GROUND_GEAR_REDUCTION"] if HAS_INOUT else 1.0
    GROUND_WHEEL_DIAMETER_INCHES = IODC["GROUND_WHEEL_DIAMETER_INCHES"] if HAS_INOUT else 1.0
    GROUND_WHEEL_RADIUS_INCHES = GROUND_WHEEL_DIAMETER_INCHES / 2.0 if HAS_INOUT else 1.0
    HOPPER_GEAR_REDUCTION = IODC["HOPPER_GEAR_REDUCTION"] if HAS_INOUT else 1.0
    HOPPER_WHEEL_DIAMETER_INCHES = IODC["HOPPER_WHEEL_DIAMETER_INCHES"] if HAS_INOUT else 1.0
    HOPPER_WHEEL_RADIUS_INCHES = HOPPER_WHEEL_DIAMETER_INCHES / 2.0 if HAS_INOUT else 1.0
    FLYWHEEL_GEAR_REDUCTION = IODC["FLYWHEEL_GEAR_REDUCTION"] if HAS_INOUT else 1.0
    FLYWHEEL_WHEEL_DIAMETER_INCHES = IODC["FLYWHEEL_WHEEL_DIAMETER_INCHES"] if HAS_INOUT else 1.0
    FLYWHEEL_WHEEL_RADIUS_INCHES = FLYWHEEL_WHEEL_DIAMETER_INCHES / 2.0 if HAS_INOUT else 1.0

    def __init__(
        self,
        io: InOutSubsystemIO,
        groundModule_io: MotorModuleIO,
        hopperModule_io: MotorModuleIO,
        flywheelModule_io: MotorModuleIO,
        groundModule_controller,
        hopperModule_controller,
        flywheelModule_controller,
        simulation: "InOutSubsystemSimulation|None" = None
    ) -> None:
        Subsystem.__init__(self)
        self.name = type(self).__name__
        self.setName(self.name)
        self.io = io
        self.inputs = InOutSubsystemIO.InOutSubsystemIOInputs()
        self.groundModule = MotorModule(name="groundModule", io=groundModule_io, controller=groundModule_controller)
        self.hopperModule = MotorModule(name="hopperModule", io=hopperModule_io, controller=hopperModule_controller)
        self.flywheelModule = MotorModule(name="flywheelModule", io=flywheelModule_io, controller=flywheelModule_controller)
        self.modules = (self.groundModule, self.hopperModule, self.flywheelModule)
        self.simulation = simulation

        self.state: InOutState = InOutState.kOff
        self.flywheelState: FlywheelState = FlywheelState.kOff

        self.cals = InOutCalSet()

        self._updateAllCals()

        self.sysIdMotorModule = SysIdMotorModule(
            XboxController(1),
            self.sysIdMotorModulePreInit,
            self.sysIdMotorModulePostInit,
            self,
        )

        self.oInt = OperatorInterface()

        self.initialize()

        self.isClosedLoop = True

    def initialize(self):
        self.setState(InOutState.kOff)
        self.setFlywheelState(FlywheelState.kOff)
        for module in self.modules:
            module.reset()
        self.setDefaultCommand(self.aDoNothingCommand())

    def sysIdMotorModulePreInit(self) -> None:
        self.initialize()
        self.setClosedLoop(False)

    def sysIdMotorModulePostInit(self) -> None:
        self.setClosedLoop(True)


    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        for module in self.modules:
            module.periodic()
        # We start LogTracing here because the above modules do their own periodic logging.
        LogTracer.resetOuter("inoutSubsystem periodic")
        LogTracer.record("ModulesPeriodic")
        self.inputs.groundIPS = self.groundRadPerSToInPerS(self.groundModule.inputs.velRadps)
        self.inputs.hopperIPS = self.hopperRadPerSToInPerS(self.hopperModule.inputs.velRadps)
        self.inputs.flywheelIPS = self.flywheelRadPerSToInPerS(self.flywheelModule.inputs.velRadps)
        self.io.updateInputs(self.inputs)  # update state of the ionout subsystem
        Logger.processInputs("inout", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.cals.hasChanged():
            self._updateAllCals()

        if self.isClosedLoop:
            self.periodicUpdateClosedLoopTargetSpeeds()
            self.periodicUpdateClosedLoopOutputs()

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput(f"{self.name}/ClosedLoop", self.isClosedLoop)
        if self.simulation:
            self.simulation.periodic()
            LogTracer.record("SimulationPeriodic")
        LogTracer.recordTotal()

    def _updateAllCals(self):
        self._updatePIDGainsAndFeedForward()

        self.calGroundIntakeTargetSpeedIPS = self.cals.groundIntakeSpeedIPS.get()
        self.calGroundOuttakeTargetSpeedIPS = self.cals.groundOuttakeSpeedIPS.get()
        self.calGroundShootTargetSpeedIPS = self.cals.groundShootSpeedIPS.get()

        self.calHopperIntakeTargetSpeedIPS = self.cals.hopperIntakeSpeedIPS.get()
        self.calHopperOuttakeTargetSpeedIPS = self.cals.hopperOuttakeSpeedIPS.get()
        self.calHopperShootTargetSpeedIPS = self.cals.hopperShootSpeedIPS.get()
        self.calFlywheelTargetSpeedIPS = self.cals.flywheelSpeedIPS.get()


    def periodicUpdateClosedLoopTargetSpeeds(self) -> None:
        match self.state:

            case InOutState.kIntaking:
                self.inputs.groundTargetIPS = self.calGroundIntakeTargetSpeedIPS
                self.inputs.hopperTargetIPS = self.calHopperIntakeTargetSpeedIPS

            case InOutState.kOutaking:
                self.inputs.groundTargetIPS = -self.calGroundOuttakeTargetSpeedIPS
                self.inputs.hopperTargetIPS = -self.calHopperOuttakeTargetSpeedIPS

            case InOutState.kShooting:
                self.inputs.groundTargetIPS = self.calGroundShootTargetSpeedIPS
                self.inputs.hopperTargetIPS = -self.calHopperShootTargetSpeedIPS

            case InOutState.kOff | _:
                self.inputs.groundTargetIPS = 0.0
                self.inputs.hopperTargetIPS = 0.0

        match self.flywheelState:
            case FlywheelState.kSpinningUp:
                self.inputs.flywheelTargetIPS = self.calFlywheelTargetSpeedIPS
            case FlywheelState.kOff | _:
                self.inputs.flywheelTargetIPS = 0.0

    def _updatePIDGainsAndFeedForward(self) -> None:
        for module in self.modules:
            module.updatePIDandFF()

    def periodicUpdateClosedLoopOutputs(self) -> None:
        self.groundModule.updateClosedLoopOutput(
            self.groundInPerSToRadPerS(self.inputs.groundTargetIPS))
        self.hopperModule.updateClosedLoopOutput(
            self.hopperInPerSToRadPerS(self.inputs.hopperTargetIPS))
        self.flywheelModule.updateClosedLoopOutput(
            self.flywheelInPerSToRadPerS(self.inputs.flywheelTargetIPS))

    def setClosedLoop(self, closedLoop: bool) -> None:
        self.isClosedLoop = closedLoop

    def setState(self, state: InOutState):
        self.state = state

    def setFlywheelState(self, flywheelState: FlywheelState):
        self.flywheelState = flywheelState

    @classmethod
    def groundRadPerSToInPerS(cls, radiansPerS: float) -> float:
        return radiansPerS * cls.GROUND_WHEEL_RADIUS_INCHES * cls.GROUND_GEAR_REDUCTION

    @classmethod
    def groundInPerSToRadPerS(cls, inPerS: float) -> float:
        return inPerS / (cls.GROUND_WHEEL_RADIUS_INCHES * cls.GROUND_GEAR_REDUCTION)

    @classmethod
    def hopperRadPerSToInPerS(cls, radiansPerS: float) -> float:
        return radiansPerS *  cls.HOPPER_WHEEL_RADIUS_INCHES * cls.HOPPER_GEAR_REDUCTION

    @classmethod
    def hopperInPerSToRadPerS(cls, inPerS: float) -> float:
        return inPerS / (cls.HOPPER_WHEEL_RADIUS_INCHES * cls.HOPPER_GEAR_REDUCTION)

    @classmethod
    def flywheelRadPerSToInPerS(cls, radiansPerS: float) -> float:
        return radiansPerS * cls.FLYWHEEL_WHEEL_RADIUS_INCHES * cls.FLYWHEEL_GEAR_REDUCTION

    @classmethod
    def flywheelInPerSToRadPerS(cls, inPerS: float) -> float:
        return inPerS / (cls.FLYWHEEL_WHEEL_RADIUS_INCHES * cls.FLYWHEEL_GEAR_REDUCTION)

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

    def doNothing(self):
        pass

    def useOperatorControls(self):
        match self.oInt.inOutCommand:
            case InOutCommand.kIntaking:
                self.setState(InOutState.kIntaking)
            case InOutCommand.kOutaking:
                self.setState(InOutState.kOutaking)
            case InOutCommand.kShooting:
                self.setState(InOutState.kShooting)
            case InOutCommand.kOff|_:
                self.setState(InOutState.kOff)

        match self.oInt.flywheelCommand:
            case FlywheelCommand.kNoCommand:
                pass
            case FlywheelCommand.kSpinningUp:
                self.setFlywheelState(FlywheelState.kSpinningUp)
            case FlywheelCommand.kSpinningDown|_:
                self.setFlywheelState(FlywheelState.kOff)

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


# See https://github.com/FRCTeam360/RainMaker26/blob/ac5238c1ef05ec7bd4adafc81331d94dc29ffe08/src/main/java/frc/robot/subsystems/Indexer/IndexerIOSim.java#L3
# for reference.
@dataclass
class OperateFlywheelSimulation():
    wrapperedMotor: WrapperedMotorSuper
    gearRatio: float        # Gear Reduction is > 1.0
    moi: float              # Moment of inertia in kg·m² = = 0.0513951385
    gearBox: DCMotor = field(init=False)        # e.g: DCMotor.getNEO(1);
    motorCtrl: SparkBase = field(init=False)   # was motorControllerSim
    sparkSim: SparkSim = field(init=False)  #
    plant: LinearSystemSim_1_1_1 = field(init=False)
    flywheelSim: FlywheelSim = field(init=False)

    def __post_init__(self) -> None:
        self.gearBox = self.wrapperedMotor.spark.gearBox
        self.motorCtrl = self.wrapperedMotor.spark.ctrl # TODO Clean this up
        self.sparkSim = self.wrapperedMotor.spark.sparkSim
        self.plant = LinearSystemId.flywheelSystem(self.gearBox, self.moi, self.gearRatio) # TODO Investigate if gearRatio and moi and inertia make sense
        self.flywheelSim = FlywheelSim(self.plant, self.gearBox, measurementStdDevs=[0.01])
        # TODO Mike Stitt doesn't think we need this step
        # self.motorCtrl.set(0.0)

    def periodic(self):
        # Step 4: Use SparkMaxSim.iterate() to update the Spark MAX/Flex with simulated values
        self.sparkSim.iterate(
            radPerSec2RPM(self.flywheelSim.getAngularVelocity()) * self.gearRatio, # Motor velocity in RPM
            RoboRioSim.getVInVoltage(),            # Simulated battery voltage
            kRobotUpdatePeriodS);                  # Time interval
        # Step 2: Set the input voltage to the physics simulation
        appliedVoltage = self.sparkSim.getAppliedOutput() * self.sparkSim.getBusVoltage()
        self.flywheelSim.setInputVoltage(appliedVoltage)
        # Step 3: Update the physics simulation
        self.flywheelSim.update(kRobotUpdatePeriodS)
        #self.sparkSim.setVelocity(self.flywheelSim.getAngularVelocity())
        # TODO Mike Stitt - I think we need this across all currend draws
        # Step 5: Update battery voltage based on current draw
        RoboRioSim.setVInVoltage(
            BatterySim.calculate(12.0, 0.020, [self.flywheelSim.getCurrentDraw()])
        )

class InOutSubsystemSimulation():
    def __init__(self,
            groundMotor:WrapperedMotorSuper,
            hopperMotor: WrapperedMotorSuper,
            flywheelMotor: WrapperedMotorSuper) -> None:
        self.groundWheelSim = OperateFlywheelSimulation(
            wrapperedMotor=groundMotor,
            gearRatio=1/InOutSubsystem.GROUND_GEAR_REDUCTION,
            moi=0.005,
        )
        self.hopperWheelSim = OperateFlywheelSimulation(
            wrapperedMotor=hopperMotor,
            gearRatio=1/InOutSubsystem.HOPPER_GEAR_REDUCTION,
            moi=0.005,
        )
        self.flywheelWheelSim = OperateFlywheelSimulation(
            wrapperedMotor=flywheelMotor,
            gearRatio=1/InOutSubsystem.FLYWHEEL_GEAR_REDUCTION,
            moi=0.01,
        )
        self.simulations = (self.groundWheelSim, self.hopperWheelSim, self.flywheelWheelSim)

    def periodic(self):
        for simulation in self.simulations:
            simulation.periodic()


def inoutSubsystemFactory() -> InOutSubsystem|None:
    inout: Optional[InOutSubsystem] = None
    if HAS_INOUT:
        match kRobotMode:
            case RobotModes.REAL | RobotModes.SIMULATION:

                groundMotorGearBox: Optional[DCMotor] = None
                hopperMotorGearBox: Optional[DCMotor] = None
                flywheelMotorGearBox: Optional[DCMotor] = None

                if kRobotMode == RobotModes.SIMULATION:
                    groundMotorGearBox = DCMotor.NEO(1)
                    hopperMotorGearBox = DCMotor.NEO(1)
                    flywheelMotorGearBox = DCMotor.neoVortex(1)

                inoutCals = InOutCalSet()
                groundMotor = WrapperedSparkMotor.makeSparkMax(name="groundMotor",
                                                canID=IODC["GROUND_MOTOR_CANID"],
                                                gearBox=groundMotorGearBox)
                groundMotor.setInverted(IODC["GROUND_MOTOR_INVERTED"])
                hopperMotor = WrapperedSparkMotor.makeSparkMax(name="hopperMotor",
                                                canID=IODC["HOPPER_MOTOR_CANID"],
                                                gearBox=hopperMotorGearBox)
                hopperMotor.setInverted(IODC["HOPPER_MOTOR_INVERTED"])
                flywheelMotor = WrapperedSparkMotor.makeSparkFlex(name="flywheelMotor",
                                                   canID=IODC["FLYWHEEL_MOTOR_CANID"],
                                                   gearBox=flywheelMotorGearBox)
                flywheelMotor.setInverted(IODC["FLYWHEEL_MOTOR_INVERTED"])

                """
                groundController = SparkVelocityController(
                    cals=inoutCals.groundCals
                )
                hopperController = SparkVelocityController(
                    cals=inoutCals.hopperCals
                )
                flywheelController = SparkVelocityController(
                    cals=inoutCals.flywheelCals
                )


                groundController = MaxMotionController(
                    cals=inoutCals.groundCals,
                    userUnitsToRadPerSec=InOutSubsystem.groundInPerSToRadPerS
                )
                hopperController = MaxMotionController(
                    cals=inoutCals.hopperCals,
                    userUnitsToRadPerSec=InOutSubsystem.hopperInPerSToRadPerS
                )
                flywheelController = MaxMotionController(
                    cals=inoutCals.flywheelCals,
                    userUnitsToRadPerSec=InOutSubsystem.flywheelInPerSToRadPerS
                )
                """

                groundController = SparkSlewRateLimitedVelocityController(
                    cals=inoutCals.groundCals,
                    userUnitsToRadPerSec=InOutSubsystem.groundInPerSToRadPerS
                )
                hopperController = SparkSlewRateLimitedVelocityController(
                    cals=inoutCals.hopperCals,
                    userUnitsToRadPerSec=InOutSubsystem.hopperInPerSToRadPerS
                )
                flywheelController = SparkSlewRateLimitedVelocityController(
                    cals=inoutCals.flywheelCals,
                    userUnitsToRadPerSec=InOutSubsystem.flywheelInPerSToRadPerS
                )

                inoutSim = None
                if kRobotMode == RobotModes.SIMULATION:
                    inoutSim = InOutSubsystemSimulation(groundMotor, hopperMotor, flywheelMotor)
            case _:
                pass

        match kRobotMode:
            case RobotModes.REAL:
                inout = InOutSubsystem(
                    io=InOutSubsystemIOReal(name="inoutIO"),
                    groundModule_io=MotorModuleIOWrappered(name="inoutGroundModuleIO", motor=groundMotor),
                    hopperModule_io=MotorModuleIOWrappered(name="inoutHopperModuleIO", motor=hopperMotor),
                    flywheelModule_io=MotorModuleIOWrappered(name="inoutFlywheelModuleIO", motor=flywheelMotor),
                    groundModule_controller=groundController,
                    hopperModule_controller=hopperController,
                    flywheelModule_controller=flywheelController,
                )
            case RobotModes.SIMULATION:
                inout = InOutSubsystem(
                    io=InOutSubsystemIORealSim(name="inoutIO"),
                    groundModule_io=MotorModuleIOWrapperedSim(name="inoutGroundModuleIO", motor=groundMotor),
                    hopperModule_io=MotorModuleIOWrapperedSim(name="inoutHopperModuleIO", motor=hopperMotor),
                    flywheelModule_io=MotorModuleIOWrapperedSim(name="inoutFlywheelModuleIO", motor=flywheelMotor),
                    groundModule_controller=groundController,
                    hopperModule_controller=hopperController,
                    flywheelModule_controller=flywheelController,
                    simulation=inoutSim
                )
            case RobotModes.REPLAY | _:
                inout = InOutSubsystem(
                    io=InOutSubsystemIO(),
                    groundModule_io=MotorModuleIO(name="inoutGroundModuleIO"),
                    hopperModule_io=MotorModuleIO(name="inoutHopperModuleIO"),
                    flywheelModule_io=MotorModuleIO(name="inoutFlywheelModuleIO"),
                    groundModule_controller=NullController(),
                    hopperModule_controller=NullController(),
                    flywheelModule_controller=NullController(),
                )

        match kRobotMode:
            case RobotModes.SIMULATION:
                groundMaxFreeSpeedRadps = groundMotorGearBox.freeSpeed
                groundMaxFreeSpeedRPM = radPerSec2RPM(groundMaxFreeSpeedRadps)
                groundMaxFreeSpeedIPS =  inout.groundRadPerSToInPerS(groundMaxFreeSpeedRadps)
                print(f"groundMaxFreeSpeedRPM={groundMaxFreeSpeedRPM:.0f} groundMaxFreeSpeedIPS={groundMaxFreeSpeedIPS:.0f}"
                      f" calGroundIntakeTargetSpeedIPS={inout.calGroundIntakeTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={inout.calGroundIntakeTargetSpeedIPS/groundMaxFreeSpeedIPS:.2f}")
                hopperMaxFreeSpeedRadps = hopperMotorGearBox.freeSpeed
                hopperMaxFreeSpeedRPM = radPerSec2RPM(hopperMaxFreeSpeedRadps)
                hopperMaxFreeSpeedIPS =  inout.hopperRadPerSToInPerS(hopperMaxFreeSpeedRadps)
                print(f"hopperMaxFreeSpeedRPM={hopperMaxFreeSpeedRPM:.0f} hopperMaxFreeSpeedIPS={hopperMaxFreeSpeedIPS:.0f}"
                      f" calHopperIntakeTargetSpeedIPS={inout.calHopperIntakeTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={inout.calHopperIntakeTargetSpeedIPS/hopperMaxFreeSpeedIPS:.2f}")
                flywheelMaxFreeSpeedRadps = flywheelMotorGearBox.freeSpeed
                flywheelMaxFreeSpeedRPM = radPerSec2RPM(flywheelMaxFreeSpeedRadps)
                flywheelMaxFreeSpeedIPS =  inout.flywheelRadPerSToInPerS(flywheelMaxFreeSpeedRadps)
                print(f"flywheelMaxFreeSpeedRPM={flywheelMaxFreeSpeedRPM:.0f} flywheelMaxFreeSpeedIPS={flywheelMaxFreeSpeedIPS:.0f}"
                      f" calFlywheelIntakeTargetSpeedIPS={inout.calFlywheelTargetSpeedIPS:.0f}"
                      f" ratioOfMaxSpeed={inout.calFlywheelTargetSpeedIPS/flywheelMaxFreeSpeedIPS:.2f}")
            case RobotModes.REAL|RobotModes.REPLAY | _:
                pass

    return inout