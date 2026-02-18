from dataclasses import dataclass, field
from enum import Enum
from math import pi
from typing import Optional, Callable
from unittest import case

import numpy as np
from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from wpimath._controls._controls.controller import SimpleMotorFeedforwardRadians

from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from rev import SparkBase, SparkSim
from wpilib.simulation import LinearSystemSim_1_1_1, FlywheelSim, RoboRioSim, BatterySim
from wpilib.sysid import State
from wpimath.system.plant import DCMotor, LinearSystemId

from constants import kRobotMode, RobotModes, kRobotUpdatePeriodS
from humanInterface.operatorInterface import OperatorInterface
from subsystems.intakeOuttake.inoutcalset import InOutCalSet
from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO

from subsystems.intakeOuttake.inoutsubsystemioreal import InOutSubsystemIOReal
from subsystems.intakeOuttake.inoutsubsystemiosim import InOutSubsystemIORealSim
from subsystems.intakeOuttake.motormodule import MotorModule
from subsystems.intakeOuttake.motormoduleio import MotorModuleIO
from subsystems.intakeOuttake.motormoduleiowrappered import MotorModuleIOWrappered
from subsystems.intakeOuttake.motormoduleiowrapperedsim import MotorModuleIOWrapperedSim
from subsystems.state.configsubsystem import ConfigSubsystem

from utils.units import radPerSec2RPM
from westwood.util.logtracer import LogTracer
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper
from wrappers.wrapperedSparkFlex import WrapperedSparkFlex
from wrappers.wrapperedSparkMax import WrapperedSparkMax


class InOutState(Enum):
    kOff = 0
    kIntaking = 1
    kOutaking = 2
    kShooting = 3

class FlywheelState(Enum):
    kOff = 0
    kSpinningUp = 1

@autologgable_output
class InOutSubsystem(Subsystem):
    GROUND_GEAR_REDUCTION = ConfigSubsystem().inoutDepConstants["GROUND_GEAR_REDUCTION"]
    GROUND_WHEEL_DIAMETER_INCHES = ConfigSubsystem().inoutDepConstants["GROUND_WHEEL_DIAMETER_INCHES"]
    GROUND_WHEEL_RADIUS_INCHES = GROUND_WHEEL_DIAMETER_INCHES / 2.0
    HOPPER_GEAR_REDUCTION = ConfigSubsystem().inoutDepConstants["HOPPER_GEAR_REDUCTION"]
    HOPPER_WHEEL_DIAMETER_INCHES = ConfigSubsystem().inoutDepConstants["HOPPER_WHEEL_DIAMETER_INCHES"]
    HOPPER_WHEEL_RADIUS_INCHES = HOPPER_WHEEL_DIAMETER_INCHES / 2.0
    FLYWHEEL_GEAR_REDUCTION = ConfigSubsystem().inoutDepConstants["FLYWHEEL_GEAR_REDUCTION"]
    FLYWHEEL_WHEEL_DIAMETER_INCHES = ConfigSubsystem().inoutDepConstants["FLYWHEEL_WHEEL_DIAMETER_INCHES"]
    FLYWHEEL_WHEEL_RADIUS_INCHES = FLYWHEEL_WHEEL_DIAMETER_INCHES / 2.0

    def __init__(
        self,
        io: InOutSubsystemIO,
        groundModule_io: MotorModuleIO,
        hopperModule_io: MotorModuleIO,
        flywheelModule_io: MotorModuleIO,
        simulation: "InOutSubsystemSimulation|None" = None
    ) -> None:
        Subsystem.__init__(self)
        self.name = type(self).__name__
        self.setName(self.name)
        self.io = io
        self.inputs = InOutSubsystemIO.InOutSubsystemIOInputs()
        self.groundModule = MotorModule(name="groundModule", io=groundModule_io)
        self.hopperModule = MotorModule(name="hopperModule", io=hopperModule_io)
        self.flywheelModule = MotorModule(name="flywheelModule", io=flywheelModule_io)
        self.modules = (self.groundModule, self.hopperModule, self.flywheelModule)
        self.simulation = simulation

        self.state: InOutState = InOutState.kOff
        self.flywheelState: FlywheelState = FlywheelState.kOff

        self.cals = InOutCalSet()

        self._updateAllCals()

        self.oInt = OperatorInterface()

        self.isClosedLoop = True

    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        LogTracer.resetOuter("inoutSubsystem periodic")
        for module in self.modules:
            module.periodic()
        LogTracer.record("ModulesPeriodic")
        self.inputs.groundIPS = self.groundRadPerSToInPerS(self.groundModule.inputs.velRadps)
        self.inputs.hopperIPS = self.hopperRadPerSToInPerS(self.hopperModule.inputs.velRadps)
        self.inputs.flywheelIPS = self.flywheelRadPerSToInPerS(self.flywheelModule.inputs.velRadps)
        self.io.updateInputs(self.inputs)  # update state of the ionout subsystem
        Logger.processInputs("inout", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.cals.hasChanged():
            self._updateAllCals()

        #print(f"self.oInt.intake={self.oInt.intake}")
        if self.oInt.intake:
            #print("self.setState(InOutState.kIntaking)")
            self.setState(InOutState.kIntaking)
        elif self.oInt.outtake:
            self.setState(InOutState.kOutaking)
        elif self.oInt.shoot:
            self.setState(InOutState.kShooting)
        else:
            self.setState(InOutState.kOff)

        if self.oInt.spinUpFlywheel:
            self.setFlywheelState(FlywheelState.kSpinningUp)
        elif self.oInt.spinDownFlywheel:
            self.setFlywheelState(FlywheelState.kOff)

        if self.isClosedLoop:
            self.periodicUpdateClosedLoop()
        LogTracer.record("Closed Loop Control")
        Logger.recordOutput(f"{self.name}/ClosedLoop", self.isClosedLoop)
        if self.simulation:
            self.simulation.periodic()
            LogTracer.record("SimulationPeriodic")
        LogTracer.recordTotal()

    def _updateAllCals(self):
        self.groundModule.io.setPID(
            self.cals.groundP.get(), 0.0, self.cals.groundD.get()
        )
        self.hopperModule.io.setPID(
            self.cals.hopperP.get(), 0.0, self.cals.hopperD.get()
        )
        self.flywheelModule.io.setPID(
            self.cals.flywheelP.get(), 0.0, self.cals.flywheelD.get()
        )
        self.calGroundKs = self.cals.groundS.get()
        self.calGroundKv = self.cals.groundV.get()
        self.calHopperKs = self.cals.hopperS.get()
        self.calHopperKv = self.cals.hopperV.get()
        self.calFlywheelKs = self.cals.flywheelS.get()
        self.calFlywheelKv = self.cals.flywheelV.get()

        self.groundFF = SimpleMotorFeedforwardRadians(self.calGroundKs, self.calGroundKv, 0.0, kRobotUpdatePeriodS)
        self.hopperFF = SimpleMotorFeedforwardRadians(self.calHopperKs, self.calHopperKv, 0.0, kRobotUpdatePeriodS)
        self.flywheelFF = SimpleMotorFeedforwardRadians(self.calFlywheelKs, self.calFlywheelKv, 0.0, kRobotUpdatePeriodS)

        self.calGroundIntakeTargetSpeedIPS = self.cals.groundIntakeSpeedIPS.get()
        self.calGroundOuttakeTargetSpeedIPS = self.cals.groundOuttakeSpeedIPS.get()
        self.calGroundShootTargetSpeedIPS = self.cals.groundShootSpeedIPS.get()

        self.calHopperIntakeTargetSpeedIPS = self.cals.hopperIntakeSpeedIPS.get()
        self.calHopperOuttakeTargetSpeedIPS = self.cals.hopperOuttakeSpeedIPS.get()
        self.calHopperShootTargetSpeedIPS = self.cals.hopperShootSpeedIPS.get()
        self.calFlywheelTargetSpeedIPS = self.cals.flywheelSpeedIPS.get()


    def periodicUpdateClosedLoop(self) -> None:
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

            case InOutState.kOff|_:
                self.inputs.groundTargetIPS = 0.0
                self.inputs.hopperTargetIPS = 0.0

        match self.flywheelState:
            case FlywheelState.kSpinningUp:
                self.inputs.flywheelTargetIPS = self.calFlywheelTargetSpeedIPS
            case FlywheelState.kOff|_:
                self.inputs.flywheelTargetIPS = 0.0

        groundTargetRadPerS = self.groundInPerSToRadPerS(self.inputs.groundTargetIPS)
        groundFFV = self.groundFF.calculate(groundTargetRadPerS)
        Logger.recordOutput(f"{self.name}/groundFFV", groundFFV)
        self.groundModule.setVelCmd(groundTargetRadPerS, groundFFV)

        hopperTargetRadPerS = self.hopperInPerSToRadPerS(self.inputs.hopperTargetIPS)
        hopperFFV = self.hopperFF.calculate(hopperTargetRadPerS)
        Logger.recordOutput(f"{self.name}/hopperFFV", hopperFFV)
        self.hopperModule.setVelCmd(hopperTargetRadPerS, hopperFFV)

        flywheelTargetRadPerS = self.flywheelInPerSToRadPerS(self.inputs.flywheelTargetIPS)
        flywheelFFV = self.flywheelFF.calculate(flywheelTargetRadPerS)
        Logger.recordOutput(f"{self.name}/flywheelFFV", flywheelFFV)
        self.flywheelModule.setVelCmd(flywheelTargetRadPerS, flywheelFFV)

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

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

        def logState(state: State) -> None:
            loggedStateStr = ""
            match state:
                case State.kQuasistaticForward:
                    loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    loggedStateStr = "dynamic-reverse"
                case State.kNone:
                    loggedStateStr = "none"
            Logger.recordOutput("inout/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.5, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_turret_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )

    def dummy(self):
        pass

    def getDefaultCommand(self) -> Optional[Command]:
        return cmd.run(lambda: self.dummy(), self)

# See https://github.com/FRCTeam360/RainMaker26/blob/ac5238c1ef05ec7bd4adafc81331d94dc29ffe08/src/main/java/frc/robot/subsystems/Indexer/IndexerIOSim.java#L3
# for reference.
@dataclass
class OperateFlywheelSimulation():
    wrapperedMotor: WrapperedMotorSuper
    gearRatio: float        # Gear Reduction is > 1.0
    moi: float              # Moment of inertia in kg·m² = = 0.0513951385
    gearbox: DCMotor = field(init=False)        # e.g: DCMotor.getNEO(1);
    motorCtrl: SparkBase = field(init=False)   # was motorControllerSim
    sparkSim: SparkSim = field(init=False)  #
    plant: LinearSystemSim_1_1_1 = field(init=False)
    flywheelSim: FlywheelSim = field(init=False)

    def __post_init__(self) -> None:
        self.gearbox = self.wrapperedMotor.gearbox
        self.motorCtrl = self.wrapperedMotor.ctrl # TODO Clean this up
        self.sparkSim = self.wrapperedMotor.sparkSim
        self.plant = LinearSystemId.flywheelSystem(self.gearbox, self.moi, self.gearRatio)
        self.flywheelSim = FlywheelSim(self.plant, self.gearbox, measurementStdDevs=[0.01])
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
    config = ConfigSubsystem()
    inout: Optional[InOutSubsystem] = None
    if config.inoutDepConstants["HAS_INOUT"]:
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
                groundMotor = WrapperedSparkMax(name="groundMotor",
                                                canID=config.inoutDepConstants["GROUND_MOTOR_CANID"],
                                                gearBox=groundMotorGearBox)
                groundMotor.setInverted(config.inoutDepConstants["GROUND_MOTOR_INVERTED"])
                groundMotor.setPID(inoutCals.groundP.get(), 0.0, inoutCals.groundD.get())
                hopperMotor = WrapperedSparkMax(name="hopperMotor",
                                                canID=config.inoutDepConstants["HOPPER_MOTOR_CANID"],
                                                gearBox=hopperMotorGearBox)
                hopperMotor.setInverted(config.inoutDepConstants["HOPPER_MOTOR_INVERTED"])
                hopperMotor.setPID(inoutCals.hopperP.get(), 0.0, inoutCals.hopperD.get())
                flywheelMotor = WrapperedSparkFlex(name="flywheelMotor",
                                                   canID=config.inoutDepConstants["FLYWHEEL_MOTOR_CANID"],
                                                   gearBox=flywheelMotorGearBox)
                flywheelMotor.setInverted(config.inoutDepConstants["FLYWHEEL_MOTOR_INVERTED"])
                flywheelMotor.setPID(inoutCals.flywheelP.get(), 0.0, inoutCals.flywheelD.get())
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
                )
            case RobotModes.SIMULATION:
                inout = InOutSubsystem(
                    io=InOutSubsystemIORealSim(name="inoutIO"),
                    groundModule_io=MotorModuleIOWrapperedSim(name="inoutGroundModuleIO", motor=groundMotor),
                    hopperModule_io=MotorModuleIOWrapperedSim(name="inoutHopperModuleIO", motor=hopperMotor),
                    flywheelModule_io=MotorModuleIOWrapperedSim(name="inoutFlywheelModuleIO", motor=flywheelMotor),
                    simulation=inoutSim
                )
            case RobotModes.REPLAY | _:
                inout = InOutSubsystem(
                    io=InOutSubsystemIO(),
                    groundModule_io=MotorModuleIO(name="inoutGroundModuleIO"),
                    hopperModule_io=MotorModuleIO(name="inoutHopperModuleIO"),
                    flywheelModule_io=MotorModuleIO(name="inoutFlywheelModuleIO"),
                )

        inout.setDefaultCommand(inout.getDefaultCommand())

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

