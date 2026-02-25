from dataclasses import dataclass, field
from enum import Enum
from math import isclose
from typing import Optional

from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from wpilib import XboxController, Timer
from wpimath.controller import SimpleMotorFeedforwardRadians

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import SparkBase, SparkSim, ClosedLoopSlot
from wpilib.simulation import LinearSystemSim_1_1_1, FlywheelSim, RoboRioSim, BatterySim
from wpilib.sysid import State
from wpimath.system.plant import DCMotor, LinearSystemId

from constants import kRobotMode, RobotModes, kRobotUpdatePeriodS
from humanInterface.operatorInterface import OperatorInterface, FlywheelCommand, InOutCommand
from subsystems.intakeOuttake.inoutcalset import InOutCalSet
from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO

from subsystems.intakeOuttake.inoutsubsystemioreal import InOutSubsystemIOReal
from subsystems.intakeOuttake.inoutsubsystemiosim import InOutSubsystemIORealSim
from subsystems.intakeOuttake.motormodule import MotorModule
from subsystems.intakeOuttake.motormodulecontroller import MaxMotionController, NullController
from subsystems.intakeOuttake.motormoduleio import MotorModuleIO
from subsystems.intakeOuttake.motormoduleiowrappered import MotorModuleIOWrappered
from subsystems.intakeOuttake.motormoduleiowrapperedsim import MotorModuleIOWrapperedSim
from subsystems.state.configsubsystem import ConfigSubsystem

from utils.units import radPerSec2RPM, sign
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

        self.oInt = OperatorInterface()

        self.initialize()

        self.isClosedLoop = True

    def initialize(self):
        self.setState(InOutState.kOff)
        self.setFlywheelState(FlywheelState.kOff)
        for module in self.modules:
            module.reset()
        self.setDefaultCommand(self.aDoNothingCommand())


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

    def feedForwardCharacterization(self, motorModule: MotorModule) -> Command:
        velocitySamples: list[float] = []
        voltageSamples: list[float] = []
        timer = Timer()

        def setup():
            self.initialize()
            self.setClosedLoop(False)
            velocitySamples.clear()
            voltageSamples.clear()
            timer.restart()

        def run():
            voltage = timer.get() * motorModule.ffCharacterizationRampVPerS
            motorModule.setVoltage(voltage)
            velocitySamples.append(motorModule.getMotorVelocityRadPerSec())
            voltageSamples.append(voltage)

        def end(_interrupted: bool):
            n = len(velocitySamples)
            sumX = sum(velocitySamples)
            sumY = sum(voltageSamples)
            sumXY = sum(velocitySamples[i] * voltageSamples[i] for i in range(n))
            sumX2 = sum(v**2 for v in velocitySamples)
            kS = 0.0
            kV = 0.0
            divisor = (n * sumX2 - sumX * sumX)
            if divisor != 0.0:
                kS = (sumY * sumX2 - sumX * sumXY) / divisor
                kV = (n * sumXY - sumX * sumY) / divisor

            print("************************************************************")
            print(f"{motorModule.name} Feed Forward Characterization Results: \nkS = {kS}\nkV = {kV}")
            self.setClosedLoop(True)

        controller = XboxController(1)
        ffWaitCmd = cmd.waitUntil(
            lambda: controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        ffCmd = (cmd.runOnce(setup, self)
                 .andThen(cmd.run(run, self).withName(f"{motorModule.name} FeedForwardCharacterization"))
                 .finallyDo(end))
        ffWithWaits = ffWaitCmd.andThen(ffCmd.onlyWhile(lambda: controller.getRightBumper()))

        return ffWithWaits


    def sysIdRoutine(self,
                     name:str,
                     motorModule: MotorModule,
                     voltsPerSec:float=0.5,
                     stepVolts:float=6.0,
                     timeoutS:float=10.0) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

        self.loggedStateStr = "none"

        def logOutputs(_)->None:

            volts = motorModule.io.motor.getDesiredVoltageOrFF()
            Logger.recordOutput(f"inout {name} SysId/voltage", volts)
            radsPerSec = motorModule.io.motor.getMotorVelocityRadPerSec()
            Logger.recordOutput(f"inout {name} SysId/radsPerSec", radsPerSec)
            rad = motorModule.io.motor.getMotorPositionRad()
            Logger.recordOutput(f"inout {name} SysId/rad", rad)

            #print(f"{Timer.getTimestamp():.6} {name} {self.loggedStateStr} SysID: volts={volts}, radsPerSec={radsPerSec}, rad={rad}")
            assert type(volts) == float, f"volts is not a float: {volts}"
            assert type(radsPerSec) == float, f"radsPerSec is not a float: {radsPerSec}"
            assert type(rad) == float, f"rad is not a float: {rad}"
            assert -1.0e10 <= volts <= 1.0e10, f"volts is out of bounds: {volts}"
            assert -1.0e10 <= radsPerSec <= 1.0e10, f"radsPerSec is out of bounds: {radsPerSec}"
            assert -1.0e10 <= rad <= 1.0e10, f"rad is out of bounds: {rad}"

        def logState(state: State) -> None:
            match state:
                case State.kQuasistaticForward:
                    self.loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    self.loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    self.loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    self.loggedStateStr = "dynamic-reverse"
                case State.kNone|_:
                    self.loggedStateStr = "none"
            Logger.recordOutput(f"inout {name} SysId/state", self.loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(
                voltsPerSec, # ramp voltage rate in V/sec
                stepVolts, # step voltage in V
                timeoutS, # timeout in seconds
                logState),
            SysIdRoutine.Mechanism(
                motorModule.setVoltage,
                logOutputs,
                self,
            ),
        )

        controller = XboxController(1)

        sysIdQFWaitCmd = cmd.waitUntil(
            lambda: controller.getRightBumper() and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdQFCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward)
        sysIdQFWithWaits = sysIdQFWaitCmd.andThen(sysIdQFCmd.onlyWhile(lambda: controller.getRightBumper()))

        sysIdQRWaitCmd = cmd.waitUntil(
            lambda: controller.getRightBumper() and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdQRCmd = charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
        sysIdQRWithWaits = sysIdQRWaitCmd.andThen(sysIdQRCmd.onlyWhile(lambda: controller.getRightBumper()))

        sysIdDFWaitCmd = cmd.waitUntil(
            lambda: controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdDFCmd = charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward)
        sysIdDFWithWaits = sysIdDFWaitCmd.andThen(sysIdDFCmd.onlyWhile(lambda: controller.getRightBumper()))

        sysIdDRWaitCmd = cmd.waitUntil(
            lambda: controller.getRightBumper()  and abs(motorModule.getMotorVelocityRadPerSec())<0.1)
        sysIdDRCmd = charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        sysIdDRWithWaits = sysIdDRWaitCmd.andThen(sysIdDRCmd.onlyWhile(lambda: controller.getRightBumper()))

        def beforeTests()->None:
            self.setClosedLoop(False)

        return cmd.sequence(
            cmd.runOnce(beforeTests, self),
            sysIdQFWithWaits,
            sysIdQRWithWaits,
            sysIdDFWithWaits,
            sysIdDRWithWaits,
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )

    def makeCommandFeedForwardCharacterizationGroundMotor(self) -> Command:
        return self.feedForwardCharacterization(self.groundModule)

    def makeCommandFeedForwardCharacterizationHopperMotor(self) -> Command:
        return self.feedForwardCharacterization(self.hopperModule)

    def makeCommandFeedForwardCharacterizationFlywheelMotor(self) -> Command:
        return self.feedForwardCharacterization(self.flywheelModule)

    def makeSysIdCommandGroundMotor(self) -> Command:
        return self.sysIdRoutine("ground", self.groundModule)

    def makeSysIdCommandHopperMotor(self) -> Command:
        return self.sysIdRoutine("hopper", self.hopperModule)

    def makeSysIdCommandFlywheelMotor(self) -> Command:
        return self.sysIdRoutine("flywheel", self.flywheelModule)

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
            moi=0.05,
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
                groundMotor = WrapperedSparkMotor.makeSparkMax(name="groundMotor",
                                                canID=config.inoutDepConstants["GROUND_MOTOR_CANID"],
                                                gearBox=groundMotorGearBox)
                groundMotor.setInverted(config.inoutDepConstants["GROUND_MOTOR_INVERTED"])
                hopperMotor = WrapperedSparkMotor.makeSparkMax(name="hopperMotor",
                                                canID=config.inoutDepConstants["HOPPER_MOTOR_CANID"],
                                                gearBox=hopperMotorGearBox)
                hopperMotor.setInverted(config.inoutDepConstants["HOPPER_MOTOR_INVERTED"])
                flywheelMotor = WrapperedSparkMotor.makeSparkFlex(name="flywheelMotor",
                                                   canID=config.inoutDepConstants["FLYWHEEL_MOTOR_CANID"],
                                                   gearBox=flywheelMotorGearBox)
                flywheelMotor.setInverted(config.inoutDepConstants["FLYWHEEL_MOTOR_INVERTED"])

                groundController = MaxMotionController(cals=inoutCals.groundCals)
                hopperController = MaxMotionController(cals=inoutCals.hopperCals)
                flywheelController = MaxMotionController(cals=inoutCals.flywheelCals)

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