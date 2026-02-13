import time
from typing import Optional

from rev import SparkFlex, SparkFlexConfig, REVLibError, ClosedLoopSlot, SparkBaseConfig, ResetMode, PersistMode, \
    SparkMaxSim
from rev import SparkClosedLoopController
from wpilib import TimedRobot
from wpimath.system.plant import DCMotor

from constants import kRobotUpdatePeriodMs
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
from wrappers.wrapperedMotorCommon import MotorControlStates
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

## Wrappered Spark Flex
# Wrappers REV's libraries to add the following functionality for spark max controllers:
# Grouped PID controller, Encoder, and motor controller objects
# Physical unit conversions into SI units (radians)
# Retry logic for initial configuration
# Fault handling for not crashing code if the motor controller is disconnected
# Fault annunication logic to trigger warnings if a motor couldn't be configured
class WrapperedSparkFlex(WrapperedMotorSuper):
    def __init__(self, canID:int, name:str, brakeMode:bool=False, currentLimitA:int=40, gearBox:Optional[DCMotor]=None):
        self.ctrl = SparkFlex(canID, SparkFlex.MotorType.kBrushless)
        self.sparkSim: Optional[SparkMaxSim] = None
        self.gearbox: Optional[DCMotor] = gearBox
        if self.gearbox is not None:
            self.sparkSim = SparkMaxSim(self.ctrl, self.gearbox)
        self.closedLoopCtrl = self.ctrl.getClosedLoopController()
        self.encoder = self.ctrl.getEncoder()
        self.name = name
        self.currentLimitA = round(currentLimitA)
        self.configSuccess = False
        self.disconFault = Fault(f"Spark Max {name} ID {canID} disconnected")
        self.simActPos = 0
        self.canID = canID

        # pylint: disable= R0801
        self.desPosRad = 0.0
        self.desVelRadps = 0.0
        self.desVolt = 0.0
        self.actPosRad = 0.0
        self.actVelRadps = 0.0
        self.actVolt = 0.0
        self.controlState = MotorControlStates.UNKNOWN

        self.cfg = SparkFlexConfig()
        self.cfg.signals.appliedOutputPeriodMs(200)
        self.cfg.signals.busVoltagePeriodMs(200)
        self.cfg.signals.primaryEncoderPositionPeriodMs(kRobotUpdatePeriodMs)
        self.cfg.signals.primaryEncoderVelocityPeriodMs(kRobotUpdatePeriodMs)
        self.cfg.setIdleMode(SparkBaseConfig.IdleMode.kBrake if brakeMode else SparkBaseConfig.IdleMode.kCoast)
        self.cfg.smartCurrentLimit(self.currentLimitA,0,5700)

        self._spark_config(retries=10, resetMode=ResetMode.kResetSafeParameters, persistMode=PersistMode.kPersistParameters, step="Initial Config")

        print(f"Init of SparkFlex {self.name} CANID={self.canID} is finished")

    def _spark_config(self, retries, resetMode, persistMode, printResults=True, step=""):
        # Perform motor configuration, tracking errors and retrying until we have success
        # Clear previous configuration, and persist anything set in this state.
        retryCounter = 0
        success=False
        while not success and retryCounter < retries:
            retryCounter += 1
            err = self.ctrl.configure(self.cfg, resetMode, persistMode)

            # Check if any operation triggered an error
            if err != REVLibError.kOk:
                if printResults:
                    print(
                        f"{step} Failure configuring Spark Max {self.name} CAN ID {self.canID}, retrying..."
                    )
            else:
                # Only attempt other communication if we're able to successfully configure
                if printResults:
                    print(f"{step} Successfully connected to {self.name} motor")
                success = True
            if retryCounter < retries:
                time.sleep(0.1)

        self.configSuccess = success

        self.disconFault.set(not self.configSuccess)

    def setFollow(self, leaderCanID:int, invert:bool=False)->None:
        self.cfg.follow(leaderCanID, invert)
        self.ctrl.configure(self.cfg,
                                ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters)

    def setInverted(self, isInverted:bool)->None:
        if self.configSuccess:
            self.cfg.inverted(isInverted)
            self.ctrl.configure(self.cfg,
                                ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters)

    def setPID(self, kP:float, kI:float, kD:float)->None:
        if self.configSuccess:
            self.cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark max.
            persist = PersistMode.kPersistParameters
            self.ctrl.configure(self.cfg,
                                ResetMode.kNoResetSafeParameters,
                                persist)

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            posCmd (float): motor desired shaft rotations in radians
            arbFF (int, optional): _description_. Defaults to 0.
        """
        self.simActPos = posCmdRad
        posCmdRev = rad2Rev(posCmdRad)

        self.desPosRad = posCmdRad
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                posCmdRev,
                SparkFlex.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.controlState = MotorControlStates.POSITION
            self.disconFault.set(err != REVLibError.kOk)

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            velCmd (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """

        self.desVelRadps = velCmdRadps
        desVelRPM = radPerSec2RPM(velCmdRadps)
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                desVelRPM,
                SparkFlex.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.controlState = MotorControlStates.VELOCITY
            self.disconFault.set(err != REVLibError.kOk)

    def setVoltage(self, outputVoltageVolts:float)->None:
        self.desVolt = outputVoltageVolts
        if self.configSuccess:
            self.ctrl.setVoltage(outputVoltageVolts)
            self.controlState = MotorControlStates.VOLTAGE

    def getMotorPositionRad(self)->float:
        if(TimedRobot.isSimulation()):
            pos = self.simActPos
        else:
            if self.configSuccess:
                pos = rev2Rad(self.encoder.getPosition())
            else:
                pos = 0
        self.actPosRad = pos
        return pos

    def getMotorVelocityRadPerSec(self)->float:
        if self.configSuccess:
            vel = self.encoder.getVelocity()
        else:
            vel = 0
        self.actVelRadps = RPM2RadPerSec(vel)
        return self.actVelRadps

    def getAppliedOutput(self)->float:
        self.actVolt = self.ctrl.getAppliedOutput() * 12
        return self.actVolt

    def getCurrentLimitA(self)->int:
        return self.currentLimitA

    def getControlState(self)->MotorControlStates:
        return self.controlState

    def setSmartCurrentLimit(self, currentLimitA: int)->None:
        self.currentLimitA = round(currentLimitA)
        self.cfg.smartCurrentLimit(self.currentLimitA,0,5700)
        self._spark_config(retries=4, resetMode=ResetMode.kNoResetSafeParameters, persistMode=PersistMode.kNoPersistParameters, printResults=True, step="Current Limit")

    def getOutputTorqueCurrentA(self)->float:
        return self.ctrl.getOutputCurrent()
