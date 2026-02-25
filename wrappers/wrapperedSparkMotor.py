import time
from typing import Optional

from rev import SparkBase, SparkFlex, SparkFlexConfig, REVLibError, ClosedLoopSlot, SparkBaseConfig, ResetMode, \
    PersistMode, \
    SparkFlexSim, SparkSim, SparkMax, SparkMaxConfig, SparkMaxSim
from rev import SparkClosedLoopController
from wpilib import TimedRobot
from wpimath.system.plant import DCMotor

from constants import kRobotUpdatePeriodMs
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
from wrappers.wrapperedMotorCommon import MotorControlStates
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

class WrapperedSparkImplementation():
    def __init__(self, ctrl:SparkBase, cfg:SparkBaseConfig, gearBox:Optional[DCMotor]=None, sparkSim:Optional[SparkSim]=None, limitRPM:int=0):
        self.ctrl = ctrl
        self.cfg = cfg
        self.gearBox = gearBox
        self.sparkSim = sparkSim
        self.limitRPM = limitRPM


## Wrappered Spark Max or Flex
# Wrappers REV's libraries to add the following functionality for spark controllers:
# Grouped PID controller, Encoder, and motor controller objects
# Physical unit conversions into SI units (radians)
# Retry logic for initial configuration
# Fault handling for not crashing code if the motor controller is disconnected
# Fault annunication logic to trigger warnings if a motor couldn't be configured
class WrapperedSparkMotor(WrapperedMotorSuper):
    NEO_CONFIGURED_FREESPEED_RADPS = DCMotor.NEO(1).freeSpeed
    VORTEX_CONFIGURED_FREESPEED_RADPS = DCMotor.neoVortex(1).freeSpeed

    def __init__(self, spark:WrapperedSparkImplementation, canID:int, name:str, brakeMode:bool=False, currentLimitA:int=40):
        self.spark = spark
        self.closedLoopCtrl = self.spark.ctrl.getClosedLoopController()
        self.encoder = self.spark.ctrl.getEncoder()
        self.name = name
        self.currentLimitA = round(currentLimitA)
        self.configSuccess = False
        self.disconFault = Fault(f"Spark Controller {name} ID {canID} disconnected")
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

        self.spark.cfg.signals.appliedOutputPeriodMs(200)
        self.spark.cfg.signals.busVoltagePeriodMs(200)
        self.spark.cfg.signals.primaryEncoderPositionPeriodMs(kRobotUpdatePeriodMs)
        self.spark.cfg.signals.primaryEncoderVelocityPeriodMs(kRobotUpdatePeriodMs)
        self.spark.cfg.setIdleMode(SparkBaseConfig.IdleMode.kBrake if brakeMode else SparkBaseConfig.IdleMode.kCoast)
        self.spark.cfg.smartCurrentLimit(self.currentLimitA,0, self.spark.limitRPM)

        self._spark_config(retries=10, resetMode=ResetMode.kResetSafeParameters, persistMode=PersistMode.kPersistParameters, step="Initial Config")

        print(f"Init of Spark Controller {self.name} CANID={self.canID} is finished")

    @classmethod
    def makeMaxImplementaion(cls, canID: int, gearBox: Optional[DCMotor] = None)->WrapperedSparkImplementation:
        ctrl = SparkMax(canID, SparkBase.MotorType.kBrushless)
        cfg = SparkMaxConfig()
        sparkSim: Optional[SparkSim] = None
        if gearBox is not None:
            sparkSim = SparkMaxSim(ctrl, gearBox)
        limitRPM = int(radPerSec2RPM(cls.NEO_CONFIGURED_FREESPEED_RADPS))
        spark = WrapperedSparkImplementation(ctrl, cfg, gearBox, sparkSim, limitRPM)
        return spark

    @classmethod
    def makeFlexImplementaion(cls, canID: int, gearBox: Optional[DCMotor] = None)->WrapperedSparkImplementation:
        ctrl = SparkFlex(canID, SparkBase.MotorType.kBrushless)
        cfg = SparkFlexConfig()
        sparkSim: Optional[SparkSim] = None
        if gearBox is not None:
            sparkSim = SparkFlexSim(ctrl, gearBox)
        limitRPM = int(radPerSec2RPM(cls.VORTEX_CONFIGURED_FREESPEED_RADPS))

        spark = WrapperedSparkImplementation(ctrl, cfg, gearBox, sparkSim, limitRPM)
        return spark

    @classmethod
    def makeSparkMax(cls, canID: int, name: str, brakeMode: bool = False, currentLimitA: int = 40,
                      gearBox: Optional[DCMotor] = None):
        spark = cls.makeMaxImplementaion(canID, gearBox)
        instance = WrapperedSparkMotor(spark, canID, name, brakeMode, currentLimitA)
        return instance

    @classmethod
    def makeSparkFlex(cls, canID:int, name:str, brakeMode:bool=False, currentLimitA:int=40, gearBox:Optional[DCMotor]=None):
        spark = cls.makeFlexImplementaion(canID, gearBox)
        instance = WrapperedSparkMotor(spark, canID, name, brakeMode, currentLimitA)
        return instance

    def _spark_config(self, retries, resetMode, persistMode, printResults=True, step=""):
        # Perform motor configuration, tracking errors and retrying until we have success
        # Clear previous configuration, and persist anything set in this state.
        retryCounter = 0
        success=False
        while not success and retryCounter < retries:
            retryCounter += 1
            err = self.spark.ctrl.configure(self.spark.cfg, resetMode, persistMode)

            # Check if any operation triggered an error
            if err != REVLibError.kOk:
                if printResults:
                    print(
                        f"{step} Failure configuring Spark Controller {self.name} CAN ID {self.canID}, retrying..."
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
        self.spark.cfg.follow(leaderCanID, invert)
        self.spark.ctrl.configure(self.spark.cfg,
                                ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters)

    def setInverted(self, isInverted:bool)->None:
        if self.configSuccess:
            self.spark.cfg.inverted(isInverted)
            self.spark.ctrl.configure(self.spark.cfg,
                                ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters)

    def setPID(self, kP:float, kI:float, kD:float)->None:
        if self.configSuccess:
            self.spark.cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark controller.
            persist = PersistMode.kPersistParameters
            self.spark.ctrl.configure(self.spark.cfg,
                                ResetMode.kNoResetSafeParameters,
                                persist)

    def setPIDFF(self, kP: float, kI: float, kD: float, kS: float, kV: float, kA: float) -> None:
        if self.configSuccess:
            (self.spark.cfg
                 .closedLoop
                 .pid(
                    kP, # DutyCycle/Rev
                    kI, # DutyCycle/(rev*ms)
                    kD, # (DutyCycle*ms)/rev
                    ClosedLoopSlot.kSlot0)
                 .feedForward
                 .kS(kS,ClosedLoopSlot.kSlot0) # Volts
                 .kV(kV,ClosedLoopSlot.kSlot0) # Volts/RPM
                 .kA(kA,ClosedLoopSlot.kSlot0) # Volts/(RPM/s)
            )
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark max.
            persist = PersistMode.kPersistParameters
            self.spark.ctrl.configure(self.spark.cfg,
                                ResetMode.kNoResetSafeParameters,
                                persist)

    def setFeedForwardKA(self, kA:float) -> None:
        if self.configSuccess:
            self.spark.cfg.closedLoop.feedForward.kA(kA,ClosedLoopSlot.kSlot0)

    def setMaxMotionVelParams(self, maxAccRadps2:float) -> None:
        maxAccRadps2 = float(maxAccRadps2)
        maxAccRPMps = radPerSec2RPM(maxAccRadps2)
        if self.configSuccess:
            (self.spark.cfg
                 .closedLoop
                 .maxMotion
                 .maxAcceleration(maxAccRPMps, ClosedLoopSlot.kSlot0)
            )
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark max.
            persist = PersistMode.kPersistParameters
            self.spark.ctrl.configure(self.spark.cfg,
                                ResetMode.kNoResetSafeParameters,
                                persist)


    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            posCmdRad (float): motor desired shaft rotations in radians
            arbFF (float, optional): _description_. Defaults to 0.
        """
        posCmdRad = float(posCmdRad)
        arbFF = float(arbFF)
        self.simActPos = posCmdRad
        posCmdRev = rad2Rev(posCmdRad)

        self.desPosRad = posCmdRad
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                posCmdRev,
                SparkBase.ControlType.kPosition,
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
        velCmdRadps = float(velCmdRadps)
        arbFF = float(arbFF)
        self.desVelRadps = velCmdRadps
        desVelRPM = radPerSec2RPM(velCmdRadps)
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                desVelRPM,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.controlState = MotorControlStates.VELOCITY
            self.disconFault.set(err != REVLibError.kOk)


    def setMaxMotionVelCmd(self, velCmdRadps:float)->None:
        """_summary_

        Args:
            velCmd (float): motor desired shaft velocity in radians per second
        """
        velCmdRadps = float(velCmdRadps)
        self.desVelRadps = velCmdRadps
        desVelRPM = radPerSec2RPM(velCmdRadps)
        self.desVolt = 0.0

        if self.configSuccess:
            err = self.closedLoopCtrl.setSetpoint(
                desVelRPM,
                SparkBase.ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot0,
            )
            self.controlState = MotorControlStates.MAXMOTIONVELOCITY
            self.disconFault.set(err != REVLibError.kOk)

    def setVoltage(self, outputVoltageVolts:float)->None:
        outputVoltageVolts = float(outputVoltageVolts)
        self.desVolt = outputVoltageVolts
        if self.configSuccess:
            #self.spark.ctrl.setVoltage(outputVoltageVolts)
            err = self.closedLoopCtrl.setReference(
                outputVoltageVolts,
                SparkBase.ControlType.kVoltage,
                ClosedLoopSlot.kSlot0
            )
            self.controlState = MotorControlStates.VOLTAGE

    def getMotorPositionRad(self)->float:
        if(TimedRobot.isSimulation() and self.spark.gearBox is None):
            pos = self.simActPos
        else:
            if self.configSuccess:
                pos = rev2Rad(self.encoder.getPosition())
            else:
                pos = 0.0
        self.actPosRad = pos
        return pos

    def getMotorVelocityRadPerSec(self)->float:
        if self.configSuccess:
            vel = self.encoder.getVelocity()
        else:
            vel = 0.0
        self.actVelRadps = RPM2RadPerSec(vel)
        return self.actVelRadps

    def getAppliedOutput(self)->float:
        self.actVolt = self.spark.ctrl.getAppliedOutput() * 12
        return self.actVolt

    def getDesiredVoltageOrFF(self)->float:
        return self.desVolt

    def getCurrentLimitA(self)->int:
        return self.currentLimitA

    def getControlState(self)->MotorControlStates:
        return self.controlState

    def setSmartCurrentLimit(self, currentLimitA: int)->None:
        self.currentLimitA = round(currentLimitA)
        self.spark.cfg.smartCurrentLimit(self.currentLimitA,0,5700)
        self._spark_config(retries=4, resetMode=ResetMode.kNoResetSafeParameters, persistMode=PersistMode.kNoPersistParameters, printResults=True, step="Current Limit")

    def getOutputTorqueCurrentA(self)->float:
        return self.spark.ctrl.getOutputCurrent()

def WrapperedSparkBase(canID:int, name:str, brakeMode:bool=False, currentLimitA:int=40, gearBox:Optional[DCMotor]=None )->WrapperedSparkMotor:
    pass




