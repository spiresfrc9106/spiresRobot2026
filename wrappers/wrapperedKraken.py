from phoenix6 import hardware, configs, signals, controls, StatusCode, SignalLogger
from wpilib import TimedRobot
#from memes.ctreMusicPlayback import CTREMusicPlayback
from constants import kRobotUpdateFrequency
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
from wrappers.wrapperedMotorCommon import MotorControlStates
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

## Wrappered WCP Kraken, Powered by Talon FX.
# Wrappers their API's into a consistent set of API's for swappability with rev hardware
# on Casserole's robots.
# References:
# https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/index.html
# https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/python/PositionClosedLoop/robot.py
# https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/python/VelocityClosedLoop/robot.py

class WrapperedKraken(WrapperedMotorSuper):
    def __init__(self, canID:int, name:str, brakeMode:bool=False, currentLimitA:int=40):
        SignalLogger.enable_auto_logging(False)
        self.ctrl = hardware.TalonFX(canID, "rio")
        self.name = name
        self.configSuccess = False
        self.disconFault = Fault(f"Kraken {name} ID {canID} disconnected")

        self.cfg = configs.TalonFXConfiguration()
        self.currentLimitA = currentLimitA
        self.cfg.current_limits.supply_current_limit = currentLimitA
        self.cfg.current_limits.supply_current_limit_enable = True

        self.motorAppliedVoltage = self.ctrl.get_motor_voltage()
        self.motorAppliedVoltage.set_update_frequency(kRobotUpdateFrequency)
        self.motorTorqueCurrent = self.ctrl.get_torque_current()
        self.motorTorqueCurrent.set_update_frequency(kRobotUpdateFrequency)
        self.motorSupplyCurrent = self.ctrl.get_supply_current()
        self.motorSupplyCurrent.set_update_frequency(5.0)
        self.motorPosSig = self.ctrl.get_rotor_position()
        self.motorPosSig.set_update_frequency(kRobotUpdateFrequency)
        self.motorVelSig = self.ctrl.get_rotor_velocity()
        self.motorVelSig.set_update_frequency(kRobotUpdateFrequency)

        self.cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE if brakeMode else signals.NeutralModeValue.COAST 

        self._applyCurCfg()

        self.desPos = 0
        self.desVel = 0
        self.desVolt = 0
        self.actPos = 0
        self.actVel = 0
        self.controlState = MotorControlStates.UNKNOWN

        # Simulation Suport
        self.simActPos = 0

        # Register with the music player
        #CTREMusicPlayback().registerDevice(self.ctrl)
        

    def _applyCurCfg(self):
        # Retry state apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.ctrl.configurator.apply(self.cfg, 0.5) # type: ignore
            if status.is_ok():
                self.configSuccess = True
                break

        self.disconFault.set(not self.configSuccess)

    def setFollow(self, leaderCanID:int, invert:bool=False)->None:
        assert False, "Not Implemented"

    def setInverted(self, isInverted:bool)->None:
        if(isInverted):
            self.cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.cfg.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self._applyCurCfg()


    def setPID(self, kP:float, kI:float, kD:float)->None:
        self.cfg.slot0.k_p = kP
        self.cfg.slot0.k_i = kI
        self.cfg.slot0.k_d = kD
        self._applyCurCfg()

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            posCmdRad (float): motor desired shaft rotations in radians
            arbFF (float, optional): _description_. Defaults to 0.
        """
        self.simActPos = posCmdRad
        self.desPos = posCmdRad
        self.desVolt = arbFF
        posCmdRev = rad2Rev(posCmdRad)
        #if(not CTREMusicPlayback().isPlaying()):
        self.ctrl.set_control(controls.PositionVoltage(posCmdRev).with_slot(0).with_feed_forward(arbFF))
        self.motorSupplyCurrent.refresh()
        self.controlState = MotorControlStates.POSITION

    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            velCmdRadps (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """
        velCmdRPM = radPerSec2RPM(velCmdRadps)
        self.desVel = velCmdRPM
        self.desVolt = arbFF
        velCmdRotPS = velCmdRPM/60.0
        #if(not CTREMusicPlayback().isPlaying()):
        self.ctrl.set_control(controls.VelocityVoltage(velCmdRotPS).with_slot(0).with_feed_forward(arbFF))
        self.motorSupplyCurrent.refresh()

    def setVoltage(self, outputVoltageVolts:float)->None:
        self.desVolt = outputVoltageVolts
        #if(not CTREMusicPlayback().isPlaying()):
        self.ctrl.set_control(controls.VoltageOut(outputVoltageVolts))
        self.motorSupplyCurrent.refresh()
        self.controlState = MotorControlStates.VOLTAGE

    def getMotorPositionRad(self)->float:
        if(TimedRobot.isSimulation()):
            pos = self.simActPos
        else:
            if self.configSuccess:
                self.motorPosSig.refresh()
                pos = rev2Rad(self.motorPosSig.value_as_double)
            else:
                pos = 0
        return pos

    def getMotorVelocityRadPerSec(self)->float:
        if self.configSuccess:
            self.motorVelSig.refresh()
            velRpm = self.motorVelSig.value_as_double*60.0
        else:
            velRpm = 0
        return RPM2RadPerSec(velRpm)

    def getAppliedOutput(self)->float:
        if self.configSuccess:
            self.motorAppliedVoltage.refresh()
            voltage = self.motorAppliedVoltage.value_as_double
        else:
            voltage = 0
        return voltage

    def getCurrentLimitA(self)->int:
        return int(self.currentLimitA)

    def getControlState(self)->MotorControlStates:
        return self.controlState

    def setSmartCurrentLimit(self, currentLimitA: int)->None:
        assert False, "Not implemented"

    def getOutputTorqueCurrentA(self)->float:
        if self.configSuccess:
            self.motorTorqueCurrent.refresh()
            current = self.motorTorqueCurrent.value_as_double
        else:
            current = 0.0
        return current
