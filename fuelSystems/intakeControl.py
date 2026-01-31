from fuelSystems.fuelSystemConstants import ALGAE_ANGLE_ABS_POS_ENC_OFFSET, IntakeWristState
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID, ALGAE_WRIST_CANID, ALGAE_ENC_PORT
from utils.units import deg2Rad, rad2Deg
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder

class IntakeController(metaclass=Singleton):

    def __init__(self):

        self.intakeEnabled = False
        self.intakeMotor = WrapperedSparkMax("intake_Motor",INTAKE_CONTROL_CANID)
        self.intakeWheelsMotor = WrapperedSparkMax("intake_Wheels_Motor",INTAKE_WHEELS_CANID)
        pass

    def update(self):
      if self.intakeEnabled:
          self.intakeWheelsMotor.setVoltage(8) 

    def enableIntake(self):
        self.intakeEnabled = True
       
    def lowerIntake(self):
        self.intakeMotor.setPosCmd(8)

    def disableIntake(self):
        self.intakeEnabled = False
        
    def raiseIntake(self):
        self.intakeMotor.setPosCmd(0)
   
   
    def getIntakeState(self):
        return self.intakeEnabled 
    
    ##this is done############################################################################


class intakeWristControl(metaclass=Singleton):
    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height

        #motor and encoder
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "Intake_Wrist_Motor", brakeMode=True, currentLimitA=20.0)
        self.intakeAbsEnc = WrapperedThroughBoreHexEncoder(port=ALGAE_ENC_PORT, name="Intake_Wrist_enc", mountOffsetRad=deg2Rad(ALGAE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)

        #PID stuff calibrations
        self.kP = Calibration(name="Intake Wrist kP", default=.6, units="V/degErr")
        self.maxV = Calibration(name="Intake Wrist maxV", default=6.0, units="V")
        self.deadzone = Calibration(name="Intake Wrist deadzone", default=4.0, units="deg")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.intakeOffGroundPos = Calibration(name="Intake Wrist Intake Off Ground Position", default = -20, units="deg")
        self.stowPos = Calibration(name="Intake Wrist Stow Position", default = 95, units="deg")
       
        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.pos = IntakeWristState.NOTHING
      

        addLog("Intake Wrist Desired Angle",lambda: self.curPosCmdDeg, "deg")
        addLog("Intake Wrist Actual Angle", lambda: rad2Deg(self.getAngleRad()), "deg")

    def setDesPos(self, desState : IntakeWristState):
        #this is called in teleop periodic or autonomous to set the desired pos of intake wrist
        self.curPosCmdDeg = self._posToDegrees(desState)

    def getAngleRad(self):
        return deg2Rad(self.intakeOffGroundPos.get())

    # Might optimize to accept 1 enum parameter for new position
    def _posToDegrees(self,pos:IntakeWristState) -> float:
        self.pos = pos
        if (pos == IntakeWristState.INTAKEOFFGROUND):
            return self.intakeOffGroundPos.get()
        else:
            return self.stowPos.get()

    def update(self):

        self.intakeAbsEnc.update()
        self.actualPos = rad2Deg(self.getAngleRad())

        if(self.intakeAbsEnc.isFaulted()):
            vCmd = 0.0 # faulted, so stop
        else:
            # Limited-output P control with deadzone
            err = self.curPosCmdDeg - self.actualPos
            if(abs(err) <= self.deadzone.get()):
                # in deadzone, no command
                vCmd = 0
            elif self.pos == IntakeWristState.NOTHING:
                # No command, so keep voltage at zero
                vCmd = 0
            else:
                # Command and outside deadzone
                # P control with limit
                
                # Adjust error so that it's offset by the deadzone
                if(err>0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                vCmd = self.kP.get() * err
                vCmd = min(self.maxV.get(), max(-self.maxV.get(), vCmd))

        self.wristMotor.setVoltage(vCmd)
    
   
        
    
       
    