from utils.singleton import Singleton
from utils.constants import INTAKE_CONTROL_CANID, INTAKE_WHEELS_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class IntakeController(metaclass=Singleton):

    def __init__(self):

        self.intakeEnabled = False
        self.intakeMotor = WrapperedSparkMax("intake_Motor",INTAKE_CONTROL_CANID)
        self.intakeWheelsMotor = WrapperedSparkMax("intake_Wheels_Motor",INTAKE_WHEELS_CANID)
        pass

    def update(self):
        pass 

    def enableIntake(self):
        self.intakeEnabled = True
        self.intakeWheelsMotor.setVelCmd(8)
        
    def lowerIntake(self):
        self.intakeMotor.setPosCmd(8)

    def disableIntake(self):
        self.intakeEnabled = False
        self.intakeWheelsMotor.setVelCmd(0)
    def raiseIntake(self):
        self.intakeMotor.setPosCmd(0)
   
   
    def getIntakeState(self):
        return self.intakeEnabled 
    
   
        
    
       
    