# Use the local WrapperedSparkMax defined below in this file to avoid import/type conflicts
import math
from rev import SparkMax, SparkMaxConfig, SparkBaseConfig
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.constants import CLIMBER_ARM_CANID, CLIMBER_PIVOT_CANID
WHEEL_GEAR_RATIO_L1 = 8.41
WHEEL_GEAR_RATIO_L2 = 6.75
WHEEL_GEAR_RATIO_L3 = 6.12
AZMTH_GEAR_RATIO = 12.8
class Climber:
    def Init(self):
        """Runs once when the robot starts."""
        self.arm_motor = WrapperedSparkMax(CLIMBER_ARM_CANID, "ArmMotor", brakeMode=True)
        self.pivot_motor = WrapperedSparkMax(CLIMBER_PIVOT_CANID, "PivotMotor", brakeMode=True)
        self.is_moving = False
        self.move_duration = 0
        self.speed = 0
        self.distance = 0
    def setSpeed(self, newSpeed):
        self.speed = newSpeed

    def getSpeed(self):
        return self.speed
    def getDistance(self):
        return self.distance
   
    def setDistance(self, newDistance):
        self.distance = newDistance
    def startClimb(self):
        if (self.arm_motor.getMotorPositionRad() < self.distance and self.speed >(0)) \
        or (self.arm_motor.getMotorPositionRad() > self.distance and self.speed <(0)):
        
            self.pivot_motor.setPosCmd(0)
                 #TODO: retract intake.
            self.arm_motor.setVelCmd(0)
            self.arm_motor.getMotorPositionRad()