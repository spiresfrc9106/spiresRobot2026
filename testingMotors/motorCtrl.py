# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.


from pykit.logger import Logger
from utils.singleton import Singleton
from subsystems.state.configio import RobotTypes
from subsystems.state.configsubsystem import ConfigSubsystem
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor

class MotorDependentConstants:
    def __init__(self):
        self.motorDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
            RobotTypes.Spires2026: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_MOTOR_TEST": True,
                "TEST_MOTOR_CANID": 23,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
        },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
        }

    def get(self):
        return self.motorDepConstants[ConfigSubsystem().getRobotType()]

motorDepConstants = MotorDependentConstants().get()


TEST_MOTOR_CANID = motorDepConstants['TEST_MOTOR_CANID']

class MotorControl(metaclass=Singleton):
    def __init__(self):

        # Elevator Motors
        self.Rmotor = WrapperedSparkMotor.makeSparkMax(TEST_MOTOR_CANID, "Test_Motor", brakeMode=False, currentLimitA=20)

        # Set P gain on motor
        self.Rmotor.setPID(0.00005, 0.0, 0.0)


    def update(self, desiredSpeedRpm):
        if desiredSpeedRpm!=0:
            #rpm -> rps
            #rpm to rad per minute *2*3.14/60
            self.Rmotor.setVelCmd(desiredSpeedRpm *2*3.14/60,0)
        else:
            self.Rmotor.setVelCmd(0,0)
            self.Rmotor.setVoltage(0.0)
        self.Rmotor.getMotorVelocityRadPerSec()
        self.Rmotor.getAppliedOutput()
        Logger.recordOutput("Test_Motor_DesRpm_rpm",desiredSpeedRpm)



