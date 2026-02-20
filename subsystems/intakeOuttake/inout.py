from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton

class InOutDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.inOutDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_INOUT": False,
            },
            RobotTypes.Spires2025: {
                "HAS_INOUT": False,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_INOUT": True,
                "GROUND_MOTOR_CANID": 17,
                "HOPPER_MOTOR_CANID": 11,
                "FLYWHEEL_MOTOR_CANID": 13,
                "GROUND_MOTOR_INVERTED": False,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 16.0 / 32.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.000_4,
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.5, # volts
                "GROUND_KV": 0.014, # volts/radPerSec
                "GROUND_KA": 0.000, # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_1,
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.5, #
                "HOPPER_KV": 0.01, # volts/radPerSec
                "HOPPER_KA": 0.000, # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_1,
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.5, # volts
                "FLYWHEEL_KV": 0.03, # volts/radPerSec
                "FLYWHEEL_KA": 0.000, # volts/radPerSecPerSec
                "GROUND_INTAKE_SPEED_IPS": 40.0, # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 20.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0, # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 20.0, # in/sec
                "GROUND_SHOOT_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 20.0, # in/sec
                "FLYWHEEL_SPEED_IPS": 100.0, # in/sec
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_INOUT": True,
                "GROUND_MOTOR_CANID": 17,
                "HOPPER_MOTOR_CANID": 11,
                "FLYWHEEL_MOTOR_CANID": 13,
                "GROUND_MOTOR_INVERTED": False,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 16.0 / 32.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.000_4,
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.5,  # volts
                "GROUND_KV": 0.014,  # volts/radPerSec
                "GROUND_KA": 0.000,  # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_1,
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.5,  #
                "HOPPER_KV": 0.01,  # volts/radPerSec
                "HOPPER_KA": 0.000,  # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_1,
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.5,  # volts
                "FLYWHEEL_KV": 0.03,  # volts/radPerSec
                "FLYWHEEL_KA": 0.000,  # volts/radPerSecPerSec
                """
                TODO What didn't work it was too reactive to noise.
                TODO I think I forgot to change the units from Radps to RPM for 
                "GROUND_KP": 0.000_04,  # 0.000_291_42, # Max Velocity Error rad/s: 144.97, Max Control Effort: 7V
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.281_67,  # volts
                "GROUND_KV": 0.020_209,  # volts/radPerSec
                "GROUND_KA": 0.000_5,  # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_78,
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.128_9,  #
                "HOPPER_KV": 0.019_315,  # volts/radPerSec
                "HOPPER_KA": 0.002_025_1,  # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.01,
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.070_225,  # volts
                "FLYWHEEL_KV": 0.016_29,  # volts/radPerSec
                "FLYWHEEL_KA": 0.006_769_98,  # volts/radPerSecPerSec
                """
                "GROUND_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "FLYWHEEL_SPEED_IPS": 500.0,  # in/sec
        },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_INOUT": False,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_INOUT": False,
            },
        }
    def get(self, robotType: RobotTypes):
        return self.inOutDepConstants[robotType]
