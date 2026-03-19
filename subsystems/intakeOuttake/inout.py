from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton

class InOutDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.inOutDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_INOUT": False,
            },
            RobotTypes.Spires2026: {
                "HAS_INOUT": True,
                "HAS_AGITATOR": False,
                "GROUND_MOTOR_CANID": 16,
                "HOPPER_MOTOR_CANID": 15,
                "FLYWHEEL_MOTOR_CANID": 13,
                "AGITATOR_MOTOR_CANID": 0,
                "GROUND_MOTOR_INVERTED": False,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "AGITATOR_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 10.0 / 84.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "AGITATOR_GEAR_REDUCTION": 5.0,
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.000_004_375,  # at 155.19 Max Vel Error rad/s
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.146_17,  # volts
                "GROUND_KV": 0.019_095,  # volts/radPerSec
                "GROUND_KA": 0.001_785_2,  # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_001_417,  # at 152.55 Max Vel Error rad/s
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.137_53,  #
                "HOPPER_KV": 0.019_441,  # volts/radPerSec
                "HOPPER_KA": 0.001_512,  # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_021_832,  # at 192 Max Velocity Error
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.001,  # volts
                "FLYWHEEL_KV": 1.0125 * 1.116 * 0.015_628,  # 0.015_628,  # volts/radPerSec
                "FLYWHEEL_KA": 1.22 * 0.014_731,  # volts/radPerSecPerSec
                "AGITATOR_KP": 0.000_001,
                "AGITATOR_KD": 0.0,
                "AGITATOR_KS": 0.0,  # volts
                "AGITATOR_KV": 0.0,  # volts/radPerSec
                "AGITATOR_KA": 0.0,  # volts/radPerSecPerSec
                "GROUND_MAX_MOTION_MAX_ACC_IPS2": 120.0,  # in/sec^2
                "HOPPER_MAX_MOTION_MAX_ACC_IPS2": 120.0,  # in/sec^2
                "FLYWHEEL_MAX_MOTION_MAX_ACC_IPS2": 600.0,  # in/sec^2
                "AGITATOR_MAX_MOTION_MAX_ACC_HZPS": 10.0,  # Hz/sec^2, TBD
                "GROUND_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "FLYWHEEL_SPEED_IPS": 650.0,  # in/sec
                "AGITATOR_INTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_OUTTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_SHOOT_SPEED_HZ": 10.0,  # Hz, TBD
            },
            RobotTypes.Spires2026Sim: {
                "HAS_INOUT": True,
                "HAS_AGITATOR": True,
                "GROUND_MOTOR_CANID": 17,
                "HOPPER_MOTOR_CANID": 11,
                "FLYWHEEL_MOTOR_CANID": 13,
                "AGITATOR_MOTOR_CANID": 14,  # TBD
                "GROUND_MOTOR_INVERTED": False,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "AGITATOR_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 16.0 / 32.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "AGITATOR_GEAR_REDUCTION": 5.0,  # TBD
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.001_2, #0.000_600, #2.4592E-05, # 0.000_4,
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.024329, #0.024329, #0.024329, # 0.5, # volts
                "GROUND_KV": 0.019456, #0.019456, # 0.014, # volts/radPerSec
                "GROUND_KA": 0.0, #0.077624, #0.077624, # 0.0077624, # 0.000, # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_02, #0.000_006, #0.000_02, #0.000_1,
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.001, #0.5, #
                "HOPPER_KV": 0.020, #0.017, #0.01, # volts/radPerSec
                "HOPPER_KA": 0.001_002_5, #0.000, # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_1,
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.5, # volts
                "FLYWHEEL_KV": 0.03, # volts/radPerSec
                "FLYWHEEL_KA": 0.000, # volts/radPerSecPerSec
                "AGITATOR_KP": 0.000_001,
                "AGITATOR_KD": 0.0,
                "AGITATOR_KS": 0.0,  # volts
                "AGITATOR_KV": 0.0,  # volts/radPerSec
                "AGITATOR_KA": 0.0,  # volts/radPerSecPerSec
                "GROUND_MAX_MOTION_MAX_ACC_IPS2": 40.0,  # in/sec^2
                "HOPPER_MAX_MOTION_MAX_ACC_IPS2": 625.0,  # in/sec^2
                "FLYWHEEL_MAX_MOTION_MAX_ACC_IPS2": 100.0,  # in/sec^2
                "AGITATOR_MAX_MOTION_MAX_ACC_HZPS": 10.0,  # Hz/sec^2, TBD
                "GROUND_INTAKE_SPEED_IPS": 40.0, # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0, # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 20.0, # in/sec
                "GROUND_SHOOT_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 20.0, # in/sec
                "FLYWHEEL_SPEED_IPS": 650.0, # in/sec
                "AGITATOR_INTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_OUTTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_SHOOT_SPEED_HZ": 10.0,  # Hz, TBD
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_INOUT": True,
                "HAS_AGITATOR": False,
                "GROUND_MOTOR_CANID": 11,
                "HOPPER_MOTOR_CANID": 17,
                "FLYWHEEL_MOTOR_CANID": 13,
                "AGITATOR_MOTOR_CANID": 0,
                "GROUND_MOTOR_INVERTED": False,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "AGITATOR_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 10.0 / 84.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "AGITATOR_GEAR_REDUCTION": 5.0,
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.000_004_375,  # at 155.19 Max Vel Error rad/s
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.146_17,  # volts
                "GROUND_KV": 0.019_095,  # volts/radPerSec
                "GROUND_KA": 0.001_785_2,  # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_001_417,  # at 152.55 Max Vel Error rad/s
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.137_53,  #
                "HOPPER_KV": 0.019_441,  # volts/radPerSec
                "HOPPER_KA": 0.001_512,  # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_021_832,  # at 192 Max Velocity Error
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.001,  # volts
                "FLYWHEEL_KV": 1.0125 * 1.116 * 0.015_628,  # 0.015_628,  # volts/radPerSec
                "FLYWHEEL_KA": 1.22 * 0.014_731,  # volts/radPerSecPerSec
                "AGITATOR_KP": 0.000_001,
                "AGITATOR_KD": 0.0,
                "AGITATOR_KS": 0.0,  # volts
                "AGITATOR_KV": 0.0,  # volts/radPerSec
                "AGITATOR_KA": 0.0,  # volts/radPerSecPerSec
                "GROUND_MAX_MOTION_MAX_ACC_IPS2": 120.0,  # in/sec^2
                "HOPPER_MAX_MOTION_MAX_ACC_IPS2": 120.0,  # in/sec^2
                "FLYWHEEL_MAX_MOTION_MAX_ACC_IPS2": 600.0,  # in/sec^2
                "AGITATOR_MAX_MOTION_MAX_ACC_HZPS": 10.0,  # Hz/sec^2, TBD
                "GROUND_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "FLYWHEEL_SPEED_IPS": 650.0,  # in/sec
                "AGITATOR_INTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_OUTTAKE_SPEED_HZ": 10.0,  # Hz, TBD
                "AGITATOR_SHOOT_SPEED_HZ": 10.0,  # Hz, TBD
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_INOUT": True,
                "HAS_AGITATOR": False,
                "GROUND_MOTOR_CANID": 16,
                "HOPPER_MOTOR_CANID": 15,
                "FLYWHEEL_MOTOR_CANID": 18,
                "AGITATOR_MOTOR_CANID": 0,  # TBD
                "GROUND_MOTOR_INVERTED": True,
                "HOPPER_MOTOR_INVERTED": False,
                "FLYWHEEL_MOTOR_INVERTED": False,
                "AGITATOR_MOTOR_INVERTED": False,
                "GROUND_GEAR_REDUCTION": 10.0 / 84.0,
                "HOPPER_GEAR_REDUCTION": 10.0 / 84.0,
                "FLYWHEEL_GEAR_REDUCTION": 1.0,
                "AGITATOR_GEAR_REDUCTION": 1.0,  # TBD
                "GROUND_WHEEL_DIAMETER_INCHES": 2.0,
                "HOPPER_WHEEL_DIAMETER_INCHES": 2.0,
                "FLYWHEEL_WHEEL_DIAMETER_INCHES": 4.0,
                "GROUND_KP": 0.000_004_375, #at 155.19 Max Vel Error rad/s
                "GROUND_KD": 0.0,
                "GROUND_KS": 0.146_17,  # volts
                "GROUND_KV": 0.019_095,  # volts/radPerSec
                "GROUND_KA": 0.001_785_2,  # volts/radPerSecPerSec
                "HOPPER_KP": 0.000_001_417, # at 152.55 Max Vel Error rad/s
                "HOPPER_KD": 0.0,
                "HOPPER_KS": 0.137_53,  #
                "HOPPER_KV": 0.019_441,  # volts/radPerSec
                "HOPPER_KA": 0.001_512,  # volts/radPerSecPerSec
                "FLYWHEEL_KP": 0.000_021_832, #at 192 Max Velocity Error
                "FLYWHEEL_KD": 0.0,
                "FLYWHEEL_KS": 0.001,  # volts
                "FLYWHEEL_KV": 1.0125*1.116*0.015_628, #0.015_628,  # volts/radPerSec
                "FLYWHEEL_KA": 1.22*0.014_731,  # volts/radPerSecPerSec
                "AGITATOR_KP": 0.000_001,
                "AGITATOR_KD": 0.0,
                "AGITATOR_KS": 0.0,  # volts
                "AGITATOR_KV": 0.0,  # volts/radPerSec
                "AGITATOR_KA": 0.0,  # volts/radPerSecPerSec
                "GROUND_MAX_MOTION_MAX_ACC_IPS2": 120.0, # in/sec^2
                "HOPPER_MAX_MOTION_MAX_ACC_IPS2": 120.0, # in/sec^2
                "FLYWHEEL_MAX_MOTION_MAX_ACC_IPS2": 600.0, # in/sec^2
                "AGITATOR_MAX_MOTION_MAX_ACC_HZPS": 10.0,  # Hz/sec^2, TBD
                "GROUND_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 80.0,  # in/sec
                "FLYWHEEL_SPEED_IPS": 500.0,  # in/sec
                "AGITATOR_INTAKE_SPEED_HZ": 1.0,  # Hz, TBD
                "AGITATOR_OUTTAKE_SPEED_HZ": 1.0,  # Hz, TBD
                "AGITATOR_SHOOT_SPEED_HZ": 1.0,  # Hz, TBD
            },
        }
    def get(self, robotType: RobotTypes):
        return self.inOutDepConstants[robotType]

