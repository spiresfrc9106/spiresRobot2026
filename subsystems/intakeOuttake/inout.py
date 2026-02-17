from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d, Transform3d
from wpimath.system.plant import DCMotor

from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton

kTurretMinAngle = Rotation2d.fromDegrees(-160)
kTurretMaxAngle = Rotation2d.fromDegrees(160)

kTurretGearRatio = (1 / 5) * (13 / 360)

kTurretMaxVelocity = Rotation2d.fromDegrees(180)  # per second
kTurretMaxAcceleration = Rotation2d.fromDegrees(360)  # per second squared
kTurretTolerance = Rotation2d.fromDegrees(1.0)

kTurretCanId = 22
kTurretPGain = 0.1
kTurretIGain = 0.0
kTurretDGain = 0.0
kTurretSGain = 0.0
kTurretVGain = 0.0
kTurretAGain = 0.0

kTurretCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kTurretSimMotor = DCMotor.falcon500(1)
kTurretSimInertia = 0.005  # kg m^2

kTurretLocation = Transform3d(
    Translation3d(0.25, 0.25, 0.25), Rotation3d()
)  # FILL IN LATER
"""Transform between turrent center and center of robot"""

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
                "GROUND_OUTTAKE_SPEED_IPS": 40.0, # in/sec
                "GROUND_SHOOT_SPEED_IPS": 40.0, # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 20.0, # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 20.0, # in/sec
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
                "GROUND_INTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_OUTTAKE_SPEED_IPS": 40.0,  # in/sec
                "GROUND_SHOOT_SPEED_IPS": 40.0,  # in/sec
                "HOPPER_INTAKE_SPEED_IPS": 20.0,  # in/sec
                "HOPPER_OUTTAKE_SPEED_IPS": 20.0,  # in/sec
                "HOPPER_SHOOT_SPEED_IPS": 20.0,  # in/sec
                "FLYWHEEL_SPEED_IPS": 100.0,  # in/sec
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
