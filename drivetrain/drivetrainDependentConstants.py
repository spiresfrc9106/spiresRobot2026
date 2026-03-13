
from ntcore import NetworkTableInstance
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d


from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton
from utils.units import deg2Rad
from subsystems.vision.vision import kRobotToBackCenterCamTransform
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera
from wrappers.wrapperedSparkFlex import WrapperedSparkFlex
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor



class CameraDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.limelightPipelineConstants = {
            RobotTypes.Spires2023: {
                "LL_PIPELINE": 2,
            },
            RobotTypes.Spires2026: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.Spires2026Sim: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.SpiresTestBoard: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.SpiresRoboRioV1: {
                "LL_PIPELINE": 1,
            },
        }

    def get(self, robotType: RobotTypes):
        return self.limelightPipelineConstants[robotType]


class DrivetrainDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.useCasseroleSwerve = True
        self.useWestwoodSwerve = False
        self.cameraDepConstants = CameraDependentConstants()
        print(f"DrivetrainDependentConstants __init__ self={id(self)}")

    def getCommonCams(self, robotType: RobotTypes):
        COMMON_CAMS = [
            {
                "CAM": WrapperedPoseEstPhotonCamera("back_center_cam", kRobotToBackCenterCamTransform),
                "POSE_EST_LOG_NAME": "photonBC",
                "PUBLISHER":
                    (
                        NetworkTableInstance.getDefault()
                        .getStructTopic("/BackCenterCamPose", Pose3d)
                        .publish()
                    ),
                "ROBOT_TO_CAM": kRobotToBackCenterCamTransform,
                "WEIGH_IN_FILTER": True,
                "USE_IN_TC_FRONT": True,
                "USE_IN_TC_BACK": True,
            },
        ]
        return COMMON_CAMS

    def getDivetrainConstants(self, robotType: RobotTypes):
        drivetrainConstants = {
            RobotTypes.Spires2023: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
                "SWERVE_WHEEL_GEAR_RATIO": 5.50,   # Base Low
                #"SWERVE_WHEEL_GEAR_RATIO": 5.08,  # Base Medium
                #"SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 24.5,
                "LENGTH": 22.5,
                "MASS_LBS": 32, #changed because coach jeremy lifted the robot and felt it was that
                "WHEEL_P": 0.000_1,
                "WHEEL_I": 0.0,
                "WHEEL_D": 0.0,
                "WHEEL_A": 0.0,
                "WHEEL_V": 0.0,
                "WHEEL_S": 0.5,
                "AZMTH_P": 0.03,
                "AZMTH_I": 0.0,
                "AZMTH_D": 0.0,
                "AZMTH_A": 0.0,
                "AZMTH_V": 0.0,
                "AZMTH_S": 0.0,
                "FL_OFFSET_RAD": deg2Rad(169.2+90+180),
                "FR_OFFSET_RAD": deg2Rad(-49.7),
                "BL_OFFSET_RAD": deg2Rad(-56.2+180),
                "BR_OFFSET_RAD": deg2Rad(-11.2-90+180),
                "GYRO": "NAVX", # "NAVX", # "ADIS16470_IMU",
                "CAMS": self.getCommonCams(RobotTypes.Spires2023),
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 3,
            },
            RobotTypes.Spires2026: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.VORTEX_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 24.5,
                "LENGTH": 22.5,
                "MASS_LBS": 100.0,
                "WHEEL_P": 0.000_04,
                "WHEEL_I": 0.0,
                "WHEEL_D": 0.0,
                "WHEEL_A": 0.0,
                "WHEEL_V": 0.0,
                "WHEEL_S": 0.2,
                "AZMTH_P": 0.12,
                "AZMTH_I": 0.0,
                "AZMTH_D": 0.0,
                "AZMTH_A": 0.0,
                "AZMTH_V": 0.0,
                "AZMTH_S": 0.0,
                "FL_OFFSET_RAD": deg2Rad( 173.4),
                "FR_OFFSET_RAD": deg2Rad(-122.3),
                "BL_OFFSET_RAD": deg2Rad( 33.4),
                "BR_OFFSET_RAD": deg2Rad(142.5),
                "GYRO": "ADIS16470_IMU",
                "CAMS": self.getCommonCams(RobotTypes.Spires2026),
                "USE_PHOTON_NAV": True,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.VORTEX_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 24.5,
                "LENGTH": 22.5,
                "MASS_LBS": 60,
                "WHEEL_P": 0.000_04,
                "WHEEL_I": 0.0,
                "WHEEL_D": 0.0,
                "WHEEL_A": 0.0,
                "WHEEL_V": 0.0,
                "WHEEL_S": 0.0,
                "AZMTH_P": 0.03,
                "AZMTH_I": 0.0,
                "AZMTH_D": 0.0,
                "AZMTH_A": 0.0,
                "AZMTH_V": 0.0,
                "AZMTH_S": 0.0,
                "FL_OFFSET_RAD": deg2Rad(0.0),
                "FR_OFFSET_RAD": deg2Rad(0.0),
                "BL_OFFSET_RAD": deg2Rad(0.0),
                "BR_OFFSET_RAD": deg2Rad(0.0),
                "CAMS": self.getCommonCams(RobotTypes.Spires2026Sim),
                "GYRO": "ADIS16470_IMU",
                "USE_PHOTON_NAV": True,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_DRIVETRAIN": False,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 24.5,
                "LENGTH": 22.5,
                "MASS_LBS": 60,
                "WHEEL_P": 0.000_04,
                "WHEEL_I": 0.0,
                "WHEEL_D": 0.0,
                "WHEEL_A": 0.0,
                "WHEEL_V": 0.0,
                "WHEEL_S": 0.0,
                "AZMTH_P": 0.03,
                "AZMTH_I": 0.0,
                "AZMTH_D": 0.0,
                "AZMTH_A": 0.0,
                "AZMTH_V": 0.0,
                "AZMTH_S": 0.0,
                "FL_OFFSET_RAD": deg2Rad(0.0),
                "FR_OFFSET_RAD": deg2Rad(0.0),
                "BL_OFFSET_RAD": deg2Rad(0.0),
                "BR_OFFSET_RAD": deg2Rad(0.0),
                "GYRO": "NoGyro",
                "CAMS": [],
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_DRIVETRAIN": False,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 24.5,
                "LENGTH": 22.5,
                "MASS_LBS": 60,
                "WHEEL_P": 0.000_04,
                "WHEEL_I": 0.0,
                "WHEEL_D": 0.0,
                "WHEEL_A": 0.0,
                "WHEEL_V": 0.0,
                "WHEEL_S": 0.0,
                "AZMTH_P": 0.03,
                "AZMTH_I": 0.0,
                "AZMTH_D": 0.0,
                "AZMTH_A": 0.0,
                "AZMTH_V": 0.0,
                "AZMTH_S": 0.0,
                "FL_OFFSET_RAD": deg2Rad(0.0),
                "FR_OFFSET_RAD": deg2Rad(0.0),
                "BL_OFFSET_RAD": deg2Rad(0.0),
                "BR_OFFSET_RAD": deg2Rad(0.0),
                "GYRO": "NoGyro",
                "CAMS": [],
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 2,
            },
        }
        return drivetrainConstants

    def get(self, robotType: RobotTypes):
        return self.getDivetrainConstants(robotType)[robotType]




