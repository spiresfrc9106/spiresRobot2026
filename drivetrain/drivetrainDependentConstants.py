
from ntcore import NetworkTableInstance
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d


from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor

# Camera Mount Offsets
# These are relative to the robot origin
# which is in the center of the chassis on the ground
ROBOT_TO_LEFT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(-10), inchesToMeters(-12), inchesToMeters(22)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -10, 90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_RIGHT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(-10), inchesToMeters(12), inchesToMeters(22)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -10, -90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_FRONT_CAM = Transform3d(
    Translation3d(
        #inchesToMeters(-7.5), inchesToMeters(+11), inchesToMeters(13)  # X  # Y  # Z
        inchesToMeters(-7.5), inchesToMeters(11), inchesToMeters(13)  # X  # Y  # Z
),
    Rotation3d.fromDegrees(0.0, -10, 0.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_LIME_1 = Transform3d(
    Translation3d(
        inchesToMeters(0), inchesToMeters(0), inchesToMeters(0)  # X:0.3556  # Y:(-0.25)-0.00635  # Z:0.2032
    ),
    Rotation3d.fromDegrees(0.0, 0.0, 0.0),  # Roll  # Pitch  # Yaw
)

# if wpilib.RobotBase.isSimulation() == False:
#     ROBOT_TO_LIME_1 = Pose3d(Translation3d(0,0,0), Rotation3d(Rotation2d(0)))


class CameraDependentConstants(metaclass=Singleton):
    def __init__(self):
        self.limelightPipelineConstants = {
            RobotTypes.Spires2023: {
                "LL_PIPELINE": 2,
            },
            RobotTypes.Spires2025: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.Spires2025Sim: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.SpiresTestBoard: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.SpiresRoboRioV1: {
                "LL_PIPELINE": 1,
            },
            RobotTypes.Spires2026Sim: {
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
                "CAM": wrapperedLimilightCameraFactory("limelight-fl", ROBOT_TO_LIME_1,
                                                       self.cameraDepConstants.get(robotType)['LL_PIPELINE']),
                "POSE_EST_LOG_NAME": "limeli-fl",
                "PUBLISHER":
                    (
                        NetworkTableInstance.getDefault()
                        .getStructTopic("/Limili-flPose", Pose3d)
                        .publish()
                    ),
                "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
                "WEIGH_IN_FILTER": True,
                "USE_IN_TC_FRONT": True,
                "USE_IN_TC_BACK": False,
            },
            {
                "CAM": wrapperedLimilightCameraFactory("limelight-fr", ROBOT_TO_LIME_1,
                                                       self.cameraDepConstants.get(robotType)['LL_PIPELINE']),
                "POSE_EST_LOG_NAME": "limeli-fr",
                "PUBLISHER":
                    (
                        NetworkTableInstance.getDefault()
                        .getStructTopic("/Limili-frPose", Pose3d)
                        .publish()
                    ),
                "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
                "WEIGH_IN_FILTER": True,
                "USE_IN_TC_FRONT": True,
                "USE_IN_TC_BACK": False,
            },
        ]
        return COMMON_CAMS

    def getDivetrainConstants(self, robotType: RobotTypes):
        drivetrainConstants = {
            RobotTypes.Spires2023: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkMax,
                "SWERVE_WHEEL_GEAR_RATIO": 5.50,   # Base Low
                #"SWERVE_WHEEL_GEAR_RATIO": 5.08,  # Base Medium
                #"SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 16.5,
                "LENGTH": 26.5,
                "MASS_LBS": 32, #changed because coach jeremy lifted the robot and felt it was that
                "FL_OFFSET_DEG": 169.2+90+180,
                "FR_OFFSET_DEG": -49.7,
                "BL_OFFSET_DEG": -56.2+180,
                "BR_OFFSET_DEG": -11.2-90+180,
                "GYRO": "NAVX", # "NAVX", # "ADIS16470_IMU",
                "CAMS": self.getCommonCams(RobotTypes.Spires2023),
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 3,
            },
            RobotTypes.Spires2025: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.VORTEX_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 26.0,
                "LENGTH": 33.0,
                "MASS_LBS": 104,
                "FL_OFFSET_DEG": 177.4-90+180-4,
                "FR_OFFSET_DEG": 0.7-123,
                "BL_OFFSET_DEG": 125.4-180-2,
                "BR_OFFSET_DEG": 117.5-90-180-155,
                "GYRO": "ADIS16470_IMU",
                "CAMS": self.getCommonCams(RobotTypes.Spires2025),
                "USE_PHOTON_NAV": True,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.VORTEX_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "CAMS": self.getCommonCams(RobotTypes.Spires2025Sim),
                "GYRO": "ADIS16470_IMU",
                "USE_PHOTON_NAV": True,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_DRIVETRAIN": False,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkMax,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "GYRO": "NoGyro",
                "CAMS": [],
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_DRIVETRAIN": False,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkMax,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.NEO_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "GYRO": "NoGyro",
                "CAMS": [],
                "USE_PHOTON_NAV": False,
                "SPEED_MULTIPLIER": 2,
            },
            RobotTypes.Spires2026Sim: {
                "HAS_DRIVETRAIN": True,
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMotor.makeSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RADPS": WrapperedSparkMotor.VORTEX_CONFIGURED_FREESPEED_RADPS,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "CAMS": self.getCommonCams(RobotTypes.SpiresTestBoard),
                "GYRO": "ADIS16470_IMU",
                "USE_PHOTON_NAV": True,
                "SPEED_MULTIPLIER": 2,
            },
        }
        return drivetrainConstants

    def get(self, robotType: RobotTypes):
        return self.getDivetrainConstants(robotType)[robotType]




