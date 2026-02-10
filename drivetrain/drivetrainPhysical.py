import math
from dataclasses import dataclass, field

from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Translation2d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import SwerveDrive4Kinematics

from utils.units import lbsToKg
from utils.units import deg2Rad
from utils.units import in2m
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera
from sensors.limelight import Limelight
from utils.singleton import Singleton
from subsystems.config.configio import RobotTypes
from subsystems.config.configsubsystem import ConfigSubsystem
from subsystems.config.robottopsubsystem import RobotTopSubsystem

"""
Defines the physical dimensions and characteristics of the drivetrain
"""


# Function make a swerve module azimuth encoder reader object
def wrapperedSwerveDriveAzmthEncoder(azmthEncoderPortIdx, moduleName, azmthOffsetRad, inverted):
    return WrapperedRevThroughBoreEncoder(
        port=azmthEncoderPortIdx,
        name=moduleName,
        mountOffsetRad=azmthOffsetRad,
        dirInverted=inverted
    )

@dataclass
class DrivetrainPhysical(metaclass=Singleton):
    c: ConfigSubsystem = field(default_factory=lambda: ConfigSubsystem())
    WHEEL_BASE_HALF_WIDTH_M: float = field(
        default_factory=lambda:inchesToMeters(ConfigSubsystem().drivetrainDepConstants["WIDTH"] / 2.0))
    WHEEL_BASE_HALF_LENGTH_M: float = field(
        default_factory=lambda: inchesToMeters(ConfigSubsystem().drivetrainDepConstants["LENGTH"] / 2.0))

    def __post_init__(self):
        ###################################################################
        # Physical dimensions and mass distribution

        # Wheel base half width: Distance from the center of the frame rail
        # out to the center of the "contact patch" where the wheel meets the ground

        # Additional distance from the wheel contact patch out to the edge of the bumper
        self.BUMPER_THICKNESS_M = inchesToMeters(2.5)
        
        # Total mass includes robot, battery, and bumpers
        # more than the "weigh-in" weight
        if self.c.isSpiresRobot():
            self.ROBOT_MASS_KG = lbsToKg(ConfigSubsystem().drivetrainDepConstants['MASS_LBS'])
        else:
            self.ROBOT_MASS_KG = lbsToKg(60)
        
        # Model the robot's moment of intertia as a square slab
        # slightly bigger than wheelbase with axis through center
        self.ROBOT_MOI_KGM2 = 1.0 / 12.0 * self.ROBOT_MASS_KG * self.WHEEL_BASE_HALF_WIDTH_M * self.WHEEL_BASE_HALF_LENGTH_M * math.pow(
            2.2,
            2) * 2
        
        # SDS MK4i Swerve Module Ratios
        # See https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
        WHEEL_GEAR_RATIO_L1 = 8.41
        WHEEL_GEAR_RATIO_L2 = 6.75
        WHEEL_GEAR_RATIO_L3 = 6.12 #TODO what is WHEEL_GEAR_RATIO_L3
        AZMTH_GEAR_RATIO = 12.8 # TODO fix me up
        
        ## CHANGE THIS DEPENDING ON WHICH MODULE GEAR RATIO IS INSTALLED
        if self.c.getRobotType() == RobotTypes.Main:
            self.WHEEL_GEAR_RATIO = self.WHEEL_GEAR_RATIO_L3
        elif self.c.getRobotType() == RobotTypes.Practice:
            self.WHEEL_GEAR_RATIO = self.WHEEL_GEAR_RATIO_L2
        elif self.c.isSpiresRobot():
            self.WHEEL_GEAR_RATIO = ConfigSubsystem().drivetrainDepConstants['SWERVE_WHEEL_GEAR_RATIO']
        else:
            self.WHEEL_GEAR_RATIO = self.WHEEL_GEAR_RATIO_L3

        # carpet/roughtop interface fudge factor
        # This accounts for the fact that roughtop tread
        # sinks into the carpet slightly. Determined empirically
        # by driving the robot a known distance, seeing the measured distance in software,
        # and adjusting this factor till the measured distance matches known
        # Might have to be different for colson wheels?
        self.WHEEL_FUDGE_FACTOR = 0.9238*(48+4.5)/48
        
        # Nominal 4-inch diameter swerve drive wheels
        # https:#www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
        self.WHEEL_RADIUS_IN = ConfigSubsystem().drivetrainDepConstants[
                                   "SWERVE_WHEEL_DIAMETER_IN"] / 2.0 * self.WHEEL_FUDGE_FACTOR

        # Drivetrain Performance Mechanical limits
        # Nominal calculations (ideal)
        if self.c.isSpiresRobot():
            self.MAX_DT_MOTOR_SPEED_RPS = ConfigSubsystem().drivetrainDepConstants['SWERVE_WHEEL_MAX_SPEED_RPS']
        else:
            self.MAX_DT_MOTOR_SPEED_RPS = DCMotor.NEO(1).freeSpeed
            # self.MAX_DT_MOTOR_SPEED_RPS = DCMotor.neoVortex(1).freeSpeed
        self.MAX_DT_LINEAR_SPEED_MPS = self.MAX_DT_MOTOR_SPEED_RPS / self.WHEEL_GEAR_RATIO * in2m(self.WHEEL_RADIUS_IN)
        # Fudged max expected performance

        speed_multiple_at_comp = ConfigSubsystem().drivetrainDepConstants['SPEED_MULTIPLIER']

        self.MAX_FWD_REV_SPEED_MPS = self.MAX_DT_LINEAR_SPEED_MPS * 0.98 * speed_multiple_at_comp  # fudge factor due to gearbox losses
        self.MAX_STRAFE_SPEED_MPS = self.MAX_DT_LINEAR_SPEED_MPS * 0.98 * speed_multiple_at_comp  # fudge factor due to gearbox losses
        self.MAX_ROTATE_SPEED_RAD_PER_SEC = deg2Rad(
            360.0
        ) * speed_multiple_at_comp  # Fixed at the maximum rotational speed we'd want.
        # Accelerations - also a total guess
        self.MAX_TRANSLATE_ACCEL_MPS2 = (
            # self.MAX_FWD_REV_SPEED_MPS / 0.50
                self.MAX_FWD_REV_SPEED_MPS / 0.10
        )  # 0-full time of 0.5 second - this is a guestimate xyzzy - investigate making this smaller
        self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = (
                self.MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25
        )  # 0-full time of 0.25 second - this is a guestaimate
        
        
        # Mechanical mounting offsets of the encoder & magnet within the shaft
        # Must be updated whenever the module is reassembled
        # Procedure:
        # 0 - Put the robot up on blocks.
        # 1 - Reset all these values to 0, deploy code
        # 2 - Pull up dashboard with encoder readings (in radians)
        # 3 - Using a square, twist the modules by hand until they are aligned with the robot's chassis
        # 4 - Read out the encoder readings for each module, put them here
        # 5 - Redeploy code, verify that the  encoder readings are correct as each module is manually rotated
        
        
        if self.c.isSpiresRobot():
            # Perhaps we invert the swerve module azimuth motor
            self.INVERT_AZMTH_MOTOR = False
            self.INVERT_AZMTH_ENCODER = True
        
            # Perhaps we invert the swerve module wheel motor drive direction
            self.FL_INVERT_WHEEL_MOTOR = False
            self.FR_INVERT_WHEEL_MOTOR = False
            self.BL_INVERT_WHEEL_MOTOR = False
            self.BR_INVERT_WHEEL_MOTOR = False
        else:
            # Perhaps we invert the swerve module azimuth motor
            self.INVERT_AZMTH_MOTOR = True
            self.INVERT_AZMTH_ENCODER = False
        
            # Perhaps we invert the swerve module wheel motor drive direction
            self.FL_INVERT_WHEEL_MOTOR = True
            self.FR_INVERT_WHEEL_MOTOR = True
            self.BL_INVERT_WHEEL_MOTOR = False
            self.BR_INVERT_WHEEL_MOTOR = False
        
        # todo they moved the configuration closer to the code.
        
        if self.c.getRobotType() == RobotTypes.Main:
            self.FR_ENCODER_MOUNT_OFFSET_RAD = 0.8412
            self.FL_ENCODER_MOUNT_OFFSET_RAD = 0.2412
            self.BR_ENCODER_MOUNT_OFFSET_RAD = 1.259
            self.BL_ENCODER_MOUNT_OFFSET_RAD = 1.777
        elif self.c.isSpiresRobot():
            self.FL_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(ConfigSubsystem().drivetrainDepConstants["FL_OFFSET_DEG"])
            self.FR_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(ConfigSubsystem().drivetrainDepConstants["FR_OFFSET_DEG"])
            self.BL_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(ConfigSubsystem().drivetrainDepConstants["BL_OFFSET_DEG"])
            self.BR_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(ConfigSubsystem().drivetrainDepConstants["BR_OFFSET_DEG"])
        else:
            self.FR_ENCODER_MOUNT_OFFSET_RAD = 0.8412
            self.FL_ENCODER_MOUNT_OFFSET_RAD = 0.2412
            self.BR_ENCODER_MOUNT_OFFSET_RAD = 1.259
            self.BL_ENCODER_MOUNT_OFFSET_RAD = 1.777

        # Module Indices (for ease of array manipulation)
        FL = 0
        FR = 1
        BL = 2
        BR = 3
        

        
        self.WHEEL_MOTOR_WRAPPER = ConfigSubsystem().drivetrainDepConstants["WHEEL_MOTOR_WRAPPER"]

        self.CAMS = ConfigSubsystem().drivetrainDepConstants["CAMS"]

        # Array of translations from robot's origin (center bottom, on floor) to the module's contact patch with the ground
        self.robotToModuleTranslations = []
        self.robotToModuleTranslations.append(
            Translation2d(self.WHEEL_BASE_HALF_LENGTH_M, self.WHEEL_BASE_HALF_WIDTH_M)
        )
        self.robotToModuleTranslations.append(
            Translation2d(self.WHEEL_BASE_HALF_LENGTH_M, -self.WHEEL_BASE_HALF_WIDTH_M)
        )
        self.robotToModuleTranslations.append(
            Translation2d(-self.WHEEL_BASE_HALF_LENGTH_M, self.WHEEL_BASE_HALF_WIDTH_M)
        )
        self.robotToModuleTranslations.append(
            Translation2d(-self.WHEEL_BASE_HALF_LENGTH_M, -self.WHEEL_BASE_HALF_WIDTH_M)
        )
        
        # WPILib Kinematics object
        self.kinematics = SwerveDrive4Kinematics(
            self.robotToModuleTranslations[FL],
            self.robotToModuleTranslations[FR],
            self.robotToModuleTranslations[BL],
            self.robotToModuleTranslations[BR],
        )

    # Utility conversion functions to go between drivetrain "linear" measurements and wheel motor rotational measurements
    def dtLinearToMotorRot(self, lin):
        # lin - meters per second at wheel contact patch
        # return - radians per second of motor shaft
        return lin / (inchesToMeters(self.WHEEL_RADIUS_IN)) * self.WHEEL_GEAR_RATIO

    def dtMotorRotToLinear(self, rot):
        # rot - radians per second of motor shaft
        # return = meters per second at wheel contact patch
        return rot * (inchesToMeters(self.WHEEL_RADIUS_IN)) / self.WHEEL_GEAR_RATIO


