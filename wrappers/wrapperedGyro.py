from wpilib import ADIS16470_IMU
from wpimath.geometry import Rotation2d
import navx
from utils.singleton import Singleton


class WrapperedNoGyro:
    def __init__(self):
        pass

    def getGyroAngleRotation2d(self):
        return Rotation2d(0.0)

    def isConnected(self):
        return False

    def getRate(self) -> float:
        return 0.0


class WrapperedNavx(navx.AHRS):
    """
    Class to wrap a navx
    """

    def __init__(self):
        super().__init__(comType=navx.AHRS.NavXComType.kMXP_SPI, updateRate=50)

    def getGyroAngleRotation2d(self) -> Rotation2d:
        return self.getRotation2d()


class WrapperedNavxSingleton(metaclass=Singleton):
    def __init__(self):
        self._gyro = WrapperedNavx()

    def isConnected(self) -> bool:
        return self._gyro.isConnected()

    def getGyroAngleRotation2d(self) -> Rotation2d:
        return self._gyro.getGyroAngleRotation2d()

    def getYaw(self) -> float:
        return self._gyro.getYaw()

    def getRate(self) -> float:
        return self._gyro.getRate()


class WrapperedAdis16470Imu(ADIS16470_IMU):
    def getGyroAngleRotation2d(self) -> Rotation2d:
        return Rotation2d().fromDegrees(self.getAngle(self.getYawAxis()))


class WrapperedAdis16470ImuSingleton(metaclass=Singleton):
    def __init__(self):
        self._gyro = ADIS16470_IMU()

    def isConnected(self) -> bool:
        return self._gyro.isConnected()

    def getGyroAngleRotation2d(self) -> Rotation2d:
        return Rotation2d().fromDegrees(self._gyro.getAngle(self._gyro.getYawAxis()))

    def getYaw(self) -> float:
        return self._gyro.getYaw()

    def getRate(self) -> float:
        return self._gyro.getRate()
