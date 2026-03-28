from dataclasses import dataclass
from pykit.autolog import autolog

from wpilib import RobotController

from subsystems.state.configio import RobotTypes
from utils.singleton import Singleton


class RobotTopDependentConstants(metaclass=Singleton):
    def __init__(self) -> None:
        self._constants = {
            RobotTypes.Spires2023: {"GYRO": "NAVX"},
            RobotTypes.Spires2026: {"GYRO": "ADIS16470_IMU"},
            RobotTypes.Spires2026Sim: {"GYRO": "ADIS16470_IMU"},
            RobotTypes.SpiresTestBoard: {"GYRO": "NoGyro"},
            RobotTypes.SpiresRoboRioV1: {"GYRO": "NoGyro"},
        }

    def get(self, robotType: RobotTypes) -> dict:
        return self._constants[robotType]


class RobotTopIO:
    """Process I/O data for the robot high-level state subsystem."""

    @autolog
    @dataclass
    class RobotTopIOInputs:
        """Hold I/O data for the robot high-level state subsystem."""

        timeUSec: int = 0
        gyroAngleRad: float = 0.0
        gyroConnected: bool = False
        gyroYawRateRadPerSec: float = 0.0

    def __init__(self, gyro) -> None:
        self.gyro = gyro

    def updateInputs(self, inputs: RobotTopIOInputs) -> None:
        """Update the robot high-level state I/O inputs.

        Args:
            inputs (RobotTopIOInputs): The robot high-level state I/O inputs to update.
        """
        inputs.timeUSec = RobotController.getFPGATime()
        inputs.gyroAngleRad = self.gyro.getGyroAngleRotation2d().radians()
        inputs.gyroConnected = self.gyro.isConnected()
        inputs.gyroYawRateRadPerSec = self.gyro.getRate()
