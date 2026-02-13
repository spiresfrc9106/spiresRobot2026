from phoenix6 import BaseStatusSignal
from phoenix6.configs.talon_fx_configs import (
    MotionMagicConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.hardware.talon_fx import TalonFX
from wpimath.geometry import Rotation2d

from subsystems.intakeOuttake.inoutsubsystemio import InOutSubsystemIO

from subsystems.intakeOuttake.inout import (
    kTurretCanId,
    kTurretCurrentLimit,
    kTurretGearRatio,
    kTurretMaxAcceleration,
    kTurretMaxVelocity,
    kTurretPGain,
    kTurretIGain,
    kTurretDGain,
    kTurretSGain,
    kTurretVGain,
    kTurretAGain,
)
from subsystems.intakeOuttake.motormodule import MotorModule
from subsystems.intakeOuttake.motormoduleio import MotorModuleIO
from westwood.constants.math import kRadiansPerRevolution
from constants import kRobotUpdateFrequency
from westwood.util.phoenixutil import PhoenixUtil, tryUntilOk
from wrappers.wrapperedSparkMax import WrapperedSparkMax


class InOutSubsystemIOReal(InOutSubsystemIO):

    def __init__(
            self,
            name:str,
    ) -> None:
        self.name = name


    def updateInputs(self, inputs: InOutSubsystemIO.InOutSubsystemIOInputs):
        """Update state of motor per the appropriate specifc API."""
        """
  

        inputs.turretPosition = Rotation2d(self.position.value * kRadiansPerRevolution)
        inputs.turretSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.turretAppliedVolts = self.applied.value
        inputs.turretSupplyAmps = self.supply.value
        """

    def set_turret_angle(self, position: Rotation2d):
        """Move the motor a specified amount of radians."""


    def set_turret_volts(self, volts: float):
        """Move the motor by applying a specific amount of volts."""
