import typing
from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import (
    InvertedValue,
    NeutralModeValue,
    Slot0Configs,
)
from phoenix6.controls import (
    MotionMagicVoltage,
    VelocityTorqueCurrentFOC, VelocityVoltage, VoltageOut,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from subsystems.common.motormoduleio import MotorModuleIO, MotorModuleConfigParams
from westwood.util.phoenixutil import PhoenixUtil, tryUntilOk

from constants import kRobotUpdateFrequency
from westwood.constants.drive import (
    kDrivePGain,
    kDriveIGain,
    kDriveDGain,
    kDriveVGain,
    kDriveCurrentLimit,
    kDriveGearingRatio,
)

from westwood.constants.math import kRadiansPerRevolution


class MotorModuleIOCTRE(MotorModuleIO):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Kraken X60 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    driveConfig: TalonFXConfiguration = TalonFXConfiguration()

    driveRequest: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0)

    def __init__(self, name: str, config: MotorModuleConfigParams) -> None:
        MotorModuleIO.__init__(self, name)
        print(f"Initializing swerve module: {self.name}")
        self.driveMotor = TalonFX(config.driveMotorID, config.canbus)

        self.driveConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kDrivePGain)
            .with_k_i(kDriveIGain)
            .with_k_d(kDriveDGain)
            .with_k_v(kDriveVGain)
        )
        self.driveConfig.feedback.sensor_to_mechanism_ratio = kDriveGearingRatio
        self.driveConfig.current_limits = kDriveCurrentLimit
        self.driveConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if config.driveMotorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        tryUntilOk(
            5, lambda: self.driveMotor.configurator.apply(self.driveConfig, 0.25)
        )
        tryUntilOk(5, lambda: self.driveMotor.set_position(0, 0.25))


        self.drivePosition = self.driveMotor.get_position()
        self.driveVelocity = self.driveMotor.get_velocity()
        self.driveApplied = self.driveMotor.get_motor_voltage()
        self.driveSupplyCurrent = self.driveMotor.get_supply_current()
        self.driveTorqueCurrent = self.driveMotor.get_torque_current()


        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
        )

        PhoenixUtil.registerSignals(
            config.canbus,
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
        )

        print("... Done")

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            posCmdRad (float): motor desired shaft rotations in radians
            arbFF (float, optional): _description_. Defaults to 0.
        """
        rotations = posCmdRad/ kRadiansPerRevolution
        self.driveMotor.set_control(MotionMagicVoltage(rotations, arbFF))


    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            velCmdRadps (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """
        rotationsPerSec = velCmdRadps/ kRadiansPerRevolution
        self.driveMotor.set_control(VelocityVoltage(rotationsPerSec, arbFF))


    def setVoltage(self, outputVoltageVolts:float)->None:
        self.driveMotor.set_control(VoltageOut(outputVoltageVolts))

    def getSimulator(
        self,
    ) -> tuple[
        typing.Callable[[], TalonFXSimState],
    ]:
        return (
            (lambda: self.driveMotor.sim_state),

        )

    def updateInputs(self, inputs: MotorModuleIO.MotorModuleIOInputs) -> None:
        inputs.connected = BaseStatusSignal.is_all_good(
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
        )

        inputs.posRad = self.drivePosition.value * kRadiansPerRevolution
        inputs.velRadps = self.driveVelocity.value * kRadiansPerRevolution
        inputs.appliedV = self.driveApplied.value
        inputs.drive_supply_current = self.driveSupplyCurrent.value
        inputs.torqueCurrentA = self.driveTorqueCurrent.value



