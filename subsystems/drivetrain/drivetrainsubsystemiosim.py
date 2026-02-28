from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.drivetrain.drivetrainsubsystemio import DrivetrainSubsystemIO
from subsystems.drivetrain.drivetrainsubsystemioreal import DrivetrainSubsystemIOReal



class DrivetrainSubsystemIORealSim(DrivetrainSubsystemIOReal):
    """Simulate the motors"""

    def __init__(self,
        name: str,
    ) -> None:
        super().__init__(name)  # Initialize the Sim motor the same way as the actual Talon motor
        """
        self.turretSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kTurretSimMotor, kTurretSimInertia, kTurretGearRatio
            ),
            kTurretSimMotor,
        )  # Create a DC motor simulation model with specified parameters
        """

    def updateInputs(self, inputs: DrivetrainSubsystemIO.DrivetrainSubsystemIOInputs) -> None:
        """Simulate the motor behavior, then update TalonIO inputs."""
        """"
        turretMotorSim = self.motor.sim_state
        simVoltage = RobotController.getInputVoltage()

        turretAppliedVoltage = clamp(turretMotorSim.motor_voltage, -12.0, 12.0)

        self.turretSimModel.setInputVoltage(turretAppliedVoltage)
        self.turretSimModel.update(kRobotUpdatePeriodS)

        turretMotorSim.set_raw_rotor_position(
            self.turretSimModel.getAngularPositionRotations() * kTurretGearRatio
        )

        turretMotorSim.set_rotor_velocity(
            self.turretSimModel.getAngularVelocity()
            / kRadiansPerRevolution
            * kTurretGearRatio
        )
        turretMotorSim.set_supply_voltage(
            clamp(
                simVoltage - turretMotorSim.supply_current * kTurretSimMotor.R,
                0,
                simVoltage,
            )  # Apply some simulated voltage within appropriate limits
        )
        """
        super().updateInputs(inputs)  # Call the TalonIO updateInputs method
