from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from subsystems.common.motormoduleiotalonfx import MotorModuleIOCTRE
from subsystems.common.motormoduleio import MotorModuleConfigParams, MotorModuleIO

from westwood.constants.math import kRadiansPerRevolution
from westwood.constants.sim import kSimMotorResistance
from westwood.constants.drive import kDriveGearingRatio
from constants import kRobotUpdatePeriodS
from westwood.util.convenientmath import clamp


class MotorModuleIOTalonSim(MotorModuleIOCTRE):
    def __init__(self, name: str, config: MotorModuleConfigParams) -> None:
        super().__init__(name, config)

        self.motorModel = DCMotor.krakenX60(1)


        self.driveSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.motorModel, 0.025, kDriveGearingRatio
            ),
            self.motorModel,
        )

        self.motorSim = (
            self.getSimulator()
        )

    def updateInputs(self, inputs: MotorModuleIO.MotorModuleIOInputs) -> None:
        motorSim = self.motorSim()

        simVoltage = RobotController.getInputVoltage()

        self.driveSim.setInputVoltage(clamp(motorSim.motor_voltage, -12, 12))
        self.driveSim.update(kRobotUpdatePeriodS)

        motorSim.set_raw_rotor_position(
            self.driveSim.getAngularPositionRotations() * kDriveGearingRatio
        )  # since the robot position is before mechanism ratio, we have to add the ratio ourselves
        motorSim.set_rotor_velocity(
            self.driveSim.getAngularVelocity()
            / kRadiansPerRevolution
            * kDriveGearingRatio
        )
        motorSim.set_supply_voltage(
            clamp(
                simVoltage - motorSim.supply_current * kSimMotorResistance,
                0,
                simVoltage,
            )
        )


        return super().updateInputs(inputs)
