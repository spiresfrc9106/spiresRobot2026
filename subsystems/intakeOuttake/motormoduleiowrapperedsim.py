from subsystems.intakeOuttake.motormoduleio import MotorModuleIO
from subsystems.intakeOuttake.motormoduleiowrappered import MotorModuleIOWrappered
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper

class MotorModuleIOWrapperedSim(MotorModuleIOWrappered):
    def __init__(self, name: str, motor: WrapperedMotorSuper) -> None:
        super().__init__(name, motor)

    def updateInputs(self, inputs: MotorModuleIO.MotorModuleIOInputs) -> None:
        """
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

        """
        return super().updateInputs(inputs)

