from constants import kRobotUpdatePeriodS
from subsystems.common.encodermoduleiosim import EncoderModuleIOSim
from subsystems.intakeOuttake.inoutsubsystem import OperateFlywheelSimulation
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper


class SwerveModuleSim:
    """Simulation physics for one swerve module.

    Drives SparkMaxSim/SparkFlexSim for the azimuth and wheel motors via
    OperateFlywheelSimulation (same pattern as InOutSubsystemSimulation).
    Integrates the azimuth mechanism angle from the FlywheelSim output and
    injects it into the EncoderModuleIOSim so that module.inputs.curAngleRad
    carries the simulated angle — making the data path identical to replay.
    """

    def __init__(
        self,
        azmthMotor: WrapperedMotorSuper,
        azmthEncoderIO: EncoderModuleIOSim,
        wheelMotor: WrapperedMotorSuper,
        wheelGearRatio: float,
        azmthGearRatio: float,
    ) -> None:
        self.azmthSim = OperateFlywheelSimulation(
            wrapperedMotor=azmthMotor,
            gearRatio=azmthGearRatio,
            moi=0.000_001,
        )
        self.wheelSim = OperateFlywheelSimulation(
            wrapperedMotor=wheelMotor,
            gearRatio=wheelGearRatio,
            moi=0.000_025,
        )
        self.azmthEncoderIO = azmthEncoderIO
        self._azmthAngleRad: float = 0.0

    def periodic(self) -> None:
        self.azmthSim.periodic()
        azmthMechVelRadps = self.azmthSim.flywheelSim.getAngularVelocity()
        self._azmthAngleRad += azmthMechVelRadps * kRobotUpdatePeriodS
        self.azmthEncoderIO.setSimAngleRad(self._azmthAngleRad)

        self.wheelSim.periodic()


class DrivetrainSimulation:
    """Holds one DrivetrainModuleSim per swerve module."""

    def __init__(self, moduleSimulations: list[SwerveModuleSim]) -> None:
        self.moduleSimulations = moduleSimulations

    def periodic(self) -> None:
        for sim in self.moduleSimulations:
            sim.periodic()
