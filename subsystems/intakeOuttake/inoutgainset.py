from drivetrain.drivetrainPhysical import DrivetrainPhysical
from utils.calibration import Calibration
from utils.singleton import Singleton
from utils.units import RPM2RadPerSec


class InOutGainSet(metaclass=Singleton):
    """Helper class to house all calibrated gains for inOut subsystem.
    """

    def __init__(self):

        #TODO pull these calibration values out of the constants
        self.groundP = Calibration("InOut Ground kP", 0.000_1)
        self.groundD = Calibration("InOut Ground kD", 0.0)
        self.groundS = Calibration("InOut Ground kS", 0.5, "volts")
        self.groundV = Calibration("InOut Ground kV", 0.0 , "volts/radPerSec")
        self.groundA = Calibration("InOut Ground kA", 0.000, "volts/radPerSecPerSec")

        self.hopperP = Calibration("InOut Hopper kP", 0.000_1)
        self.hopperD = Calibration("InOut Hopper kD", 0.0)
        self.hopperS = Calibration("InOut Hopper kS", 0.5, "volts")
        self.hopperV = Calibration("InOut Hopper kV", 0.0 , "volts/radPerSec")
        self.hopperA = Calibration("InOut Hopper kA", 0.000, "volts/radPerSecPerSec")

        self.flywheelP = Calibration("InOut Flywheel kP", 0.000_1)
        self.flywheelD = Calibration("InOut Flywheel kD", 0.0)
        self.flywheelS = Calibration("InOut Flywheel kS", 0.5, "volts")
        self.flywheelV = Calibration("InOut Flywheel kV", 0.0 , "volts/radPerSec")
        self.flywheelA = Calibration("InOut Flywheel kA", 0.000, "volts/radPerSecPerSec")

        self.groundIntakeSpeedIPS = Calibration("Ground Intake Speed IPS", 10.0, "in/sec")
        self.groundOuttakeSpeedIPS = Calibration("Ground Outtake Speed IPS", 10.0, "in/sec")
        self.groundShootSpeedIPS = Calibration("Ground Shoot Speed IPS", 10.0, "in/sec")

        self.hopperIntakeSpeedIPS = Calibration("Hopper Intake Speed IPS", 10.0, "in/sec")
        self.hopperOuttakeSpeedIPS = Calibration("Hopper Outtake Speed IPS", 10.0, "in/sec")
        self.hopperShootSpeedIPS = Calibration("Hopper Shoot Speed IPS", 10.0, "in/sec")

        self.flywheelSpeedIPS = Calibration("Flywheel Speed IPS", 10.0, "rpm")

    def hasChanged(self)->bool:
        """
        Returns:
            bool: True if any gain in the set is modified, false otherwise
        """
        return (
            self.groundP.isChanged()
            or self.groundD.isChanged()
            or self.groundS.isChanged()
            or self.groundV.isChanged()
            or self.groundA.isChanged()
            or self.hopperP.isChanged()
            or self.hopperD.isChanged()
            or self.hopperS.isChanged()
            or self.hopperV.isChanged()
            or self.hopperA.isChanged()
            or self.flywheelP.isChanged()
            or self.flywheelD.isChanged()
            or self.flywheelS.isChanged()
            or self.flywheelV.isChanged()
            or self.flywheelA.isChanged()
            or self.groundIntakeSpeedIPS.isChanged()
            or self.groundOuttakeSpeedIPS.isChanged()
            or self.groundShootSpeedIPS.isChanged()
            or self.hopperIntakeSpeedIPS.isChanged()
            or self.hopperOuttakeSpeedIPS.isChanged()
            or self.hopperShootSpeedIPS.isChanged()
            or self.flywheelSpeedIPS.isChanged()
        )

    def setGroundP(self, ground_P):
        self.groundP = ground_P

    def setGroundD(self, ground_D):
        self.groundD = ground_D

    def setGroundS(self, ground_S):
        self.groundS = ground_S

    def setGroundV(self, ground_V):
        self.groundV = ground_V

    def setGroundA(self, ground_A):
        self.groundA = ground_A

    def setHopperP(self, hopper_P):
        self.hopperP = hopper_P

    def setHopperD(self, hopper_D):
        self.hopperD = hopper_D

    def setHopperS(self, hopper_S):
        self.hopperS = hopper_S

    def setHopperV(self, hopper_V):
        self.hopperV = hopper_V

    def setHopperA(self, hopper_A):
        self.hopperA = hopper_A

    def setFlywheelP(self, flywheel_P):
        self.flywheelP = flywheel_P

    def setFlywheelD(self, flywheel_D):
        self.flywheelD = flywheel_D

    def setFlywheelS(self, flywheel_S):
        self.flywheelS = flywheel_S

    def setFlywheelV(self, flywheel_V):
        self.flywheelV = flywheel_V

    def setFlywheelA(self, flywheel_A):
        self.flywheelA = flywheel_A

    def setGroundIntakeSpeedIPS(self, groundIntake_IPS):
        self.groundIntakeSpeedIPS = groundIntake_IPS

    def setGroundOuttakeSpeedIPS(self, groundOuttake_IPS):
        self.groundOuttakeSpeedIPS = groundOuttake_IPS

    def setGroundShootSpeedIPS(self, groundShoot_IPS):
        self.groundShootSpeedIPS = groundShoot_IPS

    def setHopperIntakeSpeedIPS(self, hopperIntake_IPS):
        self.hopperIntakeSpeedIPS = hopperIntake_IPS

    def setHopperOuttakeSpeedIPS(self, hopperOuttake_IPS):
        self.hopperOuttakeSpeedIPS = hopperOuttake_IPS

    def setHopperShootSpeedIPS(self, hopperShoot_IPS):
        self.hopperShootSpeedIPS = hopperShoot_IPS

    def setFlywheelSpeedIPS(self, flywheelSpeed_IPS):
        self.flywheelSpeedIPS = flywheelSpeed_IPS