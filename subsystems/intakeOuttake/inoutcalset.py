from subsystems.intakeOuttake.motormodule import MotorModuleCals
from subsystems.state.configsubsystem import ConfigSubsystem
from utils.calibration import Calibration
from utils.singleton import Singleton

class InOutCalSet(metaclass=Singleton):
    """Helper class to house all calibrated gains for inOut subsystem.
    """

    def __init__(self):
        depConsts = ConfigSubsystem().inoutDepConstants
        if depConsts["HAS_INOUT"]:
            self.groundP = Calibration("InOut Ground kP", depConsts["GROUND_KP"])
            self.groundD = Calibration("InOut Ground kD", depConsts["GROUND_KD"])
            self.groundS = Calibration("InOut Ground kS", depConsts["GROUND_KS"], "volts")
            self.groundV = Calibration("InOut Ground kV", depConsts["GROUND_KV"], "volts/radPerSec")
            self.groundA = Calibration("InOut Ground kA", depConsts["GROUND_KA"], "volts/radPerSecPerSec")
    
            self.hopperP = Calibration("InOut Hopper kP", depConsts["HOPPER_KP"])
            self.hopperD = Calibration("InOut Hopper kD", depConsts["HOPPER_KD"])
            self.hopperS = Calibration("InOut Hopper kS", depConsts["HOPPER_KS"], "volts")
            self.hopperV = Calibration("InOut Hopper kV", depConsts["HOPPER_KV"], "volts/radPerSec")
            self.hopperA = Calibration("InOut Hopper kA", depConsts["HOPPER_KA"], "volts/radPerSecPerSec")
    
            self.flywheelP = Calibration("InOut Flywheel kP", depConsts["FLYWHEEL_KP"])
            self.flywheelD = Calibration("InOut Flywheel kD", depConsts["FLYWHEEL_KD"])
            self.flywheelS = Calibration("InOut Flywheel kS", depConsts["FLYWHEEL_KS"], "volts")
            self.flywheelV = Calibration("InOut Flywheel kV", depConsts["FLYWHEEL_KV"], "volts/radPerSec")
            self.flywheelA = Calibration("InOut Flywheel kA", depConsts["FLYWHEEL_KA"], "volts/radPerSecPerSec")

            self.groundMaxAccIPS2 = Calibration( "InOut Ground Max Acc IPS2", depConsts["GROUND_MAX_MOTION_MAX_ACC_IPS2"], "in/sec^2")
            self.hopperMaxAccIPS2 = Calibration( "InOut Hopper Max Acc IPS2", depConsts["HOPPER_MAX_MOTION_MAX_ACC_IPS2"], "in/sec^2")
            self.flywheelMaxAccIPS2 = Calibration( "InOut Hopper Max Acc IPS2", depConsts["FLYWHEEL_MAX_MOTION_MAX_ACC_IPS2"], "in/sec^2")

            self.groundCals = MotorModuleCals(
                kP=self.groundP, kD=self.groundD,
                kS=self.groundS, kV=self.groundV, kA=self.groundA,
                maxAccUserUnitsPerS2=self.groundMaxAccIPS2)
            self.hopperCals = MotorModuleCals(
                kP=self.hopperP, kD=self.hopperD,
                kS=self.hopperS, kV=self.hopperV, kA=self.hopperA,
                maxAccUserUnitsPerS2=self.hopperMaxAccIPS2)
            self.flywheelCals = MotorModuleCals(
                kP=self.flywheelP, kD=self.flywheelD,
                kS=self.flywheelS, kV=self.flywheelV, kA=self.flywheelA,
                maxAccUserUnitsPerS2=self.flywheelMaxAccIPS2)
    
            self.groundIntakeSpeedIPS = Calibration("Ground Intake Speed IPS", depConsts["GROUND_INTAKE_SPEED_IPS"], "in/sec")
            self.groundOuttakeSpeedIPS = Calibration("Ground Outtake Speed IPS", depConsts["GROUND_OUTTAKE_SPEED_IPS"], "in/sec")
            self.groundShootSpeedIPS = Calibration("Ground Shoot Speed IPS", depConsts["GROUND_SHOOT_SPEED_IPS"], "in/sec")
    
            self.hopperIntakeSpeedIPS = Calibration("Hopper Intake Speed IPS", depConsts["HOPPER_INTAKE_SPEED_IPS"], "in/sec")
            self.hopperOuttakeSpeedIPS = Calibration("Hopper Outtake Speed IPS", depConsts["HOPPER_OUTTAKE_SPEED_IPS"], "in/sec")
            self.hopperShootSpeedIPS = Calibration("Hopper Shoot Speed IPS", depConsts["HOPPER_SHOOT_SPEED_IPS"], "in/sec")
    
            self.flywheelSpeedIPS = Calibration("Flywheel Speed IPS", depConsts["FLYWHEEL_SPEED_IPS"], "in/sec")

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
            or self.groundMaxAccIPS2.isChanged()
            or self.hopperMaxAccIPS2.isChanged()
            or self.flywheelMaxAccIPS2.isChanged()
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

    def setGroundMaxAccIPS2(self, ground_maxAccIPS2):
        self.groundMaxAccIPS2 = ground_maxAccIPS2

    def setHopperMaxAccIPS2(self, hopper_maxAccIPS2):
        self.hopperMaxAccIPS2 = hopper_maxAccIPS2

    def setFlywheelMaxAccIPS2(self, flywheel_maxAccIPS2):
        self.flywheelMaxAccIPS2 = flywheel_maxAccIPS2

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