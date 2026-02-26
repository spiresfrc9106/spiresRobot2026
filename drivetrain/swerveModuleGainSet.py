from drivetrain.drivetrainPhysical import DrivetrainPhysical
from utils.calibration import Calibration
from utils.units import RPM2RadPerSec


class SwerveModuleGainSet:
    """Helper class to house all calibrated gains for one swerve drive module.
    Currently, this includes
     * Wheel feed back PID gains
     * Wheel feed-forward SVA gains
     * Azimuth feed back PID gains
    """

    def __init__(self):

        p = DrivetrainPhysical()
        self.wheelP = Calibration("Drivetrain Module Wheel kP", p.WHEEL_P)
        self.wheelI = Calibration("Drivetrain Module Wheel kI", p.WHEEL_I)
        self.wheelD = Calibration("Drivetrain Module Wheel kD", p.WHEEL_D)
        self.wheelA = Calibration(
            "Drivetrain Module Wheel kA", p.WHEEL_A, "volts/radPerSecPerSec"
        )
        self.wheelV = Calibration(
            "Drivetrain Module Wheel kV", p.WHEEL_V, "volts/radPerSec"
        )
        self.wheelS = Calibration("Drivetrain Module Wheel kS", p.WHEEL_S, "volts")
        self.azmthP = Calibration("Drivetrain Module Azmth kP", p.AZMTH_P)
        self.azmthI = Calibration("Drivetrain Module Azmth kI", p.AZMTH_I)
        self.azmthD = Calibration("Drivetrain Module Azmth kD", p.AZMTH_D)
        self.azmthA = Calibration("Drivetrain Module Azmth kA", p.AZMTH_A)
        self.azmthV = Calibration("Drivetrain Module Azmth kV", p.AZMTH_V)
        self.azmthS = Calibration("Drivetrain Module Azmth kS", p.AZMTH_S, "volts")

    def hasChanged(self)->bool:
        """
        Returns:
            bool: True if any gain in the set is modified, false otherwise
        """
        return (
            self.wheelP.isChanged()
            or self.wheelI.isChanged()
            or self.wheelD.isChanged()
            or self.wheelA.isChanged()
            or self.wheelV.isChanged()
            or self.wheelS.isChanged()
            or self.azmthP.isChanged()
            or self.azmthI.isChanged()
            or self.azmthD.isChanged()
            or self.azmthA.isChanged()
            or self.azmthV.isChanged()
            or self.azmthS.isChanged()
        )
    def setWheelP(self, wheel_P):
        self.wheelP = wheel_P
    def setWheelI(self, wheel_I):
        self.wheelI = wheel_I
    def setWheelD(self, wheel_D):
        self.wheelD = wheel_D
    def setWheelA(self, wheel_A):
        self.wheelA = wheel_A
    def setWheelV(self, wheel_V):
        self.wheelV = wheel_V
    def setWheelS(self, wheel_S):
        self.wheelS = wheel_S
    def setAzmthP(self, azmth_P):
        self.azmthP = azmth_P
    def setAzmthI(self, azmth_I):
        self.azmthI = azmth_I
    def setAzmthD(self,azmth_D):
        self.azmthD = azmth_D
    def setAzmthA(self, azmth_A):
        self.azmthA = azmth_A
    def setAzmthV(self, azmth_V):
        self.azmthV = azmth_V
    def setAzmthS(self, azmth_S):
        self.azmthS = azmth_S
