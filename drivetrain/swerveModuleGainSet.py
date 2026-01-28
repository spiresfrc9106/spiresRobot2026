from drivetrain.drivetrainPhysical import MAX_DT_MOTOR_SPEED_RPS
from utils.calibration import Calibration

class SwerveModuleGainSet:
    """Helper class to house all calibrated gains for one swerve drive module.
    Currently, this includes
     * Wheel feed back PID gains
     * Wheel feed-forward SVA gains
     * Azimuth feed back PID gains
    """

    def __init__(self,module:str=""):

        self.wheelP = Calibration(module + "Drivetrain Module Wheel kP", 0.03)
        self.wheelI = Calibration(module + "Drivetrain Module Wheel kI", 0.0)
        self.wheelD = Calibration(module + "Drivetrain Module Wheel kD", 0.0)
        self.wheelA = Calibration(
            module + "Drivetrain Module Wheel kA", 0.000, "volts/radPerSecPerSec"
        )
        self.wheelV = Calibration(
            module + "Drivetrain Module Wheel kV", 12.0 / MAX_DT_MOTOR_SPEED_RPS, "volts/radPerSec"
        )
        self.wheelS = Calibration(module + "Drivetrain Module Wheel kS" + " " + module, 0.15, "volts")
        self.azmthP = Calibration(module + "Drivetrain Module Azmth kP" + " " + module, 0.03)
        self.azmthI = Calibration(module + "Drivetrain Module Azmth kI" + " " + module, 0.0)
        self.azmthD = Calibration(module + "Drivetrain Module Azmth kD" + " " + module, 0.0)

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
        )
