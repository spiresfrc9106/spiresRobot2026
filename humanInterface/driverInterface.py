from pykit.autolog import autologgable_output, autolog_output

from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import DrivetrainPhysical
from subsystems.config.robottopsubsystem import RobotTopSubsystem
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import DriverStation, XboxController
from utils.calibration import Calibration

@autologgable_output
class DriverInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # contoller
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Driver XBox controller ({ctrlIdx}) unplugged")

        # Drivetrain motion xyzzy
        self.velXCmd = 0.0
        self.velYCmd = 0.0
        self.velTCmd = 0.0

        self.robotRelativeSlowdown = Calibration(name="Robot Relative Slowdown", default=.5, units="%")

        p = DrivetrainPhysical()
        self.MAX_FWD_REV_SPEED_MPS = p.MAX_FWD_REV_SPEED_MPS
        self.MAX_STRAFE_SPEED_MPS = p.MAX_STRAFE_SPEED_MPS

        self.MAX_ROTATE_SPEED_RAD_PER_SEC = p.MAX_ROTATE_SPEED_RAD_PER_SEC

        self.MAX_TRANSLATE_ACCEL_MPS2 = p.MAX_TRANSLATE_ACCEL_MPS2
        self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = p.MAX_ROTATE_ACCEL_RAD_PER_SEC_2

        # Driver motion rate limiters - enforce smoother driving
        self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=self.MAX_TRANSLATE_ACCEL_MPS2)
        self.velYSlewRateLimiter = SlewRateLimiter(rateLimit=self.MAX_TRANSLATE_ACCEL_MPS2)
        self.velTSlewRateLimiter = SlewRateLimiter(rateLimit=self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2)

        # Navigation xyzzy
        self.autoDriveCmd = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False

        #utility - use robot-relative xyzzy
        self.robotRelative = False

        self.autoSteerEnable = True

        #Shooter stuff
        #self.shooterCtrl = ShooterController()

        self.shootCmd = False

    @autolog_output(key="di/velXCmd_mps")
    def getVelXCmd(self) -> float:
        return self.velXCmd
    @autolog_output(key="di/velYCmd_mps")
    def getVelYCmd(self) -> float:
        return self.velYCmd
    @autolog_output(key="di/velTCmd_radps")
    def getVelTCmd(self) -> float:
        return self.velTCmd
    @autolog_output(key="di/autoSteerEnable")
    def getAutoSteerEnable(self) -> bool:
        return self.autoSteerEnable

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            # self.robotRelative = self.ctrl.getLeftBumper()

            if not self.robotRelative:
                # Correct for alliance
                if onRed():
                    vXJoyRaw *= -1.0
                    vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.05)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.05)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.05)

            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.4

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * self.MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * self.MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * self.MAX_ROTATE_SPEED_RAD_PER_SEC * 0.8 * slowMult

            if self.robotRelative:
                velCmdXRaw *= self.robotRelativeSlowdown.get()
                velCmdYRaw *= self.robotRelativeSlowdown.get()
                velCmdRotRaw *= self.robotRelativeSlowdown.get()

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = self.ctrl.getAButton()

            self.autoDriveCmd = self.ctrl.getBButton()
            self.autoSteerToAlgaeProcessor = self.ctrl.getXButton()
            self.autoSteerDownfield = self.ctrl.getYButton()

            if(self.ctrl.getBackButton()):
                self.autoSteerEnable = False
            elif(self.ctrl.getStartButton()):
                self.autoSteerEnable = True
            else:
                pass

            self.shootCmd = self.ctrl.getBButton()
            """
            if self.shootCmd:
                self.shooterCtrl.enableShooting()
            else:
                self.shooterCtrl.disableShooting()
            """
            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state xyzzy and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDriveCmd = False
            self.robotRelative = False
            self.createDebugObstacle = False
            self.shootCmd = False
            #self.shooterCtrl.disableShooting()
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
        print(f"Driver Interface:{RobotTopSubsystem().getFPGATimestampS():7.3f} {self.velXCmd:7.2f} {self.velYCmd:7.2f} {self.velTCmd:7.2f}")

    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        return retval

    def getAutoDrive(self) -> bool:
        return self.autoDriveCmd
    
    def getAutoSteerEnable(self) -> bool:
        return self.autoSteerEnable

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle

    def getRobotRelative(self):
        return self.robotRelative