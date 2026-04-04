from constants import kRobotUpdatePeriodS
from pykit.autolog import autologgable_output, autolog_output

from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import DrivetrainPhysical
from pykit.logger import Logger
from subsystems.state.robottopsubsystem import RobotTopSubsystem
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from wpimath import applyDeadband
from wpilib import DriverStation, XboxController, RobotController
from utils.calibration import Calibration
from utils.singleton import Singleton
from utils.slewratelimitawayfromzero import SlewRateLimitAwayFromZero
from utils.units import rad2Deg, deg2Rad


@autologgable_output
class DriverInterface(metaclass=Singleton):
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # contoller
        ctrlIdx = 0
        now = RobotController.getFPGATime()
        now2 = RobotTopSubsystem().getFPGATimeUS()
        print(f"now: {now}, now2: {now2}")
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Driver XBox controller ({ctrlIdx}) unplugged")

        # Drivetrain motion xyzzy
        self.velXCmd = 0.0
        self.velYCmd = 0.0
        self.velTCmd = 0.0

        p = DrivetrainPhysical()
        self.calTranslateSlowMultiplier = Calibration(
            name="Driver Interface Translate Slow Multiplier", default=0.5
        )
        self.calRotateSlowMultiplier = Calibration(
            name="Driver Interface Rotate Slow Multiplier", default=0.25
        )
        self.calRobotRelativeSlowdown = Calibration(
            name="Driver Interface Robot Relative Multiplier", default=1.0
        )
        self.calMaxRotSpeedDPS = Calibration(
            "Driver Interface Max Rotate Speed DPS",
            rad2Deg(p.MAX_ROTATE_SPEED_RAD_PER_SEC),
        )
        self.calMaxTranslateAccelFactor = Calibration(
            "Driver Interface Max Translate Accel Factor", 1.0
        )
        self.calMaxRotateAccelFactor = Calibration(
            "Driver Interface Max Rotate Accel Factor", 1.0
        )
        self.calDecelFactor = Calibration("Driver Interface Decel Factor", 1.5)
        self.initiatialize()

    def initiatialize(self) -> None:
        now = RobotController.getFPGATime()
        now2 = RobotTopSubsystem().getFPGATimeUS()
        print(f"DriverInterface Initialize - now: {now}, now2: {now2}")
        p = DrivetrainPhysical()
        self.MAX_FWD_REV_SPEED_MPS = p.MAX_FWD_REV_SPEED_MPS
        self.MAX_STRAFE_SPEED_MPS = p.MAX_STRAFE_SPEED_MPS

        self.translateSlowMultiplier = self.calTranslateSlowMultiplier.get()
        self.rotateSlowMultiplier = self.calRotateSlowMultiplier.get()

        self.MAX_ROTATE_SPEED_RAD_PER_SEC = deg2Rad(self.calMaxRotSpeedDPS.get())

        self.MAX_TRANSLATE_ACCEL_MPS2 = (
            p.MAX_TRANSLATE_ACCEL_MPS2 * self.calMaxTranslateAccelFactor.get()
        )
        self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = (
            p.MAX_ROTATE_ACCEL_RAD_PER_SEC_2 * self.calMaxRotateAccelFactor.get()
        )

        # Driver motion rate limiters - enforce smoother driving
        self.translateAccelFactor = 1.0
        self.rotateAccelFactor = 1.0

        self.newTranslateSlewRateLimiter(0.0, 0.0)
        self.newRotateSlewRateLimiter(0.0)

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False

        # utility - use robot-relative xyzzy
        self.robotRelative = False

    def newTranslateSlewRateLimiter(
        self, initialValueX: float, initialValueY: float
    ) -> None:
        self.velXSlewRateLimiter = SlewRateLimitAwayFromZero(
            awayRate=self.MAX_TRANSLATE_ACCEL_MPS2 * self.translateAccelFactor,
            towardsZeroRate=self.MAX_TRANSLATE_ACCEL_MPS2 * self.calDecelFactor.get(),
            initialValue=initialValueX,
            dtSeconds=kRobotUpdatePeriodS,
        )
        self.velYSlewRateLimiter = SlewRateLimitAwayFromZero(
            awayRate=self.MAX_TRANSLATE_ACCEL_MPS2 * self.translateAccelFactor,
            towardsZeroRate=self.MAX_TRANSLATE_ACCEL_MPS2 * self.calDecelFactor.get(),
            initialValue=initialValueY,
            dtSeconds=kRobotUpdatePeriodS,
        )
        print(
            f"DriverInterface newTranslateSlewRateLimiter - now: {RobotController.getFPGATime()}, now2: {RobotTopSubsystem().getFPGATimeUS()}"
        )
        Logger.recordOutput("di/translateAccelFactor", self.translateAccelFactor)

    def newRotateSlewRateLimiter(self, initialValue: float) -> None:
        self.velTSlewRateLimiter = SlewRateLimitAwayFromZero(
            awayRate=self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2 * self.rotateAccelFactor,
            towardsZeroRate=self.MAX_ROTATE_ACCEL_RAD_PER_SEC_2
            * self.calDecelFactor.get(),
            initialValue=initialValue,
            dtSeconds=kRobotUpdatePeriodS,
        )
        print(
            f"DriverInterface newRotateSlewRateLimiter - now: {RobotController.getFPGATime()}, now2: {RobotTopSubsystem().getFPGATimeUS()}"
        )
        Logger.recordOutput("di/rotateAccelFactor", self.rotateAccelFactor)

    @autolog_output(key="di/velXCmd_mps")
    def getVelXCmd(self) -> float:
        return self.velXCmd

    @autolog_output(key="di/velYCmd_mps")
    def getVelYCmd(self) -> float:
        return self.velYCmd

    @autolog_output(key="di/velTCmd_radps")
    def getVelTCmd(self) -> float:
        return self.velTCmd

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystick sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            self.robotRelative = self.ctrl.getLeftBumper()

            if not self.robotRelative:
                # Correct for alliance
                if onRed():
                    vXJoyRaw *= -1.0
                    vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.05)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.05)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.05)

            translateSlowMult = (
                1.0 if self.ctrl.getRightBumper() else self.translateSlowMultiplier
            )
            rotateSlowMult = (
                1.0 if self.ctrl.getRightBumper() else self.rotateSlowMultiplier
            )

            # Shape velocity command
            velCmdXRaw = (
                vXJoyWithDeadband
                * self.MAX_STRAFE_SPEED_MPS
                * self.translateAccelFactor
                * translateSlowMult
            )
            velCmdYRaw = (
                vYJoyWithDeadband
                * self.MAX_FWD_REV_SPEED_MPS
                * self.translateAccelFactor
                * translateSlowMult
            )
            velCmdRotRaw = (
                vRotJoyWithDeadband
                * self.MAX_ROTATE_SPEED_RAD_PER_SEC
                * self.rotateAccelFactor
                * rotateSlowMult
            )

            if self.robotRelative:
                velCmdXRaw *= self.calRobotRelativeSlowdown.get()
                velCmdYRaw *= self.calRobotRelativeSlowdown.get()
                velCmdRotRaw *= self.calRobotRelativeSlowdown.get()

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = self.ctrl.getAButton()

            pov_deg = self.ctrl.getPOV()
            if pov_deg >= 45 and pov_deg <= 135:
                self.rotateAccelFactor = min(1.0, self.rotateAccelFactor + 0.05)
                self.newRotateSlewRateLimiter(self.velTSlewRateLimiter.value)
            elif pov_deg >= 225 and pov_deg <= 315:
                self.rotateAccelFactor = max(0.2, self.rotateAccelFactor - 0.05)
                self.newRotateSlewRateLimiter(self.velTSlewRateLimiter.value)

            if pov_deg >= 0 and pov_deg <= 45:
                self.translateAccelFactor = min(1.0, self.translateAccelFactor + 0.05)
                self.newTranslateSlewRateLimiter(
                    self.velXSlewRateLimiter.value, self.velYSlewRateLimiter.value
                )
            elif pov_deg >= 135 and pov_deg <= 225:
                self.translateAccelFactor = max(0.2, self.translateAccelFactor - 0.05)
                self.newTranslateSlewRateLimiter(
                    self.velXSlewRateLimiter.value, self.velYSlewRateLimiter.value
                )

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state xyzzy and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.robotRelative = False
            if DriverStation.isFMSAttached():
                self.connectedFault.setFaulted()

    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        retval.robotRelative = self.robotRelative
        return retval

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getRobotRelative(self):
        return self.robotRelative
