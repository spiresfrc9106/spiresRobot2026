import random
import math
from typing import Tuple

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpilib import TimedRobot
from drivetrain.drivetrainPhysical import DrivetrainPhysical
from drivetrain.drivetrainPhysical import wrapperedSwerveDriveAzmthEncoder
from drivetrain.swerveModuleGainSet import SwerveModuleGainSet
from subsystems.common.encodermoduleio import EncoderModuleIO
from subsystems.common.motormoduleio import MotorModuleIO
from wrappers.wrapperedMotorSuper import WrapperedMotorSuper
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMotor import  WrapperedSparkMotor
from utils.units import rad2Deg
from pykit.logger import Logger


# Utility signal name calculation functions
def getAzmthDesTopicName(modName):
    return f"DtModule_{modName}_azmthDes"


def getAzmthActTopicName(modName):
    return f"DtModule_{modName}_azmthAct"


def getSpeedDesTopicName(modName):
    return f"DtModule_{modName}_speedDes"


def getSpeedActTopicName(modName):
    return f"DtModule_{modName}_speedAct"


class SwerveModuleControl:
    """
    Control logic for one swerve drive module. 

    Convention Reminders:
    The **module** refers to the whole assembly, including two motors, their built-in sensors, 
    the azimuth angle sensor, the hardware, everything.

    The **azimuth** is the motor, sensor, and mechanism to point the wheel in a specific direction.

    Positive azimuth rotation is counter-clockwise when viewed top-down. By the right hand rule, this is
    rotation in the positive-Z direction. Zero degrees is toward the front of the robot

    The **wheel** is the motor and mechanism to apply a force in that direction.

    Positive wheel rotation causes the robot to move forward if the azimuth is pointed forward.

    Uses WPILib convention for names:
    1) "State" refers to the speed of the wheel, plus the position of the azimuth
    2) "Position" refers to the position of the wheel, plus the position of the azimuth
    """

    def __init__(
        self,
        motorModulesAndEncoderSet: Tuple[str, MotorModuleIO, MotorModuleIO, EncoderModuleIO]
    ):
        """Instantiate one swerve drive module

        Args:
            todo
        """
        p = DrivetrainPhysical()
        self.dtMotorRotToLinear = p.dtMotorRotToLinear
        self.dtLinearToMotorRot = p.dtLinearToMotorRot
        self.MAX_FWD_REV_SPEED_MPS = p.MAX_FWD_REV_SPEED_MPS

        self.moduleName, self.wheelMotor, self.azmthMotor, self.azmthEnc = motorModulesAndEncoderSet

        self.wheelMotorFF = SimpleMotorFeedforwardMeters(0, 0, 0)

        self.desiredState = SwerveModuleState()
        self.optimizedDesiredState = SwerveModuleState()
        self.actualState = SwerveModuleState()
        self.actualPosition = SwerveModulePosition()

        self.azmthCtrl = PIDController(0, 0, 0)
        self.azmthCtrl.enableContinuousInput(-180.0, 180.0)
        self.azmthVoltage = 0.0

        self._prevMotorDesSpeed = 0

        self._azmthDesTopicName = getAzmthDesTopicName(self.moduleName)
        self._azmthActTopicName = getAzmthActTopicName(self.moduleName)
        self._speedDesTopicName = getSpeedDesTopicName(self.moduleName)
        self._speedActTopicName = getSpeedActTopicName(self.moduleName)

        # Simulation Support Only
        self.wheelSimFilter = SlewRateLimiter(24.0)

    def getActualPosition(self)->SwerveModulePosition:
        """
        Returns:
            SwerveModulePosition: The position of the module (azmth and wheel) as measured by sensors
        """
        return self.actualPosition

    def getActualState(self)->SwerveModuleState:
        """
        Returns:
            SwerveModuleState: The state of the module (azmth and wheel) as measured by sensors
        """
        return self.actualState

    def getDesiredState(self)->SwerveModuleState:
        """
        Returns:
            SwerveModuleState: The commanded, desired state of the module (azmth and wheel)
        """
        return self.desiredState

    def setClosedLoopGains(self, gains:SwerveModuleGainSet):
        """Set feed-forward and closed loop gains for the module

        Args:
            gains (SwerveModuleGainSet): The gains for this module
        """
        self.wheelMotor.setPID(
            gains.wheelP.get(), gains.wheelI.get(), gains.wheelD.get()
        )

        if math.isclose(gains.wheelS.get(), 0) and \
                math.isclose(gains.wheelV.get(), 0) and \
                math.isclose(gains.wheelA.get(), 0):
            self.wheelMotorFF = None
        else:
            self.wheelMotorFF = SimpleMotorFeedforwardMeters(
                gains.wheelS.get(), gains.wheelV.get(), gains.wheelA.get()
            )
        self.azmthCtrl.setPID(
            gains.azmthP.get(), gains.azmthI.get(), gains.azmthD.get()
        )

    def setDesiredState(self, desState:SwerveModuleState):
        """Main command input - Call this to tell the module to go to a certian wheel speed and azimuth angle

        Args:
            desState (SwerveModuleState): The commanded state of the module
        """
        self.desiredState = desState

    def update(self):
        """Main update function, call every 40ms"""

        # Read from the azimuth angle sensor (encoder)
        #self.azmthEnc.update() This is now being handled by PyKit moduleIO processing.

        if TimedRobot.isReal():
            # Real Robot. Use the actual sensors to get data about the module.
            # Update this module's actual state with measurements from the sensors
            self.actualState.angle = Rotation2d(self.azmthEnc.getAngleRad())
            self.actualState.speed = self.dtMotorRotToLinear(
                self.wheelMotor.getMotorVelocityRadPerSec()
            )
            self.actualPosition.distance = self.dtMotorRotToLinear(
                self.wheelMotor.getMotorPositionRad()
            )
            self.actualPosition.angle = self.actualState.angle

        # Optimize our incoming swerve command to minimize motion
        self.optimizedDesiredState = self.desiredState
        self.optimizedDesiredState.optimize(self.actualState.angle)
        
        # Use a PID controller to calculate the voltage for the azimuth motor
        self.azmthCtrl.setSetpoint(self.optimizedDesiredState.angle.degrees())  # type: ignore
        self.azmthVoltage = self.azmthCtrl.calculate(self.actualState.angle.degrees())
        self.azmthMotor.setVoltage(self.azmthVoltage)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        self.optimizedDesiredState.speed *= (self.optimizedDesiredState.angle - self.actualState.angle ).cos()

        # Send voltage and speed xyzzy to the wheel motor
        motorDesSpd = self.dtLinearToMotorRot(self.optimizedDesiredState.speed)
        motorDesAccel = (motorDesSpd - self._prevMotorDesSpeed) / 0.02
        if self.wheelMotorFF is None:
            motorVoltageFF = 0
        else:
            motorVoltageFF = self.wheelMotorFF.calculate(motorDesSpd, motorDesAccel)
        self.wheelMotor.setVelCmd(motorDesSpd, motorVoltageFF)

        self._prevMotorDesSpeed = motorDesSpd  # save for next loop

        if TimedRobot.isSimulation():
            # Simulation only. Do a very rough simulation of module behavior, and populate
            # sensor data for the next loop.

            # Very simple voltage/motor model of azimuth rotation
            self.actualState.angle += Rotation2d.fromDegrees(self.azmthVoltage / 12.0 * 1500.0 * 0.04)
            self.actualPosition.angle = self.actualState.angle

            # Wheel speed is slew-rate filtered to roughly simulate robot inertia
            speed = self.wheelSimFilter.calculate(self.desiredState.speed)
            self.actualState.speed = speed + random.uniform(-0.0, 0.0)
            self.actualPosition.distance += self.actualState.speed * 0.04

        Logger.recordOutput(f"{self._azmthDesTopicName}_deg", self.optimizedDesiredState.angle.degrees())
        Logger.recordOutput(f"{self._azmthActTopicName}_deg", self.actualState.angle.degrees())
        Logger.recordOutput(f"{self._speedDesTopicName}_frac", self.optimizedDesiredState.speed / self.MAX_FWD_REV_SPEED_MPS)
        Logger.recordOutput(f"{self._speedActTopicName}_frac", self.actualState.speed / self.MAX_FWD_REV_SPEED_MPS)

