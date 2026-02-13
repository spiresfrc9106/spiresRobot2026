#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from phoenix6.unmanaged import feed_enable
from wpilib import RobotController
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from pyfrc.physics.core import PhysicsInterface
from robot import MyRobot
from westwood.robotstate import RobotState
from westwood.subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from westwood.subsystems.drive.drivesubsystem import DriveSubsystem

from westwood.constants.sim import (
    kSimDefaultRobotLocation,
)


class SwerveDriveSim:
    def __init__(self, driveSubsystem: DriveSubsystem) -> None:
        self.driveSubsystem = driveSubsystem
        self.pose = kSimDefaultRobotLocation

    def resetPose(self, pose) -> None:
        self.pose = pose

    def getSimPose(self) -> Pose2d:
        return self.pose

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, _robotVoltage: float) -> None:
        deltaT = tm_diff

        chassisSpeed = self.driveSubsystem.getRobotRelativeSpeeds()
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    """
    The call order:
    0 robotInit has run
    ... we initialize all of the subsystems and their hardware...
    0 PhysicsEngine.__init__
    [Physics] WARNING: Westwood Swerve is Not simulating
    Robot startup complete!
    [WPILogWriter] Creating WPILOG file at
    [Auto] New Modes Selected: None Wait 0.0s Left Do Nothing None
    0 robotPeriodic
    1 robotPeriodic
    2 PhysicsEngine.update_sim
    [WPILogWriter] Renaming log to pykit_20260211_165729.wpilog
    2 robotPeriodic
    
    ... then when entering teleop mode:
    50 PhysicsEngine.update_sim
    50 teleopInit
    50 teleopPeriodic
    50 robotPeriodic
    51 PhysicsEngine.update_sim
    51 teleopPeriodic
    51 robotPeriodic
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller
        self.bot = robot

        print(f"{self.bot.count} PhysicsEngine.__init__")

        driveSubsystem: DriveSubsystem|None = None
        if robot.westwoodContainer is not None:
            driveSubsystem: DriveSubsystem = robot.westwoodContainer.drive

        if driveSubsystem is None or not isinstance(driveSubsystem.frontLeftModule.io, SwerveModuleIOCTRE):
            # do not simulation
            self.doSim = False
            print("[Physics] WARNING: Westwood Swerve is Not simulating")
            return

        self.doSim = True
        print("[Physics] beginning simulation")

        self.driveSim = SwerveDriveSim(driveSubsystem)

        RobotState.registerSimPoseResetConsumer(self.driveSim.resetPose)
        RobotState.registerSimPoseRecieverConsumer(self.driveSim.getSimPose)

        self.sim_initialized = False

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        #print(f"{self.bot.count} PhysicsEngine.update_sim")
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if not self.doSim:
            return
        feed_enable(tm_diff)

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.driveSim.update(tm_diff, voltage)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)
