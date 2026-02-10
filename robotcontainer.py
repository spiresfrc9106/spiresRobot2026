from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystem

from westwood.constants import RobotModes, kRobotMode



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, xyzzy, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.config = ConfigSubsystem()
        self.robotop = RobotTopSubsystem()
        match kRobotMode:
            case RobotModes.REAL:
                pass
            case RobotModes.SIMULATION:
                pass
            case _:
                pass
        """
        match constants.kRobotMode:
            case constants.RobotModes.REAL:
                self.drive = Drive(DriveIORomiSpark(), GyroIORomi())
                self.drive.io.debugController : XboxController|None = self.controller
                #TODO self.drive = Drive(DriveIORomiSpark(), GyroIOPigeon2())
            case constants.RobotModes.SIMULATION:
                self.drive = Drive(DriveIOSim(), GyroIO())
            case constants.RobotModes.REPLAY:
                self.drive = Drive(DriveIO(), GyroIO())
        """

    def robotPeriodic(self) -> None:
        pass

