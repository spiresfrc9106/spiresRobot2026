
from subsystems.intakeOuttake.inoutsubsystem import InOutSubsystem, inoutSubsystemFactory

from subsystems.state.configsubsystem import ConfigSubsystem
from subsystems.state.robottopsubsystem import RobotTopSubsystem




class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, xyzzy, and button mappings) should be declared here.
    """
    """
    Spires addendum to RobotContainer design, the robot container should know very little about the robot's subsystems,
    or how to build them or what they connect to. That information should be in the subsystem classes.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.config = ConfigSubsystem()
        self.robotop = RobotTopSubsystem()
        self.inout: InOutSubsystem|None = inoutSubsystemFactory()

    def robotPeriodic(self) -> None:
        pass

    def quietRobotOnExitFromActiveMode(self) -> None:
        if self.inout is not None:
            self.inout.initialize()

