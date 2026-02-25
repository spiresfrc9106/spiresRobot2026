from typing_extensions import Self
from commands2 import Subsystem
from pykit.logger import Logger
from pykit.autolog import autolog_output, autologgable_output

from subsystems.state.robottopio import RobotTopIO
from utils.singleton import _instances
from westwood.util.logtracer import LogTracer

# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class RobotTopSubsystem(Subsystem):
    """
    A singleton class that records the robot's high-level state.
    """
    _initalized = False

    # Because this is a singleton, we need to override __new__ to return the same instance every time.
    # We had to do this differently than utils.singleton.Singleton because we need to subclass
    # Subsystem, which doesn't allow us to use class RobotTopSubsystem(Subsystem, metaclass=Singleton).
    # Note that utils.singleton.Singleton uses def __call__(self, *args, **kwargs) -> Self:, while we need __new__,
    # because Subsystem has a custom __new__ method.
    def __new__(cls, *arg, **kwargs) -> Self:
        if cls not in _instances:
            instance = super().__new__(cls)
            _instances[cls] = instance
        return _instances[cls]

    def __init__(self) -> None:
        if self._initalized:
            return
        self.io = RobotTopIO()
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.inputs = RobotTopIO.RobotTopIOInputs()

        self._initalized = True

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's configuration
        """

        LogTracer.resetOuter("RobotTopSubsystemPeriodic")
        self.io.updateInputs(self.inputs)
        LogTracer.record("IOUpdate")
        """TODO put any configuation IOUpdates here"""
        LogTracer.record("StateUpdate")
        Logger.processInputs("Config", self.inputs)
        LogTracer.record("LoggerProcessInputs")

        """TODO put any configuation periodic here"""
        LogTracer.record("ModulesPeriodic")
        LogTracer.recordTotal()

    def getFPGATimeUS(self) -> int:
        """The time of the current robot periodic loop in microseconds."""
        return self.inputs.timeUSec

    def getFPGATimestampS(self) -> float:
        """The time of the current robot periodic loop in seconds."""
        return self.inputs.timeUSec / 1.0e6

