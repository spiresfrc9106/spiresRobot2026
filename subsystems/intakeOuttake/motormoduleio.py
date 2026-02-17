from dataclasses import dataclass
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


@dataclass
class MotorModuleConfigParams:
    driveMotorID: int
    driveMotorInverted: bool
    canbus: str = ""

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        driveMotorID: int,
        driveMotorInverted: bool,
        canbus: str = "",
    ) -> None:
        self.driveMotorID = driveMotorID
        self.driveMotorInverted = driveMotorInverted
        self.canbus = canbus


class MotorModuleIO:
    @autolog
    @dataclass
    class MotorModuleIOInputs:
        connected: bool = False
        desVoltsOrFfVolts: float = 0.0
        posRad: float = 0.0  # rad
        desVelRadps: float = 0.0
        velRadps: float = 0.0  # rad / sec
        appliedV: float = 0.0  # volts
        torqueCurrentA: float = 0.0  # amps

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: MotorModuleIOInputs) -> None:
        """Update the motor module I/O inputs.

        Args:
            inputs (MotorModuleIOInputs): The module I/O inputs to update.
        """

    def setPID(self, kP: float, kI: float, kD: float) -> None:
        pass

    def setPosCmd(self, posCmdRad:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            posCmdRad (float): motor desired shaft rotations in radians
            arbFF (float, optional): _description_. Defaults to 0.
        """


    def setVelCmd(self, velCmdRadps:float, arbFF:float=0.0)->None:
        """_summary_

        Args:
            velCmdRadps (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """


    def setVoltage(self, outputVoltageVolts:float)->None:
        pass



