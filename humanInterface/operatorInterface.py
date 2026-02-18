from enum import Enum

from utils.faults import Fault
from wpilib import DriverStation, XboxController
from utils.mapLookup2d import MapLookup2D
from utils.singleton import Singleton



class FlywheelCommand(Enum):
    kNoCommand = 0
    kSpinningUp = 1
    kSpinningDown = 2


class InOutCommand(Enum):
    kOff = 0
    kIntaking = 1
    kOutaking = 2
    kShooting = 3

class OperatorInterface(metaclass=Singleton):
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.intake = False
        self.inOutCommand = InOutCommand.kOff
        self.flywheelCommand = FlywheelCommand.kNoCommand


    def update(self) -> None:
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            self.connectedFault.setNoFault()

            # A = intake
            # B = outtake
            # X = spinup
            # Y = spindown
            # rightbumper = shoot
            if self.ctrl.getAButton():
                self.inOutCommand = InOutCommand.kIntaking
            elif self.ctrl.getBButton():
                self.inOutCommand = InOutCommand.kOutaking
            elif self.ctrl.getRightBumper():
                self.inOutCommand = InOutCommand.kShooting
            else:
                self.inOutCommand = InOutCommand.kOff

            if self.ctrl.getXButton():
                self.flywheelCommand = FlywheelCommand.kSpinningUp
            elif self.ctrl.getYButton():
                self.flywheelCommand = FlywheelCommand.kSpinningDown
            else:
                self.flywheelCommand = FlywheelCommand.kNoCommand

        else:
            # If the joystick is unplugged, pick safe-state xyzzy and raise a fault
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
                self.inOutCommand = InOutCommand.kOff
                self.flywheelCommand = FlywheelCommand.kNoCommand