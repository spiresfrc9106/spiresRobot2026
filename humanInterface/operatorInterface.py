from utils.faults import Fault
from wpilib import DriverStation, XboxController
from utils.mapLookup2d import MapLookup2D
from utils.singleton import Singleton


class OperatorInterface(metaclass=Singleton):
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.intake = False
        self.outtake = False
        self.shoot = False
        self.spinUpFlywheel = False
        self.spinDownFlywheel = False
        

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
                self.intake = True
                self.outtake = False
                self.shoot = False
            elif self.ctrl.getBButton():
                self.intake = False
                self.outtake = True
                self.shoot = False
            elif self.ctrl.getRightBumper():
                self.intake = False
                self.outtake = False
                self.shoot = True
            else:
                self.intake = False
                self.outtake = False
                self.shoot = False

            if self.ctrl.getXButton():
                self.spinUpFlywheel = True
                self.spinDownFlywheel = False
            elif self.ctrl.getYButton():
                self.spinUpFlywheel = False
                self.spinDownFlywheel = True
            else:
                self.spinUpFlywheel = False
                self.spinDownFlywheel = False

        else:
            # If the joystick is unplugged, pick safe-state xyzzy and raise a fault
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
                self.intake = False
                self.outtake = False
                self.shoot = False
                self.spinUpFlywheel = False
                self.spinDownFlywheel = False