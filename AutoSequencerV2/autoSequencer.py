from wpimath.geometry import Pose2d
from wpilib import DriverStation
from AutoSequencerV2.modeList import ModeList
from AutoSequencerV2.builtInModes.doNothingMode import DoNothingMode
from AutoSequencerV2.builtInModes.waitMode import WaitMode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.modes.driveForwardSlowly import DriveForwardSlowly
from Autonomous.modes.driveOut import DriveOut
from utils.singleton import Singleton
from utils.allianceTransformUtils import onRed
from utils.autonomousTransformUtils import setFlip
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from commands2 import Command

class AutoSequencer(metaclass=Singleton):
    """Top-level implementation of the AutoSequencer"""

    def __init__(self):
        # Have different delay modes for delaying the start of autonomous


        self.delayModeChooser: LoggedDashboardChooser[Command] = (
            LoggedDashboardChooser("Delay")
        )
        self.delayModeChooser.setDefaultOption("Wait 0", WaitMode(0.0))
        self.delayModeChooser.addOption("Wait 3", WaitMode(3.0))
        self.delayModeChooser.addOption("Wait 6", WaitMode(6.0))
        self.delayModeChooser.addOption("Wait 9", WaitMode(9.0))
        self._oldDelayMode = self.delayModeChooser.getSelected()

        self.flipModeChooser: LoggedDashboardChooser[Command] = (
            LoggedDashboardChooser("Flip")
        )
        self.flipModeChooser.setDefaultOption("Left", "Left")
        self.flipModeChooser.addOption("Right", "Right")
        self._oldFlipMode = self.flipModeChooser.getSelected()



        self.mainModeChooser: LoggedDashboardChooser[Command] = (
            LoggedDashboardChooser("Main Drive")
        )
        self.mainModeChooser.setDefaultOption("Do Nothing", DoNothingMode())
        self.mainModeChooser.addOption("Drive Out", DriveOut())
        self._oldMainMode = self.mainModeChooser.getSelected()

        self.topLevelCmdGroup = SequentialCommandGroup()
        self.startPose = Pose2d()

        # Alliance changes require us to re-plan autonomous
        # This variable is used to help track when alliance changes
        self._prevOnRed = onRed()

        self.updateMode(force=True)  # Ensure we load the auto sequencer at least once.

    # Returns true if the alliance has changed since the last call
    def _allianceChanged(self):
        curRed = onRed()
        retVal = curRed != self._prevOnRed
        self._prevOnRed = curRed
        return retVal


    # Call this periodically while disabled to keep the dashboard updated
    # and, when things change, re-init modes
    def updateMode(self, force=False):
        flipMode = self.flipModeChooser.getSelected()
        mainMode = self.mainModeChooser.getSelected()
        delayMode = self.delayModeChooser.getSelected()
        if (force or
                self._allianceChanged() or
                self._oldDelayMode != delayMode or
                self._oldFlipMode != flipMode or
                self._oldMainMode != mainMode):

            setFlip(flipMode == "Right")
            mainMode.__init__()
            if delayMode is None or mainMode is None or flipMode is None:
                self.topLevelCmdGroup = DoNothingMode()
                self.startPose = Pose2d()
            else:
                self.topLevelCmdGroup = delayMode.getCmdGroup().andThen(
                    mainMode.getCmdGroup()
                )
                self.startPose = mainMode.getInitialDrivetrainPose()
                print(
                    f"[Auto] New Modes Selected: {DriverStation.getAlliance()} {delayMode.getName()} {flipMode} {mainMode.getName()} {self.startPose}"
            )
            self._oldDelayMode = delayMode
            self._oldFlipMode = flipMode
            self._oldMainMode = mainMode

    # Call this once during autonmous init to init the current command sequence
    def initialize(self):
        self.updateMode() # Last-shot update before starting autonomous
        print("[Auto] Starting Sequencer")
        self.topLevelCmdGroup.initialize()

    def update(self):
        self.topLevelCmdGroup.execute()

    def end(self):
        self.topLevelCmdGroup.end(True)
        print("[Auto] Sequencer Stopped")

    def getStartingPose(self) -> Pose2d | None:
        # Returns the initial pose of the auto routine if it has a defined one.
        return self.startPose
