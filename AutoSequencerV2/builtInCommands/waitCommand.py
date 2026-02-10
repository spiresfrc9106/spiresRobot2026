import wpilib

from AutoSequencerV2.command import Command
from subsystems.state.robottopsubsystem import RobotTopSubsystem


class WaitCommand(Command):
    def __init__(self, waitDur):
        #when you run the WaitCommand, you must pass in a waiter duration to use
        self.waitDur = waitDur
        self.endTime = 0

    def initialize(self):
        #this happens when we actually start running this command
        self.endTime = RobotTopSubsystem().getFPGATimestampS() + self.waitDur

    def isDone(self):
        return RobotTopSubsystem().getFPGATimestampS() >= self.endTime

    def getName(self):
        return f"Wait {self.waitDur}s"
