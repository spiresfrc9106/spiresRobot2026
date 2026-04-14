import os

import wpilib

from constants import LoggerState, RobotTypes, RobotModes, RobotIdentification
from pykit.logger import Logger
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from utils.singleton import Singleton


class RobotLoggerSetup(metaclass=Singleton):
    """Configures pykit Logger for the robot."""

    robotName: str | None = None

    def __init__(self) -> None:
        self._logWriters: list[WPILOGWriter] = []
        self.useTiming: bool = True

        # For Logger and robotpy test to work RobotLoggerSetup should be first called from robot.__init__
        # This restriction limits the configuration of a robot that can be replayed through logger
        # to happen within __init__ calls and not within the global part of packages.
        assert self.robotName is not None, (
            "RobotLoggerSetup should be first called from robot.__init__"
        )

        Logger.recordMetadata("Robot", self.robotName)
        print(
            f"Robot Logger Setup: {self.robotName}, pid={os.getpid()} LoggerState().kRobotMode={LoggerState().kRobotMode}"
        )
        match LoggerState().kRobotMode:
            case RobotModes.REAL | RobotModes.SIMULATION:
                Logger.recordMetadata(
                    "RobotType", RobotIdentification().getRobotTypeStr()
                )
                deployConfig = wpilib.deployinfo.getDeployData()
                if deployConfig is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deployConfig.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deployConfig.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deployConfig.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deployConfig.get("code-path", "")
                    )
                    Logger.recordMetadata("Git Hash", deployConfig.get("git-hash", ""))
                    Logger.recordMetadata(
                        "Git Branch", deployConfig.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deployConfig.get("git-desc", "")
                    )
                Logger.addDataReciever(NT4Publisher(True))
                writer = WPILOGWriter()
                self._logWriters.append(writer)
                Logger.addDataReciever(writer)
            case RobotModes.REPLAY:
                self.useTiming = False
                logPath = LoggerState().logPath
                assert logPath is not None, "Log path not set"
                logPath = os.path.abspath(logPath)
                print(f"Starting log from {logPath}")
                replaySource = WPILOGReader(logPath)
                Logger.setReplaySource(replaySource)
                writer = WPILOGWriter(logPath[:-7] + "_sim.wpilog")
                self._logWriters.append(writer)
                Logger.addDataReciever(writer)

        Logger.start()

        match LoggerState().kRobotMode:
            case RobotModes.REAL | RobotModes.SIMULATION:
                self.robotTypeStr = Logger.metadata["RobotType"]
            case RobotModes.REPLAY:
                self.robotTypeStr = Logger.entry.getSubTable("RealMetadata").get(
                    "RobotType", None
                )
        self.robotType = RobotTypes[self.robotTypeStr]
        pass

    @property
    def logFiles(self) -> list[str]:
        """Returns the current absolute path(s) of log files created by this setup."""
        return [os.path.join(w.folder, w.filename) for w in self._logWriters]
