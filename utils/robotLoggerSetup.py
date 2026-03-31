import os

import wpilib

import constants
from pykit.logger import Logger
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader


class RobotLoggerSetup:
    """Configures pykit Logger for the robot and tracks created log files."""

    def __init__(self, robotName: str) -> None:
        self._logWriters: list[WPILOGWriter] = []
        self._useTiming: bool = True
        Logger.recordMetadata("Robot", robotName)
        match constants.kRobotMode:
            case constants.RobotModes.REAL | constants.RobotModes.SIMULATION:
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
            case constants.RobotModes.REPLAY:
                self._useTiming = False
                logPath = os.environ["LOG_PATH"]
                logPath = os.path.abspath(logPath)
                print(f"Starting log from {logPath}")
                replaySource = WPILOGReader(logPath)
                Logger.setReplaySource(replaySource)
                writer = WPILOGWriter(logPath[:-7] + "_sim.wpilog")
                self._logWriters.append(writer)
                Logger.addDataReciever(writer)
        Logger.start()

    @property
    def logFiles(self) -> list[str]:
        """Returns the current absolute path(s) of log files created by this setup."""
        return [os.path.join(w.folder, w.filename) for w in self._logWriters]

    @property
    def useTiming(self) -> bool:
        return self._useTiming
