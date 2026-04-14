from pykit.logger import Logger, RobotController


class LogTracerEnable:
    enable: bool = False
    innerStart: float = 0.0
    outerStart: float = 0.0

    prefix: str = ""

    @classmethod
    def resetOuter(cls, prefix: str) -> None:
        cls.outerStart = RobotController.getFPGATime()
        cls.reset()
        cls.prefix = prefix

    @classmethod
    def reset(cls) -> None:
        cls.innerStart = RobotController.getFPGATime()

    @classmethod
    def record(cls, action: str) -> None:
        now = RobotController.getFPGATime()
        Logger.recordOutput(
            f"LogTracer/{cls.prefix}/{action}MS", (now - cls.innerStart) / 1000.0
        )
        cls.innerStart = now

    @classmethod
    def recordTotal(cls) -> None:
        now = RobotController.getFPGATime()
        Logger.recordOutput(
            f"LogTracer/{cls.prefix}/TotalMS", (now - cls.outerStart) / 1000.0
        )

class LogTracerDisable:
    @classmethod
    def resetOuter(cls, prefix: str) -> None:
        pass

    @classmethod
    def reset(cls) -> None:
        pass

    @classmethod
    def record(cls, action: str) -> None:
        pass

    @classmethod
    def recordTotal(cls) -> None:
        pass

if True:
    LogTracer: LogTracerEnable|LogTracerDisable = LogTracerDisable # type: ignore
