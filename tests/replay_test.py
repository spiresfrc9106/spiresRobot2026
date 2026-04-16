"""
Test that the robot can pyKit sim and replay the sim
"""

import math
import os
import shutil
from typing import Tuple

import pytest
import wpilib
from wpiutil.log import DataLogReader
from constants import LoggerState
from pykit.wpilog.wpilogwriter import WPILOGWriter
from tests.controllerTestPyKitReplay import PyKitReplayTestController


# Keys whose values legitimately differ between real and replay
_SKIP_PREFIXES = (
    "Logger/",
    "LoggedRobot/",
    "SystemStats/",
    "Console",
    "LogTracer/",
)

# Keys containing these substrings are skipped regardless of prefix.
# /sol/ — logged by WrapperedPoseEstPhotonCamera which runs only in sim, not replay.
_SKIP_SUBSTRINGS = ("/sol/",)

step2InputLogName = "test_log_and_replay_step2.wpilog"
globalStep2Path = os.path.join(WPILOGWriter.defaultPathSim, step2InputLogName)
replayLogPath = os.path.join(
    WPILOGWriter.defaultPathSim, "test_log_and_replay_step2_sim.wpilog"
)


@pytest.fixture()
def forceRobotInReplay():
    LoggerState().logPath = globalStep2Path
    print("-----------------------")
    print("-----------------------")
    print(f"-----------------------Forcing replay mode LOG_PATH: {globalStep2Path}\n\n")
    print("-----------------------")
    print("-----------------------")


@pytest.fixture(scope="function")
def pykitReplayControl(reraise, robot: wpilib.RobotBase) -> PyKitReplayTestController:
    """
    A pytest fixture that provides control over your robot
    """
    return PyKitReplayTestController(reraise, robot)


@pytest.mark.order(1)
def test_log_and_replay_step1(pykitReplayControl, robot):
    # -----------------------------------------------------------------------
    # Phase 1: sim run
    # -----------------------------------------------------------------------
    # Set c_backup as the active auto so the chooser logs it and replay can replay it.
    # RobotContainer adds non-side autos with a "C-" prefix ("C-c_backup").
    robot.container.autoChooser.sendableChooser.setDefaultOption(
        "C-c_backup", "C-c_backup"
    )

    with pykitReplayControl.run_robot():
        pykitReplayControl.step_timing(seconds=0.5, autonomous=True, enabled=False)
        pykitReplayControl.step_timing(seconds=15.0, autonomous=True, enabled=True)
        pykitReplayControl.step_timing(seconds=0.5, autonomous=True, enabled=False)

    step1LogPath = robot.loggerSetup.logFiles[0]

    step1Dir = os.path.dirname(step1LogPath)
    step2Path = os.path.join(step1Dir, step2InputLogName)
    shutil.copyfile(step1LogPath, step2Path)

    assert os.path.exists(step2Path), f"Log file not created: {step2Path}"


@pytest.mark.order(2)
def test_log_and_replay_step2(forceRobotInReplay, pykitReplayControl, robot):
    # -----------------------------------------------------------------------
    # Phase 2: replay the pykit log
    # -----------------------------------------------------------------------

    with pykitReplayControl.run_robot():
        pykitReplayControl.step_timing(
            seconds=0.5, autonomous=True, enabled=False, assert_alive=False
        )
        pykitReplayControl.step_timing(
            seconds=15.0, autonomous=True, enabled=True, assert_alive=False
        )
        pykitReplayControl.step_timing(
            seconds=0.5, autonomous=True, enabled=False, assert_alive=False
        )

    assert os.path.exists(replayLogPath), f"Replay log not created: {replayLogPath}"

    # -----------------------------------------------------------------------
    # Compare logs
    # -----------------------------------------------------------------------
    _compareLogFiles(globalStep2Path, replayLogPath)


# ---------------------------------------------------------------------------
# Log comparison helpers
# ---------------------------------------------------------------------------


def _readLogEntries(
    path: str, prefix: str
) -> Tuple[dict[str, list], dict[str, list[int]]]:
    """Read all entries under `prefix`, return {bare_key: [val, ...]}."""
    reader = DataLogReader(path)
    entryNames: dict[int, str] = {}
    result: dict[str, list] = {}
    times_us: dict[str, list[int]] = {}

    for record in reader:
        if record.isControl():
            if record.isStart():
                sd = record.getStartData()
                if sd.name.startswith(prefix):
                    entryNames[sd.entry] = sd.name[len(prefix) :]
        else:
            name = entryNames.get(record.getEntry())
            if name is None:
                continue
            if any(name.startswith(skip) for skip in _SKIP_PREFIXES):
                continue
            if any(sub in name for sub in _SKIP_SUBSTRINGS):
                continue
            result.setdefault(name, [])
            times_us.setdefault(name, [])
            for getter in (
                record.getDouble,
                record.getFloat,
                record.getInteger,
                record.getBoolean,
                record.getString,
                record.getDoubleArray,
            ):
                try:
                    result[name].append(getter())
                    times_us[name].append(record.getTimestamp())
                    break
                except Exception:
                    continue

    return (result, times_us)


def _compareLogFiles(origPath: str, replayPath: str, tol: float = 1e-2) -> None:
    realOut, realTimesDict = _readLogEntries(origPath, "/RealOutputs/")
    replayOut, replayTimesDict = _readLogEntries(replayPath, "/ReplayOutputs/")

    missingKeys = set(realOut) - set(replayOut)
    assert not missingKeys, (
        f"Keys in RealOutputs missing from ReplayOutputs: {missingKeys}"
    )

    countMismatch = False
    for key, realVals in realOut.items():
        replayVals = replayOut[key]
        realTimes_us = realTimesDict[key]
        replayTimes_us = replayTimesDict[key]
        for i, (rv, pv, rTime_us, pTime_us) in enumerate(
            zip(realVals, replayVals, realTimes_us, replayTimes_us)
        ):
            # TODO we need a better way to control absolute and relative tolerance by type of value.
            if isinstance(rv, float):
                assert math.isclose(rv, pv, rel_tol=tol, abs_tol=tol), (
                    f"{key}[{i}]: real={rv} at {rTime_us}, replay={pv} at {pTime_us}"
                )
            elif isinstance(rv, list):
                for j, (re, pe) in enumerate(zip(rv, pv)):
                    if isinstance(re, float):
                        assert math.isclose(re, pe, rel_tol=tol, abs_tol=tol), (
                            f"{key}[{i}][{j}]: real={re} at {rTime_us}, replay={pe} at {pTime_us}"
                        )
            else:
                assert rv == pv, (
                    f"{key}[{i}]: real={rv} at {rTime_us}, replay={pv} at {pTime_us}"
                )
            assert rTime_us == pTime_us, f"{key}[{i}]: at {rTime_us}, at {pTime_us}"
        lenRealVals = len(realVals)
        lenReplayVals = len(replayVals)
        if lenRealVals != lenReplayVals:
            countMismatch = True
            print(f"Cycle count mismatch for {key}: {lenRealVals} vs {lenReplayVals}")
    assert not countMismatch, "A count mismatch occurred."
