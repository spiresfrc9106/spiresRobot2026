"""
Runs the robot through c_backup in sim, then replays the resulting log
and verifies that RealOutputs ≈ ReplayOutputs (determinism check).

Run:
    uv run -- robotpy test tests/replay_test.py -s
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
from utils.singleton import _instances


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

origLogName = "test_log_and_replay_step3.wpilog"
origLogPath = os.path.join(WPILOGWriter.defaultPathSim, origLogName)
replayLogPath = os.path.join(
    WPILOGWriter.defaultPathSim, "test_log_and_replay_step3_sim.wpilog"
)


@pytest.fixture()
def forceRobotInReplay():
    LoggerState().logPath = origLogPath
    print("-----------------------")
    print("-----------------------")
    print(f"\n\n-----------------------Forcing replay mode LOG_PATH: {origLogPath}\n\n")
    print("-----------------------")
    print("-----------------------")


@pytest.fixture(scope="function")
def pykitReplayControl(reraise, robot: wpilib.RobotBase) -> PyKitReplayTestController:
    """
    A pytest fixture that provides control over your robot
    """
    return PyKitReplayTestController(reraise, robot)


# @pytest.mark.dependency(name="test_log_and_replay_step1")
@pytest.mark.order(1)
def test_log_and_replay_step1(control, robot):
    # -----------------------------------------------------------------------
    # Phase 1: sim run
    # -----------------------------------------------------------------------
    # Set c_backup as the active auto so the chooser logs it and replay can replay it.
    # RobotContainer adds non-side autos with a "C-" prefix ("C-c_backup").
    robot.container.autoChooser.sendableChooser.setDefaultOption(
        "C-c_backup", "C-c_backup"
    )

    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        control.step_timing(seconds=15.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        global origLogPath
        origLogPath = robot.loggerSetup.logFiles[0]

        originalDir = os.path.dirname(origLogPath)  # r'c:\temp'
        newName = "test_log_and_replay_step3.wpilog"
        newPath = os.path.join(originalDir, newName)
        shutil.copyfile(origLogPath, newPath)
        global replayLogPath
        replayLogPath = origLogPath[:-7] + "_sim.wpilog"

    assert os.path.exists(origLogPath), f"Log file not created: {origLogPath}"


@pytest.mark.order(2)
def test_step2():
    print(f"test_step2:instances={_instances}")


# @pytest.mark.dependency(depends=["test_log_and_replay_step1"])
@pytest.mark.order(3)
@pytest.mark.replay
def test_log_and_replay_step3(forceRobotInReplay, pykitReplayControl, robot):
    # -----------------------------------------------------------------------
    # Phase 2: replay in a subprocess so REVLib/HAL device registries are fresh
    # -----------------------------------------------------------------------
    # robot.container.autoChooser.sendableChooser.setDefaultOption(
    #    "C-c_backup", "C-c_backup"
    # )

    # todo project_root = pathlib.Path(__file__).parent.parent

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
    _compareLogFiles(origLogPath, replayLogPath)


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


def _compareLogFiles(origPath: str, replayPath: str, tol: float = 1e-4) -> None:
    realOut, realTimesDict = _readLogEntries(origPath, "/RealOutputs/")
    replayOut, replayTimesDict = _readLogEntries(replayPath, "/ReplayOutputs/")

    missingKeys = set(realOut) - set(replayOut)
    assert not missingKeys, (
        f"Keys in RealOutputs missing from ReplayOutputs: {missingKeys}"
    )

    for key, realVals in realOut.items():
        replayVals = replayOut[key]
        realTimes_us = realTimesDict[key]
        replayTimes_us = replayTimesDict[key]
        for i, (rv, pv, rTime_us, pTime_us) in enumerate(
            zip(realVals, replayVals, realTimes_us, replayTimes_us)
        ):
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
        assert lenRealVals == lenReplayVals, (
            f"Cycle count mismatch for {key}: {lenRealVals} vs {lenReplayVals}"
        )
