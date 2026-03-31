"""
Subprocess replay runner invoked by replay_test.py Phase 2.

Usage:
    python tests/_replay_runner.py <origLogPath>

Runs MyRobot in REPLAY mode using the given log, then exits.  The resulting
_sim.wpilog is written next to origLogPath by RobotLoggerSetup.
"""

import os
import pathlib
import sys

# Project root is two levels up from this file (tests/_replay_runner.py).
_PROJECT_ROOT = pathlib.Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(_PROJECT_ROOT))

# Tell robotpy/wpilib where robot.py lives so that getDeployDirectory() points
# at <project_root>/deploy instead of <tests>/deploy.
import robotpy.main  # noqa: E402

robotpy.main.robot_py_path = _PROJECT_ROOT / "robot.py"

# HAL must be initialised in simulation mode before any wpilib/robot imports.
import hal  # noqa: E402

hal.initialize(500, 0)

# Now safe to import robot code.
import constants  # noqa: E402
from robot import MyRobot  # noqa: E402


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: _replay_runner.py <origLogPath>", file=sys.stderr)
        sys.exit(1)

    origLogPath = sys.argv[1]
    constants.kRobotMode = constants.RobotModes.REPLAY
    os.environ["LOG_PATH"] = origLogPath

    robot = MyRobot()
    try:
        robot.startCompetition()
    except SystemExit:
        pass
    finally:
        robot.endCompetition()


if __name__ == "__main__":
    main()
