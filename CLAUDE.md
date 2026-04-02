# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 9106 (Spires) robot code for the 2026 season. Written in Python using RobotPy (WPILib's Python bindings) with the commands2 framework. Forked from FRC#1736 RobotCasserole2026 with PyKit logging from FRC#1757 Westwood Robotics.

## Key Commands for the human that invokes claude

Prior to starting claude, the human that invokes claude, the cwd must be the top level directory
of this repo and must have activated the python virtual environment:

```bash
. .venv/bin/activate
```

if this has happened correctly then both the command:
```bash
.venv/bin/python -c "import sys; print(sys.prefix)"
```
and the command:
```bash
python -c "import sys; print(sys.prefix)"
```

should return the same directory path that ends in "/.venv", e.g. "/Users/mikestitt/Documents/first/2026/sw/spiresRobot2026/.venv"

## Commands the human must run, and Claude may not run:

"uv" commands. For example:
```bash
# Sync dependencies (after pyproject.toml changes)
uv sync
```

"ssh" commands like:
```bash
# ssh into the roborio:
ssh lvuser@roboRIO-9106-frc.local
```

"git" commands that change the state of git. For example: `git checkout`, `git commit`, `git push`, `git switch`.

"robotpy" commands that deploy code to the robot. For example: `robotpy deploy`

Claude may not invoke `robotpy deploy`, regardless of the methods to reach `robotpy`

```bash
# Claude may not run these:
robotpy deploy
robotpy deploy --skip-tests
.venv/bin/robotpy deploy 
.venv/bin/robotpy deploy --skip-tests
python -m robotpy deploy
python -m robotpy deploy --skip-tests
.venv/bin/python -m robotpy deploy 
.venv/bin/python -m robotpy deploy --skip-tests
uv run robotpy deploy
uv run robotpy deploy --skip-tests
uv run -- robotpy deploy
uv run -- robotpy deploy --skip-tests
uv run python -m robotpy deploy
uv run python -m robotpy deploy --skip-tests
uv run -- python -m robotpy deploy
uv run -- python -m robotpy deploy --skip-tests
```

## Commands that claude may run:

```bash
# Run tests
.venv/bin/robotpy test
# Run this test after: ".venv/bin/robotpy test" to test replay of a simulation
.vevn/bin/robotpy test .\tests\replay_test.py::test_log_and_replay_step3 -- --runreplay

# -- Run tests - other useful permutations:
.venv/bin/robotpy coverage test
.venv/bin/robotpy coverage test -- --no-header -vvv
.venv/bin/robotpy coverage test -- --no-header -vvv -s
.venv/bin/robotpy test -- --no-header -vvv
.venv/bin/robotpy test -- --no-header -vvv -s
.vevn/bin/robotpy test .\tests\replay_test.py::test_log_and_replay_step3 -- --runreplay --no-header -vvv
.vevn/bin/robotpy test .\tests\replay_test.py::test_log_and_replay_step3 -- --runreplay --no-header -vvv -s


# Run simulator
.venv/bin/robotpy sim

# Lint / type checks (must all pass before merging)
.venv/bin/ruff format          # auto-format
.venv/bin/ruff check           # lint
.venv/bin/ruff check --fix     # lint with auto-fix
.venv/bin/mypy .               # type check

# Network utilities
.venv/bin/netconsole roboRIO-9106-frc.local
```

## CI Requirements

Claude should ensure that these three formatting checks pass before completing a coding task. All three must pass on every push/PR (checked by `.github/workflows/lint.yml`):

```bash
# versions of the three formating checks that Claude can run:
.venv/bin/ruff check
.venv/bin/ruff format --check --diff
.venv/bin/mypy .
# equivalent versions of the three that are invoked by the `.github/workflows/lint.yml`
uv run ruff check
uv run ruff format --check --diff
uv run mypy .
```

## Project Structure

```
robot.py                   # Entry point — LoggedRobot, minimal lifecycle
robotcontainer.py          # Subsystem wiring, command/button bindings
robotstate.py              # Shared robot state (pose, module positions)
physics.py                 # Simulation physics
constants/                 # Per-subsystem constants (k-prefix classes)
drivetrain/                # Swerve drive logic (command, control, pose estimation)
subsystems/
  common/                  # IO abstraction layer (motormoduleio, encodermoduleio)
  drivetrain/              # Drivetrain subsystem with IO variants (real/sim)
  vision/                  # Vision subsystem with IO variants (limelight/photon/sim)
  state/                   # Robot state, config, top-level subsystem
  intakeOuttake/           # Intake/outtake subsystem
humanInterface/            # Driver/operator interface, LED control
sensors/                   # Limelight sensor wrapper
wrappers/                  # Hardware wrappers (SparkMax, SparkFlex, Kraken, Gyro, etc.)
util/                      # Shared utilities (math, logging, joystick, etc.)
utils/                     # More utilities (calibration, faults, field layout, etc.)
navigation/                # Repulsor field planner
pykit/                     # Logging library (WPILog, NetworkTables, NT4)
tests/                     # pyfrc tests
deploy/                    # Files deployed to robot (PathPlanner paths, etc.)
```

## Architecture Patterns

### IO Abstraction Layer

Hardware is abstracted behind IO interfaces (similar to AdvantageKit). Each subsystem has:
- `*io.py` — abstract base class defining the interface
- `*iowrappered.py` — real hardware implementation
- `*iowrapperedsim.py` — simulation implementation

Example: `subsystems/common/motormoduleio.py`, `motormoduleiowrappered.py`, `motormoduleiowrapperedsim.py`

This allows the same subsystem logic to run in simulation without real hardware.

### Constants

- Constants live in `constants/` with one file per subsystem area
- Use `k`-prefix class names: `kDriveConfig`, `kVisionConfig`, etc.
- Constants are plain class attributes — no `__init__`, no instantiation
- `constants/__init__.py` re-exports commonly used constants

### Logging (PyKit)

- Uses `pykit` library for structured WPILog logging
- `WPILOGWriter` writes logs; `WPILOGReader` replays them
- `NT4Publisher` publishes to NetworkTables
- Inherit from `LoggedRobot` (in `pykit/loggedrobot.py`) in `robot.py`
- Use `Logger` for subsystem-level logging

### Motor Wrappers

Motor controllers are wrapped in `wrappers/`:
- `wrapperedKraken.py` — TalonFX (Kraken X60)
- `wrapperedSparkMax.py` — REV SparkMax (NEO, NEO 550)
- `wrapperedSparkFlex.py` — REV SparkFlex (NEO Vortex)
- `wrapperedMotorSuper.py` — Abstract base class
- `wrapperedGyro.py` — Gyroscope wrapper (NavX)

All motor wrappers extend `WrapperedMotorSuper`.

### Vision

Vision has multiple IO implementations:
- `visioniophoton.py` — PhotonVision camera
- `visioniolimelight.py` — Limelight camera
- `visioniophotonsim.py` — PhotonVision simulation
- `visioniosim.py` — Generic simulation

Camera configurations are set up in `subsystems/vision/vision.py`.

## Commands2 Patterns

### Every Command subclass must:
```python
class MyCommand(commands2.Command):
    def __init__(self, subsystem: MySubsystem):
        super().__init__()   # REQUIRED — must be first
        self.addRequirements(subsystem)

    def initialize(self): ...
    def execute(self): ...
    def isFinished(self) -> bool: return False
    def end(self, interrupted: bool): ...
```

### Every Subsystem subclass must:
```python
class MySubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()   # REQUIRED — must be first
```

## Type Annotations

- All function parameters and return types should be annotated
- Use `Optional[X]` or `X | None` for nullable types — never `def foo(x: float = None)`
- Abstract/interface methods that return nothing use `...` as body (not `pass`)
- mypy is run in CI — all type errors must be resolved

## Python Version

- Robot runs Python 3.14 on the roboRIO
- `match`/`case` syntax is safe to use
- `pyproject.toml` sets `requires-python = "==3.14.*"`

## Pre-commit

Pre-commit hooks run ruff on commit. To run manually:
```bash
uv run -- pre-commit run --all-files
```

## Simulation

- `physics.py` contains `PhysicsEngine` for drivetrain simulation
- Use `wpilib.RobotBase.isSimulation()` to guard sim-only code
- `simulationPeriodic()` is called automatically by the scheduler in sim only
- Simulate with `uv run -- robotpy sim`
- Use Glass/AdvantageScope for visualization

## Testing

- Tests live in `tests/`
- Run with `uv run -- robotpy test`
- Use sim feedback when testing hardware-dependent code
- `uv run -- robotpy coverage test` for coverage reports
