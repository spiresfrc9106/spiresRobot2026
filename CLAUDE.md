# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 9106 (Spires) robot code for the 2026 season. Written in Python using RobotPy (WPILib's Python bindings) with the commands2 framework. Forked from FRC#1736 RobotCasserole2026 with PyKit logging from FRC#1757 Westwood Robotics.

## Key Commands

```bash
# Sync dependencies (after pyproject.toml changes)
uv sync
uv run robotpy sync

# Run tests
uv run -- robotpy test
uv run -- robotpy coverage test -- --no-header -vvv -s

# Run simulator
uv run -- robotpy sim

# Deploy to robot (must be connected to robot network)
uv run robotpy deploy --skip-tests

# Lint / type checks (must all pass before merging)
uv run -- ruff format          # auto-format
uv run -- ruff check           # lint
uv run -- ruff check --fix     # lint with auto-fix
uv run -- mypy .               # type check

# Network utilities
uv run netconsole roboRIO-9106-frc.local
ssh lvuser@roboRIO-9106-frc.local
```

## CI Requirements

All three must pass on every push/PR (checked by `.github/workflows/lint.yml`):
```bash
uv run -- ruff check
uv run -- ruff format --check --diff
uv run -- mypy .
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

- Robot runs Python 3.12 on the roboRIO
- `match`/`case` syntax is safe to use
- `pyproject.toml` sets `requires-python = "==3.12.*"`

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
