# spiresRobot2026

Main 2026 Robot Code for FRC Team 9106 - Spires

---

Thank you to our sponsors!

This repository started as a fork of [FRC#1736 RobotCasserole2026](https://github.com/RobotCasserole1736/RobotCasserole2026). 
Then PyKit logging from [FRC#1757 Westwood Robotics 2026-Rebuilt](https://github.com/1757WestwoodRobotics/2026-Rebuilt).
We think we might see if the software can be configured to use either Robot Casserole Swerve or Westwood Robotics Swerve

We are grateful to teams #1736 and #1757 for the inspiration!

![ci Test Status](https://github.com/spiresfrc9106/spiresRobot2026/actions/workflows/ci.yml/badge.svg?branch=main)
![lint Status](https://github.com/spiresfrc9106/spiresRobot2026/actions/workflows/lint.yml/badge.svg?branch=main)

## Installation

Before developing code on a new computer:

1. [Install wpilib](https://github.com/wpilibsuite/allwpilib/releases)
2. [Install python 3.14](https://www.python.org/downloads/)

On Windows use the Python Installation Manager:
```
Welcome to the Python installation manager configuration helper.

********************************************************************************

Windows is not configured to allow paths longer than 260 characters.

... Some packages may fail to install without long path support
enabled.
Update setting now? [y/N] y
The setting has been successfully updated, and will take effect after the next reboot.

********************************************************************************

The global shortcuts directory is not configured.

Configuring this enables commands like python3.14.exe to run from your
terminal ...
Add commands directory to your PATH now? [y/N] y
PATH has been updated, and will take effect after opening a new terminal.

********************************************************************************

You do not have the latest Python runtime. ...

Install CPython now? [Y/n] y
Python install manager was successfully updated to 26.0.

...

Find additional information at https://docs.python.org/dev/using/windows.

View online help? [y/N] N
```
3. [Install git](https://git-scm.com/install/windows)
4. [Install GitHub CLI](https://cli.github.com/)
5. [Install uv](https://docs.astral.sh/uv/getting-started/installation/)
6. On Windows, we suggest installing this reposistory in 'C:\spires'
   1. Open a Terminal Window
   2. `cd \`
   3. `mkdir spires`
   4. `cd spires`
   5. `gh repo clone spiresfrc9106/spiresRobot2026`
   6. `cd spiresRobot2026`
   7. `uv sync`
   8. `uv run robotpy sync`
   9. `uv run robotpy test`

## Development

Useful commands after cd to this directory `cd \spires\spiresRobot2026`
* After a change to [pyproject.toml](https://github.com/spiresfrc9106/spiresRobot2026/blob/main/pyproject.toml)
   ```
   uv sync
   uv run robotpy sync
   ```
* To reformat code to follow coding standards:
   ```
   uv run ruff format
   ```
* To check if code follows type hinting coding standard:
   ```
   uv run mypy .
   ```
* To run automated tests within the development computer:
   ```
   uv run robotpy test
   ```
* To run the robot code in a simulator within the development computer:
   ```
   uv run robotpy sim
   ```
* To deploy the code to the computer within the robot.
(This computer must be connected to the robot network via Wi-Fi or Ethernet.)
(FRC Driver Station must be running and connected to the robot network.)
  ```
  uv run robotpy deploy --skip-tests
  ```


## Docs

- [High level code design patterns](.docs/designPatterns)
- [Commonly used modules in this repository](.docs/commonModules)
- [Most recent relationship diagram between classes](.docs/graph.md)
    - Keep this file up to date by periodically running `codeStructureReportGen/reportGen.py`

  
## Interesting links

[RobotPy source code](https://github.com/robotpy/mostrobotpy)

[PyTest Docs](https://docs.pytest.org/en/7.4.x/)

[PyTest Examples](https://pytest.org/en/7.4.x/example/index.html)

### Deploy Notes

`uv run robotpy deploy --skip-tests` to avoid requiring tests to pass before deployment can proceed. This is helpful for quick iterations, but don't make it a bad habit.

`.deploy_cfg` contains specific configuration about the deploy process.

Any folder or file prefixed with a `.` will be skipped in the deploy. This is good to avoid sending unnecessary files to the resource limited RoboRIO like documentation and images.

## Linting

"Linting" is the process of checking our code format and style to keep it looking nice and avoid unnecessary inconsistencies.

```
uv run ruff format --check
uv run ruff format
uv run mypy .
```

## Testing

`uv run robotpy test`

OR to get details of what is happening during the tests:

`uv run robotpy test -- --no-header -vvv -s`

OR to run a specific test file from the '.\tests' directory:

`uv run robotpy test autoSequencer_test.py -- --no-header -vvv -s`

OR to run a specific test file and test case from the '.\tests' directory:

`uv run robotpy test autoSequencer_test.py::test_parallel -- --no-header -vvv -s`

OR to skip a specific test:

`uv run robotpy test -- -k 'not pyfrc_test.py'`


## Useful commands:

See network logs: `uv run netconsole roboRIO-9106-frc.local` or `uv run netconsole 10.91.6.2`

SSH into the robot: `ssh lvuser@roboRIO-9106-frc.local` or `ssh lvuser@10.91.6.2`

```
powershell {$Env:LOG_PATH="C:\spires\spiresRobot2026\pyLogs\pykit_20260209_132514.wpilog" ; $Env:LOG_PATH; uv run -- robotpy sim }
```

## Roborio 2.0 image install

Use balenaEtcher to install the roborio image

The 2024 roboio 2.0 image is here:

C:\Program Files (x86)\National Instruments\LabVIEW 2023\project\roboRIO Tool\FRC Images\SD Images

use roborio team number setter to set the team number

## Notes
