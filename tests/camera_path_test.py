"""
Drives the robot through c_backup path with vision sim active.
Captures camera data to pyLogs/ for analysis of the camera pose bug.

Run:  uv run -- robotpy test tests/camera_path_test.py -s
Then: convert the resulting pyLogs/*.wpilog to CSV for analysis.

Useful log keys to examine in the CSV:
  /RealOutputs/back_center_cam/target/0/bestCameraToTarget/translation/[x,y,z]
  /RealOutputs/back_center_cam/target/0/bestRobotField/translation/[x,y,z]
  /RealOutputs/back_center_cam/sol/0/pose/translation/[x,y]   <- correct multi-tag estimate
  /RealOutputs/VisionSim/Camback_center_camPose/translation/[x,y,z]  <- sim ground truth
  /RealOutputs/Robot/Pose/Estimator/Pose/translation/[x,y]
"""

from pathplannerlib.auto import PathPlannerAuto


def test_camera_path(control, robot):
    """Drive through c_backup path, capturing camera data to log."""
    auto_cmd = PathPlannerAuto("c_backup")
    robot.container.autoChooser.getSelected = lambda: auto_cmd

    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
        control.step_timing(seconds=15.0, autonomous=True, enabled=True)
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)
