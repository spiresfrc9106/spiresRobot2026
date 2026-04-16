"""
Plots camera pose fix verification from a camera_path_test CSV.

Usage:
    uv run --with matplotlib python tools/plot_vision_fix.py                  # newest pyLogs/*.csv
    uv run --with matplotlib python tools/plot_vision_fix.py pyLogs/file.csv  # specific file

Saves: tools/vision_fix_plots.png

Three series are compared against sol/0/pose (multi-tag ground truth):
  - bestRobotField    — solvePnP "best" solution
  - alternateRobotField — solvePnP "alternate" solution
  - closest           — whichever of best/alternate is nearer to sol/0/pose

If the "closest" error is much smaller than "best" error, it means the correct
pose is present in one of the two solutions but the ambiguity selector is
choosing the wrong one. If "closest" is still large, the camera transform
constants may need calibration.
"""

import csv
import math
import sys
from glob import glob
from pathlib import Path

import matplotlib.pyplot as plt

# --- Column keys ---
COL_TIME = "Timestamp"
COL_TAG_ID = "/RealOutputs/back_center_cam/target/0/id"
COL_BEST_X = "/RealOutputs/back_center_cam/target/0/bestRobotField/translation/x"
COL_BEST_Y = "/RealOutputs/back_center_cam/target/0/bestRobotField/translation/y"
COL_ALT_X = "/RealOutputs/back_center_cam/target/0/alternateRobotField/translation/x"
COL_ALT_Y = "/RealOutputs/back_center_cam/target/0/alternateRobotField/translation/y"
COL_SOL_X = "/RealOutputs/back_center_cam/sol/0/pose/translation/x"
COL_SOL_Y = "/RealOutputs/back_center_cam/sol/0/pose/translation/y"
COL_EST_X = "/RealOutputs/Robot/Pose/Estimator/Pose/translation/x"
COL_EST_Y = "/RealOutputs/Robot/Pose/Estimator/Pose/translation/y"


def pick_csv() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    candidates = glob("pyLogs/*.csv")
    if not candidates:
        sys.exit("No CSV files found in pyLogs/. Run camera_path_test first.")
    return Path(max(candidates, key=lambda p: Path(p).stat().st_mtime))


def safe_float(val: str) -> float | None:
    try:
        return float(val)
    except ValueError, TypeError:
        return None


def load(csv_path: Path) -> dict:
    print(f"Loading {csv_path}")
    times_all, est_x_all, est_y_all = [], [], []
    times_tag = []
    best_x, best_y = [], []
    alt_x, alt_y = [], []
    sol_x, sol_y = [], []

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = safe_float(row.get(COL_TIME, ""))
            ex = safe_float(row.get(COL_EST_X, ""))
            ey = safe_float(row.get(COL_EST_Y, ""))
            if t is not None and ex is not None and ey is not None:
                times_all.append(t)
                est_x_all.append(ex)
                est_y_all.append(ey)

            tag_id = row.get(COL_TAG_ID, "").strip()
            if tag_id and tag_id != "0":
                bx = safe_float(row.get(COL_BEST_X, ""))
                by = safe_float(row.get(COL_BEST_Y, ""))
                ax_ = safe_float(row.get(COL_ALT_X, ""))
                ay = safe_float(row.get(COL_ALT_Y, ""))
                sx = safe_float(row.get(COL_SOL_X, ""))
                sy = safe_float(row.get(COL_SOL_Y, ""))
                if None not in (t, bx, by, ax_, ay, sx, sy):
                    times_tag.append(t)
                    best_x.append(bx)
                    best_y.append(by)
                    alt_x.append(ax_)
                    alt_y.append(ay)
                    sol_x.append(sx)
                    sol_y.append(sy)

    print(f"  {len(times_all)} total rows, {len(times_tag)} rows with tag detected")

    # Build "closest" series: whichever of best/alternate is nearer to sol/0/pose
    closest_x, closest_y = [], []
    for bx, by, ax_, ay, sx, sy in zip(best_x, best_y, alt_x, alt_y, sol_x, sol_y):
        if math.hypot(bx - sx, by - sy) <= math.hypot(ax_ - sx, ay - sy):
            closest_x.append(bx)
            closest_y.append(by)
        else:
            closest_x.append(ax_)
            closest_y.append(ay)

    return dict(
        times_all=times_all,
        est_x=est_x_all,
        est_y=est_y_all,
        times_tag=times_tag,
        best_x=best_x,
        best_y=best_y,
        alt_x=alt_x,
        alt_y=alt_y,
        closest_x=closest_x,
        closest_y=closest_y,
        sol_x=sol_x,
        sol_y=sol_y,
    )


def mean(vals: list[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def plot(data: dict, csv_path: Path) -> None:
    err_best = [
        math.hypot(bx - sx, by - sy)
        for bx, by, sx, sy in zip(
            data["best_x"], data["best_y"], data["sol_x"], data["sol_y"]
        )
    ]
    err_closest = [
        math.hypot(cx - sx, cy - sy)
        for cx, cy, sx, sy in zip(
            data["closest_x"], data["closest_y"], data["sol_x"], data["sol_y"]
        )
    ]

    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle(
        f"Vision fix verification — {csv_path.name}\n"
        "Green=sol/0/pose (reference) | Red=bestRobotField | Orange=closest(best,alternate)",
        fontsize=10,
    )

    # --- Subplot 1: XY trajectory ---
    ax = axes[0]
    ax.plot(
        data["est_x"],
        data["est_y"],
        "b-",
        linewidth=1.5,
        label="Estimator (ref)",
        zorder=1,
    )
    ax.scatter(
        data["sol_x"],
        data["sol_y"],
        c="green",
        s=20,
        zorder=4,
        label="sol/0/pose (multi-tag)",
    )
    ax.scatter(
        data["best_x"],
        data["best_y"],
        c="red",
        s=15,
        zorder=2,
        alpha=0.6,
        label="bestRobotField",
    )
    ax.scatter(
        data["closest_x"],
        data["closest_y"],
        c="orange",
        s=15,
        zorder=3,
        alpha=0.8,
        label="closest(best, alternate)",
    )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # --- Subplot 2: distance from sol/0/pose vs time ---
    ax = axes[1]
    ax.scatter(
        data["times_tag"],
        err_best,
        c="red",
        s=12,
        alpha=0.6,
        label=f"bestRobotField (mean={mean(err_best):.2f}m)",
    )
    ax.scatter(
        data["times_tag"],
        err_closest,
        c="orange",
        s=12,
        alpha=0.8,
        label=f"closest(best,alt) (mean={mean(err_closest):.2f}m)",
    )
    ax.axhline(0.5, color="gray", linestyle="--", label="0.5m threshold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance from sol/0/pose (m)")
    ax.set_title("Distance from Navigation Solution vs Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    out = Path(__file__).parent / "vision_fix_plots.png"
    fig.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.show()


if __name__ == "__main__":
    csv_path = pick_csv()
    data = load(csv_path)
    if not data["times_tag"]:
        sys.exit(
            "No rows with tags detected. Check COL_TAG_ID or run the path test again."
        )
    plot(data, csv_path)
