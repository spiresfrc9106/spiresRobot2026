"""
tools/find_replay_divergence.py

Reads a _sim.wpilog (which contains both /RealOutputs/* and /ReplayOutputs/*)
and finds the first cycle where any monitored key diverges between the two.

Usage:
    python tools/find_replay_divergence.py                        # newest pyLogs/*_sim.wpilog
    python tools/find_replay_divergence.py pyLogs/foo_sim.wpilog  # specific file
    python tools/find_replay_divergence.py --config my.toml       # custom config
    python tools/find_replay_divergence.py --show-all             # all divergences, not just first

Config (tools/replay_divergence.toml loaded automatically if present):
    rel_tolerance = 1e-4

    skip_prefixes = ["Logger/", "LoggedRobot/", "SystemStats/", "Console", "LogTracer/"]
    skip_substrings = ["/sol/"]
    watch_keys = []   # if non-empty, only diff keys matching any substring here

    [rel_tolerances]
    "Robot/top/distanceToHubIn" = 1e-3   # per-key override (first substring match wins)
"""

from __future__ import annotations

import argparse
import math
import sys
import tomllib
from dataclasses import dataclass, field
from glob import glob
from pathlib import Path
from typing import Any

from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_DEFAULT_SKIP_PREFIXES: list[str] = [
    "Logger/",
    "LoggedRobot/",
    "SystemStats/",
    "Console",
    "LogTracer/",
]
_DEFAULT_SKIP_SUBSTRINGS: list[str] = ["/sol/"]
_DEFAULT_REL_TOL: float = 1e-10

_ABS_TOL = 1e-10


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class Config:
    rel_tolerance: float = _DEFAULT_REL_TOL
    skip_prefixes: list[str] = field(
        default_factory=lambda: list(_DEFAULT_SKIP_PREFIXES)
    )
    skip_substrings: list[str] = field(
        default_factory=lambda: list(_DEFAULT_SKIP_SUBSTRINGS)
    )
    watch_keys: list[str] = field(default_factory=list)
    rel_tolerances: dict[str, float] = field(default_factory=dict)


def _load_config(path: Path | None) -> Config:
    cfg = Config()
    if path is None:
        default = Path(__file__).parent / "replay_divergence.toml"
        if default.exists():
            path = default
        else:
            return cfg
    with open(path, "rb") as f:
        data = tomllib.load(f)
    if "rel_tolerance" in data:
        cfg.rel_tolerance = float(data["rel_tolerance"])
    if "skip_prefixes" in data:
        cfg.skip_prefixes = [str(s) for s in data["skip_prefixes"]]
    if "skip_substrings" in data:
        cfg.skip_substrings = [str(s) for s in data["skip_substrings"]]
    if "watch_keys" in data:
        cfg.watch_keys = [str(s) for s in data["watch_keys"]]
    if "rel_tolerances" in data:
        cfg.rel_tolerances = {
            str(k): float(v) for k, v in data["rel_tolerances"].items()
        }
    return cfg


def _tol_for_key(key: str, cfg: Config) -> float:
    for pattern, tol in cfg.rel_tolerances.items():
        if pattern in key:
            return tol
    return cfg.rel_tolerance


# ---------------------------------------------------------------------------
# Log reading
# ---------------------------------------------------------------------------

Value = float | int | bool | str | list[Any]


def _read_log(
    path: Path,
    cfg: Config,
) -> tuple[dict[str, list[tuple[float, Value]]], dict[str, list[tuple[float, Value]]]]:
    """
    Read a _sim.wpilog and return:
        real[bare_key]   = [(ts_sec, value), ...]   from /RealOutputs/
        replay[bare_key] = [(ts_sec, value), ...]   from /ReplayOutputs/
    Skips and watch_keys filters are applied here.
    """
    real: dict[str, list[tuple[float, Value]]] = {}
    replay: dict[str, list[tuple[float, Value]]] = {}

    # entry_id -> (bare_key, target_dict)
    entry_map: dict[int, tuple[str, dict[str, list[tuple[float, Value]]]]] = {}

    reader = DataLogReader(str(path))

    def _should_skip(bare: str) -> bool:
        if any(bare.startswith(p) for p in cfg.skip_prefixes):
            return True
        if any(sub in bare for sub in cfg.skip_substrings):
            return True
        if cfg.watch_keys and not any(w in bare for w in cfg.watch_keys):
            return True
        return False

    for record in reader:
        if record.isControl():
            if record.isStart():
                sd = record.getStartData()
                name: str = sd.name
                if name.startswith("/RealOutputs/"):
                    bare = name[len("/RealOutputs/") :]
                    if not _should_skip(bare):
                        entry_map[sd.entry] = (bare, real)
                elif name.startswith("/ReplayOutputs/"):
                    bare = name[len("/ReplayOutputs/") :]
                    if not _should_skip(bare):
                        entry_map[sd.entry] = (bare, replay)
            continue

        info = entry_map.get(record.getEntry())
        if info is None:
            continue
        bare, target = info
        ts = record.getTimestamp() / 1_000_000.0

        val: Value | None = None
        for getter in (
            record.getDouble,
            record.getFloat,
            record.getInteger,
            record.getBoolean,
            record.getString,
            record.getDoubleArray,
        ):
            try:
                val = getter()
                break
            except Exception:
                continue
        if val is None:
            continue

        target.setdefault(bare, []).append((ts, val))

    return real, replay


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------


@dataclass
class Divergence:
    cycle: int
    ts: float
    key: str
    real_val: Value
    replay_val: Value
    diff: float | None  # None for non-numeric


def _vals_close(rv: Value, pv: Value, tol: float) -> bool:
    if isinstance(rv, float) and isinstance(pv, float):
        return math.isclose(rv, pv, rel_tol=tol, abs_tol=_ABS_TOL)
    if isinstance(rv, list) and isinstance(pv, list):
        if len(rv) != len(pv):
            return False
        return all(
            math.isclose(float(a), float(b), rel_tol=tol, abs_tol=_ABS_TOL)
            if isinstance(a, float) and isinstance(b, float)
            else a == b
            for a, b in zip(rv, pv)
        )
    return rv == pv


def _numeric_diff(rv: Value, pv: Value) -> float | None:
    if isinstance(rv, (int, float)) and isinstance(pv, (int, float)):
        return abs(float(rv) - float(pv))
    return None


@dataclass
class TsGap:
    real_only: list[float]  # timestamps present in real but not replay
    replay_only: list[float]  # timestamps present in replay but not real
    first_ts: float  # earliest timestamp across both sides
    last_ts: float  # latest timestamp across both sides


def _compare(
    real: dict[str, list[tuple[float, Value]]],
    replay: dict[str, list[tuple[float, Value]]],
    cfg: Config,
    show_all: bool,
) -> tuple[list[Divergence], dict[str, tuple[int, int]], list[str], dict[str, TsGap]]:
    """
    Returns:
        divergences:      list of Divergence (all or first-per-key depending on show_all)
        count_mismatches: {key: (real_count, replay_count)}
        missing_keys:     keys in real but not replay
        ts_gaps:          {key: TsGap} for keys with unmatched timestamps
    """
    divergences: list[Divergence] = []
    count_mismatches: dict[str, tuple[int, int]] = {}
    missing_keys: list[str] = []
    ts_gaps: dict[str, TsGap] = {}

    all_keys = sorted(real.keys())
    for key in all_keys:
        if key not in replay:
            missing_keys.append(key)
            continue
        real_vals = real[key]
        replay_vals = replay[key]
        print(f"Comparing {key}... {len(real_vals)} {len(replay_vals)}")
        if len(real_vals) != len(replay_vals):
            count_mismatches[key] = (len(real_vals), len(replay_vals))
        tol = _tol_for_key(key, cfg)

        # Build dicts keyed by timestamp for aligned comparison.
        # Timestamps come from integer µs divided by 1e6, so the same wpilog
        # timestamp always produces the same float — exact dict keying is safe.
        real_dict: dict[float, Value] = {ts: v for ts, v in real_vals}
        replay_dict: dict[float, Value] = {ts: v for ts, v in replay_vals}
        real_ts = set(real_dict)
        replay_ts = set(replay_dict)

        real_only_ts = sorted(real_ts - replay_ts)
        replay_only_ts = sorted(replay_ts - real_ts)
        if real_only_ts or replay_only_ts:
            all_ts = sorted(real_ts | replay_ts)
            ts_gaps[key] = TsGap(
                real_only=real_only_ts,
                replay_only=replay_only_ts,
                first_ts=all_ts[0],
                last_ts=all_ts[-1],
            )

        common_ts = sorted(real_ts & replay_ts)
        for cycle, ts in enumerate(common_ts):
            rv = real_dict[ts]
            pv = replay_dict[ts]
            if not _vals_close(rv, pv, tol):
                d = Divergence(
                    cycle=cycle,
                    ts=ts,
                    key=key,
                    real_val=rv,
                    replay_val=pv,
                    diff=_numeric_diff(rv, pv),
                )
                divergences.append(d)
                if not show_all:
                    break

    divergences.sort(key=lambda d: d.cycle)
    return divergences, count_mismatches, missing_keys, ts_gaps


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------


def _fmt_val(v: Value) -> str:
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, list):
        inner = ", ".join(f"{x:.4g}" if isinstance(x, float) else str(x) for x in v[:4])
        suffix = ", ..." if len(v) > 4 else ""
        return f"[{inner}{suffix}]"
    return str(v)


def _fmt_diff(d: float | None) -> str:
    if d is None:
        return "n/a"
    if d == 0.0:
        return "0"
    return f"{d:.3e}"


def _fmt_ts_list(ts_list: list[float], max_show: int = 4) -> str:
    shown = ", ".join(f"{t:.3f}s" for t in ts_list[:max_show])
    suffix = (
        f", ... (+{len(ts_list) - max_show} more)" if len(ts_list) > max_show else ""
    )
    return f"[{shown}{suffix}]"


def _print_report(
    divergences: list[Divergence],
    count_mismatches: dict[str, tuple[int, int]],
    missing_keys: list[str],
    show_all: bool,
    real_key_count: int,
    replay_key_count: int,
    ts_gaps: dict[str, TsGap],
) -> None:
    print(f"  Real keys: {real_key_count}  Replay keys: {replay_key_count}")

    # Count mismatches
    if count_mismatches:
        print(f"\n=== Count mismatches ({len(count_mismatches)}) ===")
        for key, (rc, pc) in sorted(count_mismatches.items()):
            gap_info = ""
            if key in ts_gaps:
                gap = ts_gaps[key]
                gap_info = (
                    f"  (real-only={len(gap.real_only)}  replay-only={len(gap.replay_only)}"
                    f"  span={gap.first_ts:.3f}s–{gap.last_ts:.3f}s)"
                )
            print(f"  {key:<60}  real={rc}  replay={pc}{gap_info}")
    else:
        print("\n=== Count mismatches: (none) ===")

    # Timestamp gap detail
    if ts_gaps:
        print(f"\n=== Timestamp gaps ({len(ts_gaps)} keys) ===")
        for key, gap in sorted(ts_gaps.items()):
            print(f"  {key}")
            print(
                f"    span: {gap.first_ts:.3f}s – {gap.last_ts:.3f}s"
                f"  (real-only={len(gap.real_only)}  replay-only={len(gap.replay_only)})"
            )
            if gap.real_only:
                print(f"    real-only ts:   {_fmt_ts_list(gap.real_only)}")
            if gap.replay_only:
                print(f"    replay-only ts: {_fmt_ts_list(gap.replay_only)}")

    # Divergences
    label = (
        "All divergences" if show_all else "First divergence per key (sorted by cycle)"
    )
    if divergences:
        print(f"\n=== {label} ({len(divergences)}) ===")
        key_w = min(60, max(len(d.key) for d in divergences) + 2)
        val_w = 14
        header = (
            f"{'Cycle':>6}  {'Timestamp':>10}  "
            f"{'Key':<{key_w}}  {'Real':>{val_w}}  {'Replay':>{val_w}}  {'Diff':>10}"
        )
        print(header)
        print("-" * len(header))
        for d in divergences:
            print(
                f"{d.cycle:>6}  {d.ts:>9.3f}s  "
                f"{d.key:<{key_w}}  {_fmt_val(d.real_val):>{val_w}}  "
                f"{_fmt_val(d.replay_val):>{val_w}}  {_fmt_diff(d.diff):>10}"
            )
    else:
        print(f"\n=== {label}: (none) ===")

    # Missing keys
    if missing_keys:
        print(f"\n=== Keys missing from ReplayOutputs ({len(missing_keys)}) ===")
        for k in missing_keys:
            print(f"  {k}")
    else:
        print("\n=== Keys missing from ReplayOutputs: (none) ===")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _pick_wpilog() -> Path:
    candidates = glob("pyLogs/*_sim.wpilog")
    if not candidates:
        # fall back to any wpilog
        candidates = glob("pyLogs/*.wpilog")
    if not candidates:
        sys.exit("No .wpilog files found in pyLogs/")
    return Path(max(candidates, key=lambda p: Path(p).stat().st_mtime))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Find first divergence between /RealOutputs and /ReplayOutputs in a _sim.wpilog"
    )
    parser.add_argument(
        "log", nargs="?", help="Path to _sim.wpilog (default: newest in pyLogs/)"
    )
    parser.add_argument("--config", metavar="FILE.toml", help="TOML config file")
    parser.add_argument("--reltol", type=float, help="Global rel_tolerance override")
    parser.add_argument(
        "--show-all",
        action="store_true",
        help="Show all divergences (not just first per key)",
    )
    args = parser.parse_args()

    config_path = Path(args.config) if args.config else None
    cfg = _load_config(config_path)
    if args.reltol is not None:
        cfg.rel_tolerance = args.relcol
        cfg.rel_tolerances = {}  # CLI tol wins over per-key overrides

    log_path = Path(args.log) if args.log else _pick_wpilog()
    if not log_path.exists():
        sys.exit(f"File not found: {log_path}")

    print(f"Reading {log_path} ...")
    real, replay = _read_log(log_path, cfg)

    divergences, count_mismatches, missing_keys, ts_gaps = _compare(
        real, replay, cfg, args.show_all
    )
    _print_report(
        divergences,
        count_mismatches,
        missing_keys,
        args.show_all,
        len(real),
        len(replay),
        ts_gaps,
    )


if __name__ == "__main__":
    main()
