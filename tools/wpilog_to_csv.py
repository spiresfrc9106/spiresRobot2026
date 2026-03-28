"""
tools/wpilog_to_csv.py
Converts a WPILib .wpilog file to CSV, expanding struct types to sub-columns.
Matches AdvantageScope's CSV export format.

Usage:
    uv run python tools/wpilog_to_csv.py                       # newest pyLogs/*.wpilog
    uv run python tools/wpilog_to_csv.py pyLogs/file.wpilog    # specific file
    uv run python tools/wpilog_to_csv.py pyLogs/file.wpilog out.csv

pandas:
    from tools.wpilog_to_csv import to_dataframe
    df = to_dataframe("pyLogs/file.wpilog")
"""

import csv
import re
import struct as st
import sys
from glob import glob
from pathlib import Path
from typing import Any

from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Primitive type table: type_name -> (struct_format, byte_size)
# All little-endian.
# ---------------------------------------------------------------------------
_PRIM: dict[str, tuple[str, int]] = {
    "bool":   ("<B", 1),  # stored as uint8
    "char":   ("<B", 1),
    "int8":   ("<b", 1),
    "uint8":  ("<B", 1),
    "int16":  ("<h", 2),
    "uint16": ("<H", 2),
    "int32":  ("<i", 4),
    "uint32": ("<I", 4),
    "int64":  ("<q", 8),
    "uint64": ("<Q", 8),
    "float":  ("<f", 4),
    "double": ("<d", 8),
}

# ---------------------------------------------------------------------------
# Schema parsing
# ---------------------------------------------------------------------------

def _parse_schema_fields(schema_str: str) -> list[tuple[str, str, int]]:
    """
    Parse a schema string like 'double x;double y;Translation2d translation'
    into [(field_name, type_name, count), ...].
    """
    fields = []
    for part in schema_str.split(";"):
        part = part.strip()
        if not part:
            continue
        tokens = part.split()
        if len(tokens) < 2:
            continue
        raw_type = tokens[0].split(":")[0]  # strip bit-field width (e.g. uint8:4 -> uint8)
        name = tokens[-1]
        m = re.match(r"(\w+)\[(\d+)\]", raw_type)
        if m:
            fields.append((name, m.group(1), int(m.group(2))))
        else:
            fields.append((name, raw_type, 1))
    return fields


def _build_leaf_fields(
    type_name: str,
    schemas: dict[str, list[tuple[str, str, int]]],
    prefix: str = "",
) -> list[tuple[str, str, int]]:
    """
    Recursively expand a type into leaf (path, fmt, byte_size) tuples.
    path uses '/' as separator, matching AdvantageScope sub-column naming.
    """
    if type_name in _PRIM:
        fmt, size = _PRIM[type_name]
        return [(prefix, fmt, size)]
    if type_name not in schemas:
        return []
    result = []
    for field_name, field_type, count in schemas[type_name]:
        sub = f"{prefix}/{field_name}" if prefix else field_name
        if count == 1:
            result.extend(_build_leaf_fields(field_type, schemas, sub))
        else:
            for idx in range(count):
                result.extend(_build_leaf_fields(field_type, schemas, f"{sub}[{idx}]"))
    return result


def _decode_struct(
    data: bytes,
    type_name: str,
    schemas: dict[str, list[tuple[str, str, int]]],
) -> dict[str, Any]:
    """Decode raw struct bytes into {sub_path: value} dict."""
    leaves = _build_leaf_fields(type_name, schemas)
    result: dict[str, Any] = {}
    offset = 0
    for path, fmt, size in leaves:
        if offset + size > len(data):
            break
        result[path] = st.unpack_from(fmt, data, offset)[0]
        offset += size
    return result


# ---------------------------------------------------------------------------
# Log reading
# ---------------------------------------------------------------------------

def _read_log(
    wpilog_path: Path,
) -> tuple[list[float], dict[str, list[tuple[float, Any]]]]:
    """
    Read a wpilog file.
    Returns:
        timestamps: sorted list of all unique timestamps (seconds)
        series: dict of column_name -> [(timestamp_sec, value), ...]
    """
    log = DataLogReader(str(wpilog_path))

    # entry_id -> (name, type_str)
    entries: dict[int, tuple[str, str]] = {}
    # struct type name (e.g. 'Pose2d') -> parsed fields
    schemas: dict[str, list[tuple[str, str, int]]] = {}
    # schema entry ids -> struct type name (so we can decode the schema data record)
    schema_entry_ids: dict[int, str] = {}

    # Pre-scan for all start records and schema data records
    for record in log:
        if record.isStart():
            sd = record.getStartData()
            entries[sd.entry] = (sd.name, sd.type)
            if sd.type == "structschema":
                # e.g. name='/.schema/struct:Pose2d'
                struct_name = sd.name.removeprefix("/.schema/struct:")
                schema_entry_ids[sd.entry] = struct_name
        elif not record.isFinish() and not record.isSetMetadata() and not record.isControl():
            eid = record.getEntry()
            if eid in schema_entry_ids:
                schema_str = bytes(record.getRaw()).decode("utf-8", errors="replace")
                struct_name = schema_entry_ids[eid]
                schemas[struct_name] = _parse_schema_fields(schema_str)

    # Second pass: collect data records
    timestamps_set: set[float] = set()
    series: dict[str, list[tuple[float, Any]]] = {}

    def _add(col: str, ts: float, val: Any) -> None:
        if col not in series:
            series[col] = []
        series[col].append((ts, val))
        timestamps_set.add(ts)

    for record in DataLogReader(str(wpilog_path)):
        if record.isStart() or record.isFinish() or record.isSetMetadata() or record.isControl():
            continue
        eid = record.getEntry()
        if eid not in entries or eid in schema_entry_ids:
            continue

        name, type_str = entries[eid]
        ts = record.getTimestamp() / 1_000_000.0  # microseconds -> seconds

        try:
            if type_str == "double":
                _add(name, ts, record.getDouble())
            elif type_str == "float":
                _add(name, ts, record.getFloat())
            elif type_str in ("int64", "int32", "int16", "int8",
                              "uint64", "uint32", "uint16", "uint8"):
                _add(name, ts, record.getInteger())
            elif type_str == "boolean":
                _add(name, ts, record.getBoolean())
            elif type_str == "string":
                _add(name, ts, record.getString())
            elif type_str == "double[]":
                for i, v in enumerate(record.getDoubleArray()):
                    _add(f"{name}[{i}]", ts, v)
            elif type_str == "float[]":
                for i, v in enumerate(record.getFloatArray()):
                    _add(f"{name}[{i}]", ts, v)
            elif type_str == "int64[]":
                for i, v in enumerate(record.getIntegerArray()):
                    _add(f"{name}[{i}]", ts, v)
            elif type_str == "boolean[]":
                for i, v in enumerate(record.getBooleanArray()):
                    _add(f"{name}[{i}]", ts, v)
            elif type_str == "string[]":
                for i, v in enumerate(record.getStringArray()):
                    _add(f"{name}[{i}]", ts, v)
            elif type_str.startswith("struct:"):
                struct_name = type_str.removeprefix("struct:")
                raw = bytes(record.getRaw())
                decoded = _decode_struct(raw, struct_name, schemas)
                for sub_path, val in decoded.items():
                    _add(f"{name}/{sub_path}", ts, val)
            # other types (raw bytes etc.) are skipped
        except Exception:
            pass

    timestamps = sorted(timestamps_set)
    return timestamps, series


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

def to_csv(wpilog_path: Path, csv_path: Path | None = None) -> Path:
    """Convert wpilog to CSV. Returns the path of the written CSV."""
    if csv_path is None:
        csv_path = wpilog_path.with_suffix(".csv")

    print(f"Reading {wpilog_path} ...")
    timestamps, series = _read_log(wpilog_path)

    columns = sorted(series.keys())
    print(f"  {len(timestamps)} timestamps, {len(columns)} columns")

    # Build per-column iterator with forward-fill
    col_iters = {col: iter(series[col]) for col in columns}
    col_next: dict[str, tuple[float, Any] | None] = {
        col: next(col_iters[col], None) for col in columns
    }
    col_current: dict[str, Any] = {}

    print(f"Writing {csv_path} ...")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp"] + columns)

        for ts in timestamps:
            # Advance iterators for all columns at this timestamp
            for col in columns:
                while col_next[col] is not None and col_next[col][0] <= ts:
                    col_current[col] = col_next[col][1]
                    col_next[col] = next(col_iters[col], None)
            row = [ts] + [col_current.get(col, "") for col in columns]
            writer.writerow(row)

    print(f"Done: {csv_path}")
    return csv_path


# ---------------------------------------------------------------------------
# pandas helper (optional — requires pandas to be installed)
# ---------------------------------------------------------------------------

def to_dataframe(wpilog_path: str | Path):  # type: ignore[return]
    """
    Read a wpilog and return a pandas DataFrame with forward-filled values.
    Requires: pip install pandas  (or: uv run --with pandas python ...)
    """
    try:
        import pandas as pd
    except ImportError:
        raise ImportError("pandas is required: uv run --with pandas python tools/wpilog_to_csv.py")

    timestamps, series = _read_log(Path(wpilog_path))

    # Build a dict of {col: pd.Series indexed by timestamp}
    data: dict[str, Any] = {"Timestamp": timestamps}
    for col, records in series.items():
        ts_list, val_list = zip(*records) if records else ([], [])
        s = pd.Series(val_list, index=ts_list, dtype=object)
        s = s.reindex(timestamps, method="ffill")
        data[col] = s.values

    return pd.DataFrame(data)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def _pick_wpilog() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    candidates = glob("pyLogs/*.wpilog")
    if not candidates:
        sys.exit("No .wpilog files found in pyLogs/")
    return Path(max(candidates, key=lambda p: Path(p).stat().st_mtime))


if __name__ == "__main__":
    wpilog = _pick_wpilog()
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    to_csv(wpilog, out)
