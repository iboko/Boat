#!/usr/bin/env python3
"""Validate v0.20 paired instant/windowed identification comparisons."""
import csv
import math
import sys
from pathlib import Path

COLUMNS = (
    "profile", "parameter", "window_samples", "trials", "instant_accepted",
    "windowed_accepted", "truth", "instant_mean", "windowed_mean",
    "instant_relative_bias", "windowed_relative_bias", "windowed_p05",
    "windowed_p95",
)
PROFILES = ("ideal", "low", "moderate")
PARAMETERS = (
    "mass_kg", "surge_linear_drag_ns_per_m", "yaw_inertia_kgm2",
    "yaw_drag_nms_per_rad",
)


def verify(content: str) -> None:
    rows = list(csv.DictReader(content.splitlines(), strict=True))
    if len(rows) != 12 or not rows or tuple(rows[0]) != COLUMNS:
        raise ValueError("expected 12 rows with exact v0.20 schema")
    seen = set()
    for row in rows:
        if None in row or any(v is None or v == "" for v in row.values()):
            raise ValueError("missing or extra CSV fields")
        name = (row["profile"], row["parameter"])
        if name in seen or name[0] not in PROFILES or name[1] not in PARAMETERS:
            raise ValueError("unknown or duplicate comparison")
        seen.add(name)
        nums = {field: float(row[field]) for field in COLUMNS[2:]}
        if any(not math.isfinite(v) for v in nums.values()):
            raise ValueError("nonfinite result")
        if nums["window_samples"] != 25 or nums["trials"] != 60:
            raise ValueError("wrong experiment settings")
        if any(not nums[key].is_integer() or nums[key] <= 0 or
               nums[key] > nums["trials"] for key in
               ("instant_accepted", "windowed_accepted")):
            raise ValueError("invalid acceptance counts")
        if (nums["truth"] <= 0 or nums["instant_mean"] <= 0 or
            nums["windowed_mean"] <= 0 or nums["windowed_p05"] <= 0 or
            nums["windowed_p95"] < nums["windowed_p05"]):
            raise ValueError("nonphysical estimate or invalid percentile order")
        if (abs(nums["instant_mean"]/nums["truth"]-1.0-
                nums["instant_relative_bias"]) > 1e-7 or
            abs(nums["windowed_mean"]/nums["truth"]-1.0-
                nums["windowed_relative_bias"]) > 1e-7):
            raise ValueError("reported relative bias does not match means")
        if name[0] == "ideal" and (
            abs(nums["instant_relative_bias"]) > 1e-6 or
            abs(nums["windowed_relative_bias"]) > 1e-6 or
            abs(nums["windowed_p95"]-nums["windowed_p05"]) > 1e-6 or
            nums["instant_accepted"] != 60 or
            nums["windowed_accepted"] != 60
        ):
            raise ValueError("noiseless control failed")
    if seen != {(a,b) for a in PROFILES for b in PARAMETERS}:
        raise ValueError("incomplete comparison matrix")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: verify_windowed_csv.py FILE|-", file=sys.stderr)
        return 2
    try:
        content = sys.stdin.read() if sys.argv[1] == "-" else Path(sys.argv[1]).read_text()
        verify(content)
    except (OSError, ValueError, csv.Error) as error:
        print(f"FAIL_WINDOWED_CSV: {error}", file=sys.stderr)
        return 1
    print("PASS: complete paired marine windowed regression comparison")
    return 0


if __name__ == "__main__":
    sys.exit(main())
