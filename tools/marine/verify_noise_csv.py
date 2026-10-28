#!/usr/bin/env python3
"""Check completeness and internal consistency of the v0.19 noise study."""
import csv
import math
import sys
from pathlib import Path

COLUMNS = (
    "profile", "parameter", "speed_sigma_mps", "yaw_rate_sigma_radps",
    "heading_sigma_rad", "trials", "accepted", "rejected", "truth", "mean",
    "relative_bias", "p05", "p95",
)
PARAMETERS = (
    "mass_kg", "surge_linear_drag_ns_per_m", "yaw_inertia_kgm2",
    "yaw_drag_nms_per_rad",
)
PROFILES = {
    "ideal": (0.0, 0.0, 0.0),
    "low": (0.001, 0.0005, 0.001),
    "moderate": (0.005, 0.002, 0.005),
}


def verify(content: str) -> None:
    rows = list(csv.DictReader(content.splitlines(), strict=True))
    if len(rows) != 12 or not rows or tuple(rows[0].keys()) != COLUMNS:
        raise ValueError("expected exactly 12 rows of the v0.19 schema")
    seen = set()
    for row in rows:
        profile, parameter = row["profile"], row["parameter"]
        if profile not in PROFILES or parameter not in PARAMETERS:
            raise ValueError("unknown noise profile or parameter")
        key = (profile, parameter)
        if key in seen:
            raise ValueError("duplicate noise profile/parameter")
        seen.add(key)
        if None in row or any(value is None or value == "" for value in row.values()):
            raise ValueError("missing CSV value")
        nums = {}
        for field in COLUMNS[2:]:
            nums[field] = float(row[field])
            if not math.isfinite(nums[field]):
                raise ValueError("nonfinite CSV value")
        expected = PROFILES[profile]
        if any(abs(nums[name] - value) > 1e-10 for name, value in zip(
            ("speed_sigma_mps", "yaw_rate_sigma_radps", "heading_sigma_rad"),
            expected,
        )):
            raise ValueError("wrong noise profile coefficients")
        if any(nums[name] != 60 for name in ("trials",)):
            raise ValueError("wrong Monte Carlo trial count")
        if (not nums["accepted"].is_integer() or
                not nums["rejected"].is_integer() or
                nums["accepted"] <= 0 or nums["rejected"] < 0 or
                nums["accepted"] + nums["rejected"] != nums["trials"]):
            raise ValueError("invalid accepted/rejected accounting")
        if nums["truth"] <= 0 or nums["mean"] <= 0:
            raise ValueError("nonphysical parameter")
        if abs(nums["relative_bias"] -
               (nums["mean"] / nums["truth"] - 1)) > 1e-7:
            raise ValueError("incorrect bias calculation")
        if not 0 < nums["p05"] <= nums["p95"] or nums["mean"] <= 0:
            raise ValueError("invalid percentile/mean ordering")
        if profile == "ideal" and (
            abs(nums["relative_bias"]) > 1e-6 or
            abs(nums["p95"] - nums["p05"]) > 1e-6 or
            nums["rejected"] != 0
        ):
            raise ValueError("noiseless control failed")
    if seen != {(a, b) for a in PROFILES for b in PARAMETERS}:
        raise ValueError("incomplete noise result matrix")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: verify_noise_csv.py FILE|-", file=sys.stderr)
        return 2
    try:
        content = sys.stdin.read() if sys.argv[1] == "-" else Path(sys.argv[1]).read_text()
        verify(content)
    except (OSError, ValueError, csv.Error) as error:
        print(f"FAIL_MARINE_NOISE_CSV: {error}", file=sys.stderr)
        return 1
    print("PASS: 12-row marine noise analysis matrix")
    return 0


if __name__ == "__main__":
    sys.exit(main())
