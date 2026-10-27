#!/usr/bin/env python3
"""Fail-closed check of the v0.17 *unvalidated* marine surrogate sweep."""
import csv
import io
import math
from pathlib import Path
import sys

ENVIRONMENTS = {"calm", "current", "wind"}
PARAMETERS = {
    "mass_kg", "surge_linear_drag_ns_per_m", "sway_linear_drag_ns_per_m",
    "yaw_drag_nms_per_rad", "wind_force_gain", "propulsor_half_spacing_m",
}
FIELDS = [
    "environment", "parameter", "perturbation_fraction", "baseline_drift_m",
    "low_drift_m", "high_drift_m", "central_sensitivity_m",
    "baseline_yaw_change_rad",
]


def validate_csv(content: str) -> dict:
    reader = csv.DictReader(io.StringIO(content))
    if reader.fieldnames != FIELDS:
        raise ValueError("unexpected sensitivity CSV schema")
    keys = set()
    baselines = {}
    responsive = 0
    for row in reader:
        if None in row or any(row[field] is None for field in FIELDS):
            raise ValueError("malformed sensitivity CSV row")
        environment, parameter = row["environment"], row["parameter"]
        key = (environment, parameter)
        if environment not in ENVIRONMENTS or parameter not in PARAMETERS or key in keys:
            raise ValueError(f"invalid or duplicate sensitivity key: {key}")
        keys.add(key)
        try:
            p, baseline, low, high, slope, yaw = (
                float(row[k]) for k in FIELDS[2:]
            )
        except (ValueError, TypeError) as exc:
            raise ValueError(f"invalid sensitivity number: {key}") from exc
        if not all(map(math.isfinite, (p, baseline, low, high, slope, yaw))):
            raise ValueError(f"nonfinite sensitivity: {key}")
        if abs(p - 0.2) > 1e-9 or min(baseline, low, high) < 0.0:
            raise ValueError(f"invalid perturbation or negative displacement: {key}")
        if abs(slope - (high-low)/(2*p)) > 2e-7:
            raise ValueError(f"inconsistent finite difference: {key}")
        if abs(slope) > 0.05:
            responsive += 1
        if environment in baselines and abs(baselines[environment] - baseline) > 1e-7:
            raise ValueError(f"baseline changed within a profile: {environment}")
        baselines[environment] = baseline
    if keys != {(env, param) for env in ENVIRONMENTS for param in PARAMETERS}:
        raise ValueError("incomplete sensitivity matrix")
    if responsive < 5:
        raise ValueError("model sensitivity not excited")
    if (abs(baselines["calm"]-baselines["current"]) <= 0.1 or
            abs(baselines["calm"]-baselines["wind"]) <= 0.1):
        raise ValueError("environmental profiles indistinguishable")
    return {"source": "uncalibrated deterministic C++ 3DOF surrogate",
            "cases": len(keys), "responsive": responsive,
            "baseline_post_neutral_drift_m": baselines}


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: verify_sensitivity_csv.py file.csv|-", file=sys.stderr)
        return 2
    try:
        content = (sys.stdin.read() if sys.argv[1] == "-" else
                   Path(sys.argv[1]).read_text(encoding="utf-8"))
        report = validate_csv(content)
    except (OSError, ValueError) as exc:
        print(f"FAIL_MARINE_SENSITIVITY: {exc}", file=sys.stderr)
        return 1
    print("PASS_MARINE_SENSITIVITY", report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
