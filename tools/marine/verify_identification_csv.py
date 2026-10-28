#!/usr/bin/env python3
"""Strict CI verification of the *noiseless synthetic* v0.18 experiment."""
import csv
import math
import sys
from pathlib import Path

NAMES = (
    "mass_true_kg", "mass_estimated_kg",
    "surge_drag_true_ns_per_m", "surge_drag_estimated_ns_per_m",
    "yaw_inertia_true_kgm2", "yaw_inertia_estimated_kgm2",
    "yaw_drag_true_nms_per_rad", "yaw_drag_estimated_nms_per_rad",
    "surge_excitation", "yaw_excitation", "surge_fit_rmse_mps2",
    "yaw_fit_rmse_radps2", "holdout_position_rmse_m", "holdout_heading_rmse_rad",
)


def verify(content: str) -> None:
    rows = list(csv.reader(content.splitlines(), strict=True))
    if len(rows) != 2 or tuple(rows[0]) != NAMES or len(rows[1]) != len(NAMES):
        raise ValueError("expected one result row and the exact v0.18 CSV header")
    values = {}
    for name, raw in zip(NAMES, rows[1]):
        if not raw or raw.strip() != raw:
            raise ValueError(f"empty or padded value: {name}")
        number = float(raw)
        if not math.isfinite(number):
            raise ValueError(f"nonfinite value: {name}")
        values[name] = number
    for label, truth, estimated in (
        ("mass", "mass_true_kg", "mass_estimated_kg"),
        ("surge_drag", "surge_drag_true_ns_per_m", "surge_drag_estimated_ns_per_m"),
        ("yaw_inertia", "yaw_inertia_true_kgm2", "yaw_inertia_estimated_kgm2"),
        ("yaw_drag", "yaw_drag_true_nms_per_rad", "yaw_drag_estimated_nms_per_rad"),
    ):
        if values[truth] <= 0 or values[estimated] <= 0:
            raise ValueError(f"invalid {label} coefficient")
        if abs(values[truth] - values[estimated]) > 1e-5 * values[truth]:
            raise ValueError(f"failed noiseless {label} recovery")
    for name in ("surge_excitation", "yaw_excitation"):
        if not 1e-3 < values[name] <= 1.0:
            raise ValueError(f"insufficient {name}")
    for name in NAMES[-4:]:
        if not 0.0 <= values[name] < 1e-5:
            raise ValueError(f"excessive noiseless {name}")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: verify_identification_csv.py FILE|-", file=sys.stderr)
        return 2
    try:
        content = sys.stdin.read() if sys.argv[1] == "-" else Path(sys.argv[1]).read_text()
        verify(content)
    except (OSError, ValueError, csv.Error) as error:
        print(f"FAIL_MARINE_IDENTIFICATION_CSV: {error}", file=sys.stderr)
        return 1
    print("PASS: synthetic marine identification coefficients and holdout")
    return 0


if __name__ == "__main__":
    sys.exit(main())
