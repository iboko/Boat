#!/usr/bin/env python3
"""Compare actual Gazebo odometry after HOLD for controlled flow cases.

Never infer a force from an SDF parameter alone. Require the model's raw
odometry, verified ArduPilot HOLD and a nonzero Gazebo plugin force log.
"""
import argparse
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

from verify_blueboat_trial import load_truth
from prepare_blueboat_world import ENVIRONMENTS, PLUGIN_NAME

VECTOR = re.compile(
    r"BOAT_ENV_FORCES_APPLIED force=\s*([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)"
)


def hold_vector(directory: Path, profile: str) -> dict:
    mission = json.loads((directory / "dynamic_mission_report.json").read_text())
    verification = json.loads(
        (directory / "gazebo_blueboat_verification.json").read_text()
    )
    if mission.get("bridge_mode") != "onboard-blackout" or not mission.get("hold_confirmed"):
        raise ValueError(f"{profile}: no onboard GNSS blackout HOLD")
    if not verification.get("hold_confirmed") or not verification.get("not_rover_simstate_only"):
        raise ValueError(f"{profile}: no independently verified Gazebo HOLD")
    if verification.get("hold_drift_window_s", 0.0) < 19.0:
        raise ValueError(f"{profile}: insufficient post-HOLD duration")
    truth = load_truth(directory / "gazebo_blueboat_truth.jsonl")
    hold_at = mission.get("hold_confirmed_monotonic_s")
    if not isinstance(hold_at, (int, float)) or not math.isfinite(hold_at):
        raise ValueError(f"{profile}: invalid HOLD timestamp")
    points = [p for p in truth if p["monotonic_s"] >= hold_at]
    if len(points) < 8:
        raise ValueError(f"{profile}: no real Gazebo samples after HOLD")
    delta = (points[-1]["x"] - points[0]["x"],
             points[-1]["y"] - points[0]["y"])
    if not all(math.isfinite(x) for x in delta):
        raise ValueError(f"{profile}: invalid Gazebo displacement")
    if profile != "calm":
        sdf = ET.parse(directory / "waves_with_blueboat.sdf").getroot()
        node = sdf.find(f"world/plugin[@name='{PLUGIN_NAME}']")
        if node is None:
            raise ValueError(f"{profile}: no Gazebo world force plugin")
        values = tuple(float(node.findtext(field)) for field in
                       ("current_x", "current_y", "wind_x", "wind_y"))
        if values != ENVIRONMENTS[profile]:
            raise ValueError(f"{profile}: mismatched flow configuration")
        log = (directory / "gazebo.log").read_text()
        if ("BOAT_ENV_FORCES_CONFIGURED" not in log or
                "BOAT_ENV_FORCES_CONFIG_FAILED" in log or
                "BOAT_ENV_FORCES_INVALID_VELOCITY" in log):
            raise ValueError(f"{profile}: plugin not loaded correctly")
        match = VECTOR.search(log)
        if not match:
            raise ValueError(f"{profile}: no Gazebo force application log")
        force = tuple(float(x) for x in match.groups())
        if not all(math.isfinite(x) for x in force) or math.hypot(*force[:2]) < 0.1:
            raise ValueError(f"{profile}: force not physically nonzero")
    else:
        if ET.parse(directory / "waves_with_blueboat.sdf").getroot().find(
                f"world/plugin[@name='{PLUGIN_NAME}']") is not None:
            raise ValueError("calm: unexpected forcing plugin")
        force = (0.0, 0.0, 0.0)
    return {"profile": profile, "post_hold_displacement_xy_m": delta,
            "post_hold_distance_m": round(math.hypot(*delta), 3),
            "first_applied_force_xyz_n": force,
            "actual_gazebo_samples": len(points)}


def compare(root: Path) -> dict:
    profiles = {
        "calm": hold_vector(root / "onboard-hold", "calm"),
        "current": hold_vector(root / "onboard-hold-current", "current"),
        "wind": hold_vector(root / "onboard-hold-wind", "wind"),
    }
    baseline = profiles["calm"]["post_hold_displacement_xy_m"]
    for name in ("current", "wind"):
        result = profiles[name]
        delta = result["post_hold_displacement_xy_m"]
        difference = math.hypot(delta[0] - baseline[0], delta[1] - baseline[1])
        result["vector_difference_from_calm_m"] = round(difference, 3)
        # Must measure a response in Gazebo rather than echo parameters.
        if difference < 0.5:
            raise ValueError(f"{name}: no measurable post-HOLD response against calm")
    return {"source": "actual Gazebo BlueBoat model odometry",
            "observations": profiles,
            "note": "Uncalibrated experimental loads; not a validated sea-current/wind model"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = compare(args.root)
        args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as error:
        sys.exit("FAIL: " + str(error))
    print("BLUEBOAT_GAZEBO_ENVIRONMENT_MATRIX_PASS " + json.dumps(result))


if __name__ == "__main__":
    main()
