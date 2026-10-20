#!/usr/bin/env python3
"""Strict SITL-only acceptance audit for experimental multi-sensor C++ ESKF.

Consumes existing evidence files and never controls any vehicle.
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
rows = list(csv.DictReader((root / "dynamic_cpp_odometry.csv").open(
    newline="", encoding="utf-8")))
assert len(rows) >= 60, "insufficient C++ ESKF navigation samples"

errors = [float(r["estimate_error_m"]) for r in rows]
sigma = [float(r["eskf_position_sigma_m"]) for r in rows]
sent = [int(r["odometry_sent"]) for r in rows]
radar = [int(r["radar_fix"]) for r in rows]
dvl = [int(r["dvl_fix"]) for r in rows]
assert all(math.isfinite(v) and v >= 0 for v in errors + sigma)
assert max(errors) < 3.0, "synthetic C++ ESKF diverged >3m"
p95 = sorted(errors)[int(0.95 * (len(errors) - 1))]
assert p95 < 2.0, f"synthetic C++ ESKF P95 {p95:.2f}m exceeded"
assert sum(sent) >= 60, "too many rejected outbound MAVLink solutions"
assert 10 <= sum(radar) < len(rows), "radar not simulated at lower sample rate"
assert 10 <= sum(dvl) < len(rows), "DVL not simulated at lower sample rate"
assert any(r == 0 and d == 0 for r, d in zip(radar, dvl)), (
    "combined sensor-outage interval not exercised"
)

log = (root / "dynamic_bridge.log").read_text(encoding="utf-8")
match = re.search(
    r"SITL_SENSOR_ESKF packets=(\d+) blocked=(\d+) radar_fixes=(\d+) "
    r"dvl_fixes=(\d+) radar_dropouts=(\d+) dvl_dropouts=(\d+)",
    log)
assert match, "sensor-ESKF source and diagnostics missing"
packets, blocked, radar_count, dvl_count, missed_radar, missed_dvl = map(
    int, match.groups())
assert packets == sum(sent), "ODOMETRY counter does not match CSV"
assert radar_count == sum(radar) and dvl_count == sum(dvl), (
    "sensor counter does not match CSV"
)
assert missed_radar > 0 and missed_dvl > 0, "sensor failure schedules not exercised"

report = json.loads((root / "dynamic_mission_report.json").read_text(
    encoding="utf-8"))
assert report["bridge_mode"] == "sensor-eskf", (
    "direct truth fallback is not an acceptable v0.7 result"
)
assert report["pass"] and report["armed_heartbeat_observed"] and (
    report["auto_heartbeat_observed"]
)
assert set((1, 2, 3)).issubset(report["mission_reached_events"])
assert all(float(d) < 2.5
           for d in report["closest_true_distance_to_route_m"][1:])
assert report["ekf_physics_p95_horizontal_error_m"] < 5.0

summary = {
    "sensor_eskf_p95_error_m": round(p95, 3),
    "sensor_eskf_max_error_m": round(max(errors), 3),
    "mavlink_packets": packets,
    "outbound_blocked": blocked,
    "shore_fixes": radar_count,
    "bottom_track_fixes": dvl_count,
    "missed_shore_scheduled": missed_radar,
    "missed_dvl_scheduled": missed_dvl,
    "mission_reached": report["mission_reached_events"],
    "ardupilot_ekf_truth_p95_m":
        report["ekf_physics_p95_horizontal_error_m"],
    "gnss_used": False,
    "model": "rover physics and SITL-only synthetic sensor modalities",
}
(root / "sensor_eskf_acceptance.json").write_text(
    json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print("PASS_SENSOR_ESKF_INTEGRITY " + json.dumps(summary), flush=True)
