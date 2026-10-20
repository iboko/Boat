#!/usr/bin/env python3
"""Validate generated C++ marine surrogate route and navigation-loss trials.

No Gazebo and no ArduPilot commands are sent by this report verifier.
"""
import csv
import json
import math
import sys
from pathlib import Path

out=Path(sys.argv[1])
def trial(name,expect_completion):
    path=out/f"marine_{name}.csv"
    rows=list(csv.DictReader(path.open(newline="",encoding="utf-8")))
    assert len(rows)>=200, f"too few physical measurement samples in {name}"
    for row in rows:
        assert all(math.isfinite(float(v)) for v in row.values()), "non-finite marine measurement"
    completed=sum(int(row["target_index"])>=3 for row in rows)>0
    integrity_lost=any(int(row["nav_integrity"])==0 for row in rows)
    first_neutral=next(i for i,r in enumerate(rows) if int(r["neutral_propulsion"])==1)
    segment=rows[first_neutral:]
    assert len(segment)>100, "20 s of post-neutral drift not recorded"
    assert all(float(r["thrust_n"])==0 and float(r["yaw_moment_nm"])==0 for r in segment),(
        "neutral motor state violated"
    )
    last=rows[-1]
    first=rows[first_neutral]
    drift=math.hypot(float(last["physics_n"])-float(first["physics_n"]),
                     float(last["physics_e"])-float(first["physics_e"]))
    assert drift>2.0,"zero thrust incorrectly treated as stationary"
    if expect_completion:
        assert completed and not integrity_lost, "normal route failed"
        reached=[i for i,r in enumerate(rows) if int(r["target_index"])>0]
        assert reached, "no waypoint progression"
    else:
        assert integrity_lost and not completed, "blackout did not interrupt route"
        assert float(first["time_s"])>7.5, "stale fix checked before actual outage"
    errors=sorted(float(row["estimate_error_m"]) for row in rows)
    p95=errors[int(0.95*(len(errors)-1))]
    assert p95<3.0, "ESKF error limit exceeded"
    return {"samples":len(rows),"route_complete":completed,
            "navigation_lost":integrity_lost,
            "neutral_duration_s":round(float(last["time_s"])-float(first["time_s"]),2),
            "neutral_drift_m":round(drift,3),
            "eskf_p95_m":round(p95,3)}
res={"model":"standalone planar 3DOF C++ force model, NOT Gazebo/BlueBoat",
     "wind_ned_mps":[2.0,3.0],
     "water_current_ned_mps":[0.35,-0.20],
     "route":trial("route",True),"blackout":trial("blackout",False),
     "gazebo_executed":False,
     "arduPilot_marine_model_executed":False,
     "actual_hydrodynamic_coefficients_calibrated":False}
(out/"marine_validation.json").write_text(json.dumps(res,indent=2)+"\n",
                                          encoding="utf-8")
print("MARINE_SURROGATE_VALIDATED "+json.dumps(res))
