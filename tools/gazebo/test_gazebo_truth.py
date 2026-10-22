#!/usr/bin/env python3
import json
import math
import tempfile
from pathlib import Path
from gazebo_truth_monitor import parse_pose
from verify_blueboat_trial import load_truth, assess

MSG = """pose {
  position {
    x: 11.2
    y: -4.8
    z: 0.19
  }
  orientation {
    w: 1
  }
}
"""
assert parse_pose(MSG)=={"x":11.2,"y":-4.8,"z":0.19}
assert parse_pose("pose { orientation { w:1 } }") is None
assert parse_pose("position { x: nan y: 0 z: 0 }") is None
with tempfile.TemporaryDirectory() as directory:
    root=Path(directory)
    path=root/"truth.jsonl"
    base=10000.0
    # Not a live execution; deterministic fixture for verifier only.
    rows=[{"monotonic_s":base+i*0.25, "x":float(i)*0.15,
           "y":float(i)*0.04,"z":0.0, "source":"Gazebo BlueBoat"}
          for i in range(150)]
    path.write_text("\n".join(json.dumps(r) for r in rows)+"\n")
    truth=load_truth(path)
    complete={"bridge_mode":"sensor-eskf",
              "armed_heartbeat_observed":True,
              "auto_heartbeat_observed":True,
              "mission_reached_events":[1,2,3],"pass":True}
    report=assess(truth,complete,"route",20.0)
    assert report["route_confirmed"] and report["model_max_displacement_m"]>10
    outage={"bridge_mode":"sensor-blackout","armed_heartbeat_observed":True,
            "auto_heartbeat_observed":True,
            "hold_confirmed":True,
            "hold_confirmed_monotonic_s":base+10.0,
            "mission_reached_events":[1]}
    report=assess(truth,outage,"hold",20)
    assert report["hold_drift_m"]>5
    # Regression: Gazebo previously let the boat free-fall without hydrodynamics.
    sinking = [dict(row) for row in rows]
    sinking[10]["z"]=-540.0
    path.write_text("\n".join(json.dumps(r) for r in sinking)+"\n")
    try:
        load_truth(path)
        raise AssertionError("sinking BlueBoat accepted as valid marine physics")
    except ValueError:
        pass
    for bad in ([],[{"monotonic_s":1,"x":0,"y":0,"z":0,"source":"Rover SIM_STATE"}]):
        path.write_text("\n".join(json.dumps(r) for r in bad))
        try:
            load_truth(path)
            if bad:
                raise AssertionError("fake Gazebo truth accepted")
        except ValueError:
            if not bad:
                pass
print("PASS: real Gazebo odometry parser and strict independent route/HOLD verifier fixture")
