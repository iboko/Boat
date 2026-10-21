#!/usr/bin/env python3
"""Verify actual Gazebo BlueBoat truth against MAVLink mission/hold outcomes.

This never converts Rover SIM_STATE alone into Gazebo proof.
"""
import argparse
import json
import math
import sys
from pathlib import Path


def load_truth(path):
    observations = []
    for text in Path(path).read_text(encoding="utf-8").splitlines():
        obj = json.loads(text)
        if obj.get("source") != "Gazebo BlueBoat":
            raise ValueError("truth stream is not labelled Gazebo BlueBoat")
        for field in ("x", "y", "z", "monotonic_s"):
            if not isinstance(obj.get(field), (int, float)) or not math.isfinite(obj[field]):
                raise ValueError(f"invalid Gazebo truth field {field}")
        if abs(obj["z"])>3.0:
            raise ValueError("Gazebo hull exited plausible sea-surface envelope; wave/buoyancy plugin failed")
        if observations and obj["monotonic_s"] <= observations[-1]["monotonic_s"]:
            raise ValueError("non-monotonic Gazebo source clock")
        observations.append(obj)
    return observations


def displacement(a, b):
    return math.hypot(a["x"]-b["x"],a["y"]-b["y"])


def assess(truth, mission, mode, drift_seconds):
    if len(truth) < 30:
        raise ValueError("not enough independent Gazebo BlueBoat poses")
    # Reject physics substitutes that fall through the water or teleport.
    for current, following in zip(truth,truth[1:]):
        dt=following["monotonic_s"]-current["monotonic_s"]
        if dt>0.0 and displacement(current,following)>max(2.0,30.0*dt):
            raise ValueError("implausible BlueBoat horizontal jump in Gazebo physics")
    start = truth[0]
    max_motion = max(displacement(start, p) for p in truth)
    if max_motion < 5.0:
        raise ValueError(f"BlueBoat Gazebo physical movement too small: {max_motion:.2f}m")
    required_bridge = "sensor-eskf" if mode == "route" else "sensor-blackout"
    if mission.get("bridge_mode") != required_bridge:
        raise ValueError(f"expected C++ {required_bridge}, got {mission.get('bridge_mode')}")
    if not mission.get("armed_heartbeat_observed") or not mission.get("auto_heartbeat_observed"):
        raise ValueError("no evidence of ArduPilot armed AUTO")
    result = {"truth_source":"Gazebo BlueBoat actual model odometry",
              "physics_samples":len(truth),
              "model_max_displacement_m":round(max_motion,3),
              "mission_check":mode,
              "not_rover_simstate_only":True}
    if mode == "route":
        if not set((1,2,3)).issubset(mission.get("mission_reached_events",[])):
            raise ValueError("ArduPilot did not confirm BlueBoat waypoints 1,2,3")
        if not mission.get("pass"):
            raise ValueError("ArduPilot navigation mission was not successful")
        result["route_confirmed"]=True
    else:
        if not mission.get("hold_confirmed"):
            raise ValueError("HOLD not confirmed in ArduPilot heartbeat")
        if 3 in mission.get("mission_reached_events",[]):
            raise ValueError("mission reached final waypoint before safety interruption")
        hold_time=mission.get("hold_confirmed_monotonic_s")
        if not isinstance(hold_time,(float,int)):
            raise ValueError("no monotonic timestamp of HOLD heartbeat")
        after=[p for p in truth if p["monotonic_s"]>=hold_time]
        if len(after)<8:
            raise ValueError("Gazebo truth unavailable after HOLD")
        elapsed=after[-1]["monotonic_s"]-after[0]["monotonic_s"]
        if elapsed < drift_seconds-0.9:
            raise ValueError("insufficient real Gazebo post-HOLD drift observation")
        result["hold_drift_window_s"]=round(elapsed,2)
        result["hold_drift_m"]=round(displacement(after[0],after[-1]),3)
        result["hold_confirmed"]=True
        result["station_keeping_confirmed"]=False
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--truth",required=True)
    parser.add_argument("--mission",required=True)
    parser.add_argument("--mode",choices=("route","hold"),required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--drift-seconds",type=float,default=20.0)
    args=parser.parse_args()
    try:
        report=assess(load_truth(args.truth),
            json.loads(Path(args.mission).read_text(encoding="utf-8")),
            args.mode,args.drift_seconds)
        Path(args.output).write_text(json.dumps(report,indent=2)+"\n",
                                    encoding="utf-8")
        print("BLUEBOAT_GAZEBO_VERIFIED "+json.dumps(report),flush=True)
    except (OSError,ValueError,KeyError,TypeError) as e:
        sys.exit("FAIL: "+str(e))
