#!/usr/bin/env python3
"""Deterministic C++ 3DOF onboard-navigation sensitivity study (NOT Gazebo).

No connection to ArduPilot, hardware or network. The executable itself
requires --sitl-only-marine-surrogate, and only synthetic sensor channels
are used for ESKF corrections.
"""
import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path


PROFILES=("calm","nominal","crosswind")
MODES=("onboard-route","onboard-blackout")
COUNTERS=("waypoints","drift_after_neutral_20s_m","eskf_p95_m",
          "max_eskf_error_m","stopped_for_integrity","current_n",
          "current_e","wind_n","wind_e")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def parse_result(log, profile, mode):
    matches=re.findall(r"^PASS_MARINE_(ROUTE|BLACKOUT)\s+(.+)$",log,re.MULTILINE)
    require(len(matches)==1,"expected one complete marine trial result")
    tag, tail=matches[0]
    metrics=dict(re.findall(r"([A-Za-z_0-9]+)=([^\s]+)",tail))
    require(tag==("BLACKOUT" if mode.endswith("blackout") else "ROUTE"),
            "incorrect marine experiment mode")
    require(metrics.get("navigation")=="onboard" and
            metrics.get("environment")==profile and
            metrics.get("NOT_GAZEBO")=="true",
            "result lacks onboard/no-Gazebo source validation")
    require(all(key in metrics for key in COUNTERS),"missing numeric result fields")
    data={key:float(metrics[key]) for key in COUNTERS}
    require(all(math.isfinite(value) for value in data.values()),
            "non-finite marine experiment metric")
    require(data["eskf_p95_m"]<3.0,
            "position uncertainty unexpectedly exceeded research bound")
    if mode.endswith("blackout"):
        require(data["stopped_for_integrity"]==1 and
                data["waypoints"]<3,
                "GNSS-denied navigation was not stopped before route completion")
    else:
        require(data["stopped_for_integrity"]==0 and
                data["waypoints"]==3,
                "the three physical route points were not reached")
    data["waypoints"]=int(data["waypoints"])
    data["stopped_for_integrity"]=int(data["stopped_for_integrity"])
    return data


def run(binary, output_dir):
    binary=Path(binary).resolve()
    require(binary.is_file(),"missing C++ marine validation executable")
    out=Path(output_dir).resolve()
    out.mkdir(parents=True,exist_ok=True)
    results=[]
    for profile in PROFILES:
        for mode in MODES:
            prefix=f"{profile}_{mode.replace('-', '_')}"
            csv_path=out/f"{prefix}.csv"
            log_path=out/f"{prefix}.log"
            command=[str(binary),"--sitl-only-marine-surrogate",mode,
                     str(csv_path),"--environment",profile]
            result=subprocess.run(command,capture_output=True,text=True,
                                  timeout=90,check=False)
            log=result.stdout+result.stderr
            log_path.write_text(log,encoding="utf-8")
            require(result.returncode==0,
                    f"marine {profile}/{mode} failed: {log[-500:]}")
            metrics=parse_result(log,profile,mode)
            require(csv_path.is_file() and
                    sum(1 for _ in csv_path.open(encoding="utf-8"))>=90,
                    f"missing marine time series {prefix}")
            results.append({"environment":profile,"scenario":mode,
                            "csv":csv_path.name,"log":log_path.name,**metrics})
    calm=next(x for x in results if x["environment"]=="calm" and
              x["scenario"]=="onboard-blackout")
    severe=next(x for x in results if x["environment"]=="crosswind" and
                x["scenario"]=="onboard-blackout")
    drift_difference=severe["drift_after_neutral_20s_m"]-calm["drift_after_neutral_20s_m"]
    require(drift_difference>1.0,
            "crosswind/current case did not increase simulated post-stop drift")
    report={"kind":"C++ planar marine 3DOF synthetic stress study",
            "not_gazebo":True,
            "not_real_vessel":True,
            "navigation":"synthetic GNSS/INS/compass/bottom DVL -> C++ ESKF",
            "shore_infrastructure_used":False,
            "trials":results,
            "crosswind_minus_calm_blackout_drift_m":round(drift_difference,3)}
    (out/"onboard_environment_matrix.json").write_text(
        json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("PASS_ONBOARD_ENVIRONMENT_MATRIX "+json.dumps({
        "trials":len(results),"crosswind_minus_calm_drift_m":round(drift_difference,3),
        "all_routes_reached":True,"all_blackouts_stopped":True}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--sitl-only",action="store_true")
    parser.add_argument("--executable",required=True)
    parser.add_argument("--output-dir",required=True)
    args=parser.parse_args()
    try:
        require(args.sitl_only,"--sitl-only must be explicitly supplied")
        run(args.executable,args.output_dir)
    except (OSError,ValueError,subprocess.TimeoutExpired) as exc:
        sys.exit("FAIL_ONBOARD_ENVIRONMENT_MATRIX: "+str(exc))
