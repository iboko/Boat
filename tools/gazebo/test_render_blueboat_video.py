#!/usr/bin/env python3
"""Small deterministic video encoder smoke test, NOT a Gazebo trial."""
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

from render_blueboat_video import render
from verify_blueboat_trial import assess, load_truth


assert shutil.which("ffmpeg"), "ffmpeg is required for the MP4 evidence artifact"
assert shutil.which("ffprobe"), "ffprobe is required to validate encoded MP4"
with tempfile.TemporaryDirectory(prefix="boat-renderer-unit-") as directory:
    root=Path(directory)
    path=root/"test_only_synthetic_truth.jsonl"
    start=30000.0
    rows=[{"monotonic_s":start+i*0.25,"x":i*0.11,
           "y":math.sin(i*0.1)*0.4,"z":0.0,"source":"Gazebo BlueBoat"}
          for i in range(160)]
    path.write_text("\n".join(json.dumps(row) for row in rows)+"\n",
                    encoding="utf-8")
    truth=load_truth(path)

    for mode in ("onboard-route","onboard-blackout"):
        is_hold=mode.endswith("blackout")
        mission={"bridge_mode":mode,"onboard_without_shore":True,
                 "pass":True,"armed_heartbeat_observed":True,
                 "auto_heartbeat_observed":True,
                 "gnss_brief_outage_after_armed_auto":True,
                 "gnss_blackout_after_armed_auto":True,
                 "mission_reached_events":[1] if is_hold else [1,2,3],
                 "hold_confirmed":is_hold,
                 "hold_confirmed_monotonic_s":start+10.0 if is_hold else None}
        verified=assess(truth,mission,"hold" if is_hold else "route",20.0)
        report=root/"mission.json"
        evidence=root/"verification.json"
        report.write_text(json.dumps(mission),encoding="utf-8")
        evidence.write_text(json.dumps(verified),encoding="utf-8")
        target=root/(mode+".mp4")
        summary=render(path,report,evidence,target,frames=24,fps=12)
        assert summary["not_a_camera_recording"] and target.stat().st_size>3000
        probe=subprocess.run(["ffprobe","-v","error","-select_streams","v:0",
                              "-show_entries","stream=codec_name,nb_frames",
                              "-of","json",str(target)],check=True,
                             capture_output=True,text=True)
        streams=json.loads(probe.stdout)["streams"]
        assert len(streams)==1 and streams[0]["codec_name"]=="h264"

    # Rendering requires already verified physical reports, never naked Rover truth.
    false_evidence={"truth_source":"Rover SIM_STATE",
                    "not_rover_simstate_only":False}
    evidence.write_text(json.dumps(false_evidence),encoding="utf-8")
    try:
        render(path,report,evidence,root/"invalid.mp4",frames=24,fps=12)
        raise AssertionError("unverified physics accepted by video renderer")
    except ValueError:
        pass

print("PASS: verified BlueBoat trajectory -> labelled decodable MP4 route/HOLD; "
      "unverified source rejected")
