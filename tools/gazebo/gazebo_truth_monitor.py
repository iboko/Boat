#!/usr/bin/env python3
"""Capture raw BlueBoat world odometry emitted by Gazebo (not ArduPilot EKF).

Requires gz CLI and the actual /model/blueboat/odometry publisher.
Monitor never sends actuation or MAVLink messages.
"""
import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

NUMBER = r"[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[Ee][-+]?[0-9]+)?"
COORD = re.compile(r"\b([xyz]):\s*(" + NUMBER + r")")
POSITION = re.compile(r"\bposition\s*\{([^{}]*)\}", re.DOTALL)


def parse_pose(message):
    match = POSITION.search(message)
    if match is None:
        return None
    vals = {name: float(value) for name, value in COORD.findall(match.group(1))}
    if not set("xyz").issubset(vals):
        return None
    if not all(math.isfinite(vals[x]) for x in "xyz"):
        return None
    if max(abs(vals[x]) for x in "xyz") > 20000:
        return None
    return vals


def capture(args):
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Topic is explicitly fixed to expected BlueBoat odometry; Gazebo
    # pose truth must not be replaced by EKF or NAV_CONTROLLER_OUTPUT.
    command = ["gz", "topic", "-e", "-t", "/model/blueboat/odometry"]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, bufsize=1)
    count = 0
    start = time.monotonic()
    chunks = []
    braces = 0
    active = False
    try:
        with path.open("w", encoding="utf-8") as out:
            for line in proc.stdout:
                if time.monotonic() - start > args.seconds:
                    break
                # Each gz.msgs.Odometry printed protobuf starts at 'header {'.
                if (line.startswith("header {") or line.startswith("pose {")) and braces == 0:
                    chunks = []
                    active = True
                if not active:
                    continue
                chunks.append(line)
                braces += line.count("{") - line.count("}")
                if braces < 0:
                    raise RuntimeError("unexpected Gazebo protobuf brace structure")
                if braces == 0 and chunks:
                    point = parse_pose("".join(chunks))
                    if point is not None:
                        record = {"monotonic_s": round(time.monotonic(), 6),
                                  "x": point["x"], "y": point["y"],
                                  "z": point["z"], "source": "Gazebo BlueBoat"}
                        out.write(json.dumps(record) + "\n")
                        out.flush()
                        count += 1
                    chunks = []
                    active = False
                    if args.max_samples and count >= args.max_samples:
                        break
        if count == 0:
            raise RuntimeError("NO_GAZEBO_PHYSICS_ODOMETRY: no valid BlueBoat positions")
        print(f"BLUEBOAT_GAZEBO_TRUTH samples={count} file={path}", flush=True)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=200)
    parser.add_argument("--max-samples", type=int, default=0)
    args = parser.parse_args()
    try:
        capture(args)
    except (OSError, RuntimeError) as exc:
        sys.exit("FAIL: " + str(exc))
