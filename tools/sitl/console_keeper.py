#!/usr/bin/env python3
"""SITL-only telemetry audit. Python is a test dependency, not the Boat runtime.

Connects to the independent Rover console MAVLink channel, checks that the
autopilot actually loaded the GNSS-denied ExternalNav profile, then holds
the console session open. Never arms the vehicle or commands the motors.
"""
import json
import socket
import sys
import time
from pathlib import Path

from pymavlink import mavutil

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5760
OUTPUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("sitl_parameters.json")
EXPECTED = {
    "FRAME_CLASS": 2,
    "GPS1_TYPE": 0,
    "GPS2_TYPE": 0,
    "VISO_TYPE": 3,
    "EK3_SRC1_POSXY": 6,
    "EK3_SRC1_VELXY": 6,
    "EK3_SRC1_YAW": 1,
}
deadline = time.monotonic() + 30
master = None
while time.monotonic() < deadline:
    try:
        master = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{PORT}",
            source_system=252,
            source_component=190,
            autoreconnect=False,
        )
        break
    except (ConnectionError, ConnectionRefusedError, OSError, socket.timeout):
        time.sleep(0.3)
if master is None:
    sys.exit("FAIL: SERIAL0 TCP port unavailable")

heartbeat = master.wait_heartbeat(timeout=20)
if heartbeat is None or heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    sys.exit("FAIL: heartbeat not received from ArduPilot")

received = {}
for name, expected in EXPECTED.items():
    matched = False
    for _ in range(4):
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            -1,
        )
        until = time.monotonic() + 2
        while time.monotonic() < until:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", "ignore")
            param_id = str(param_id).rstrip("\x00")
            if param_id != name:
                continue
            received[name] = float(msg.param_value)
            if abs(received[name] - expected) > 0.01:
                sys.exit(f"FAIL: {name} expected {expected}, got {received[name]}")
            matched = True
            break
        if matched:
            break
    if not matched:
        sys.exit(f"FAIL: no telemetry verification for parameter {name}")

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps({"vehicle_sysid": int(heartbeat.get_srcSystem()),
                              "parameters": received,
                              "note": "SITL only; EKF measurement fusion not verified"},
                             indent=2), encoding="utf-8")
print("GNSS_DISABLED_CONFIRMED Rover FRAME_CLASS=2 EK3_SRC1_POSXY=6 "
      "EK3_SRC1_VELXY=6", flush=True)

# Retain the MAVLink console so the SITL simulator keeps running.
ending = time.monotonic() + 45
while time.monotonic() < ending:
    try:
        master.recv_match(blocking=True, timeout=1)
    except (ConnectionError, OSError):
        break
