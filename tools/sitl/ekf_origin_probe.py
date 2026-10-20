#!/usr/bin/env python3
"""ArduPilot Rover SITL-only EKF origin and ExternalNav telemetry audit.

Runs on independent MAVLink SERIAL0. Does NOT command arming, motors, modes,
or missions. No claim of EKF fusion without observable EKF/position evidence.
"""
import json
import math
import socket
import sys
import time
from pathlib import Path

from pymavlink import mavutil

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5760
RUN_DIR = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("sitl-smoke-output")
RUN_DIR.mkdir(parents=True, exist_ok=True)
ORIGIN = (-35.363261, 149.165230, 584.0)
ORIGIN_INT = (
    round(ORIGIN[0] * 1e7),
    round(ORIGIN[1] * 1e7),
    round(ORIGIN[2] * 1000),
)
EXPECTED = {
    "FRAME_CLASS": 2,
    "GPS1_TYPE": 0,
    "GPS2_TYPE": 0,
    "VISO_TYPE": 3,
    "EK3_ENABLE": 1,
    "EK3_SRC1_POSXY": 6,
    "EK3_SRC1_VELXY": 6,
    "EK3_SRC1_YAW": 1,
}
def dump(name, payload):
    (RUN_DIR / name).write_text(json.dumps(payload, indent=2) + "\n",
                                encoding="utf-8")
def request_interval(link, message_id, hz):
    link.mav.command_long_send(link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1000000 / hz, 0, 0, 0, 0, 0)
def request_message(link, message_id):
    link.mav.command_long_send(link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        message_id, 0, 0, 0, 0, 0, 0)

link = None
deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    try:
        link = mavutil.mavlink_connection(
            f"tcp:127.0.0.1:{PORT}", source_system=252,
            source_component=190, autoreconnect=False)
        break
    except (ConnectionError, ConnectionRefusedError, OSError, socket.timeout):
        time.sleep(0.25)
if link is None:
    sys.exit("FAIL: SITL SERIAL0 unavailable")
heartbeat = link.wait_heartbeat(timeout=20)
if heartbeat is None or heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    sys.exit("FAIL: verified ArduPilot heartbeat unavailable")

parameters = {}
for name, expected in EXPECTED.items():
    found = False
    for _ in range(4):
        link.mav.param_request_read_send(
            link.target_system, link.target_component, name.encode("ascii"), -1)
        cutoff = time.monotonic() + 2
        while time.monotonic() < cutoff:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            key = msg.param_id
            if isinstance(key, bytes):
                key = key.decode("ascii", "ignore")
            if str(key).rstrip("\x00") != name:
                continue
            observed = float(msg.param_value)
            parameters[name] = observed
            if abs(observed - expected) > 0.01:
                sys.exit(f"FAIL: {name} actual={observed}, expected={expected}")
            found = True
            break
        if found:
            break
    if not found:
        sys.exit(f"FAIL: parameter not visible in telemetry: {name}")

dump("parameters.json", {"vehicle": int(heartbeat.get_srcSystem()),
                         "parameters": parameters,
                         "note": "SITL-only parameter audit"})
print("GNSS_DISABLED_CONFIRMED EK3_SRC1_POSXY=6 EK3_SRC1_VELXY=6",
      flush=True)

# With no satellite fix, setting geographic EKF origin is mandatory.
# A sent message alone is NOT proof; await GLOBAL_ORIGIN telemetry echo.
origin_echo = None
for attempt in range(5):
    link.mav.set_gps_global_origin_send(link.target_system, *ORIGIN_INT)
    request_message(link, mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
    until = time.monotonic() + 2.0
    while time.monotonic() < until:
        msg = link.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=0.25)
        if msg is None:
            continue
        observed = (int(msg.latitude), int(msg.longitude), int(msg.altitude))
        if abs(observed[0] - ORIGIN_INT[0]) <= 2 and (
            abs(observed[1] - ORIGIN_INT[1]) <= 2 and
            abs(observed[2] - ORIGIN_INT[2]) <= 2000):
            origin_echo = observed
            break
    if origin_echo is not None:
        break
if origin_echo is None:
    dump("origin_result.json", {"origin_set_confirmed": False,
                                "requested_origin": ORIGIN_INT})
    sys.exit("FAIL: no GPS_GLOBAL_ORIGIN confirmation from SITL")
dump("origin_result.json", {"origin_set_confirmed": True,
                            "requested_origin": ORIGIN_INT,
                            "confirmed_origin": origin_echo})
print("EKF_ORIGIN_CONFIRMED_FROM_TELEMETRY", flush=True)

for msg, rate in ((mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5),
                  (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),
                  (mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 1)):
    request_interval(link, msg, rate)

# This probe watches externally reported EKF status and position during the
# companion's 8s fixture transmission. It does not infer fusion from params.
observations = {"ekf_samples": 0, "local_position_samples": 0,
                "absolute_position_flag_samples": 0,
                "relative_position_flag_samples": 0,
                "external_nav_use_reports": 0,
                "latest_ekf_flags": None,
                "last_local_position": None,
                "status_text": [],
                "origin_set_confirmed": True}
stop = time.monotonic() + 24.0
while time.monotonic() < stop:
    try:
        msg = link.recv_match(blocking=True, timeout=0.5)
    except (ConnectionError, OSError):
        break
    if msg is None:
        continue
    mtype = msg.get_type()
    if mtype == "EKF_STATUS_REPORT":
        observations["ekf_samples"] += 1
        flags = int(msg.flags)
        observations["latest_ekf_flags"] = flags
        observations["absolute_position_flag_samples"] += int(bool(flags & 16))
        observations["relative_position_flag_samples"] += int(bool(flags & 8))
    elif mtype == "LOCAL_POSITION_NED":
        observations["local_position_samples"] += 1
        observations["last_local_position"] = {
            "x": float(msg.x), "y": float(msg.y), "z": float(msg.z),
            "vx": float(msg.vx), "vy": float(msg.vy)}
    elif mtype == "STATUSTEXT":
        raw = msg.text
        msg_text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
        if any(token in msg_text.lower() for token in
               ("ekf", "origin", "vision", "external", "gps", "yaw")):
            if "is using external nav data" in msg_text.lower():
                observations["external_nav_use_reports"] += 1
            observations["status_text"].append(msg_text[:160])
            observations["status_text"] = observations["status_text"][-35:]

# Discriminate observation from proof:
# these flags mean horizontal position is estimated by the EKF,
# not that a specific ODOMETRY packet was fused.
observations["ekf_horizontal_estimate_observed"] = bool(
    observations["absolute_position_flag_samples"] or
    observations["relative_position_flag_samples"])
observations["ekf_state_telemetry_observed"] = bool(
    observations["ekf_samples"] and observations["local_position_samples"])
observations["external_nav_selected_by_ekf_reported"] = bool(
    observations["external_nav_use_reports"])
observations["external_nav_fusion_directly_proven"] = False
dump("ekf_observations.json", observations)
print("SITL_EKF_OBSERVATIONS " + json.dumps({
    k: observations[k] for k in (
        "ekf_samples", "local_position_samples",
        "absolute_position_flag_samples", "relative_position_flag_samples",
        "ekf_horizontal_estimate_observed", "ekf_state_telemetry_observed",
        "external_nav_use_reports", "external_nav_selected_by_ekf_reported")}),
    flush=True)
