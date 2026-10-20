#!/usr/bin/env python3
"""SITL-ONLY waypoint mission upload/readback, without arming or motion.

A protocol test is NOT an autonomous voyage. The route is never started.
The target host is intentionally fixed at localhost.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

from pymavlink import mavutil

HOME_LAT = -35.363261
HOME_LON = 149.165230
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(HOME_LAT))


def mission_waypoints():
    """Home + three local waypoint offsets, encoded in GLOBAL_RELATIVE_ALT_INT."""
    offsets = [(0.0, 0.0), (12.0, 0.0),
               (12.0, 12.0), (0.0, 12.0)]
    return [(round((HOME_LAT + dn/M_PER_DEG_LAT) * 1e7),
             round((HOME_LON + de/M_PER_DEG_LON) * 1e7))
            for dn, de in offsets]


def receive(link, allowed, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = link.recv_match(blocking=True, timeout=0.4)
        if msg and msg.get_type() in allowed:
            return msg
    raise TimeoutError("MAVLink timeout waiting for " + ", ".join(sorted(allowed)))


def upload_and_verify(link):
    target_sys, target_comp = link.target_system, link.target_component
    kind = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    route = mission_waypoints()
    link.mav.mission_clear_all_send(target_sys, target_comp, kind)
    cleared = receive(link, {"MISSION_ACK"}, 8)
    if int(cleared.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission clear rejected: {cleared.type}")

    link.mav.mission_count_send(target_sys, target_comp, len(route), kind)
    pending = set(range(len(route)))
    messages = 0
    deadline = time.monotonic() + 35
    while pending and time.monotonic() < deadline:
        msg = link.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            raise RuntimeError(f"Unexpected MISSION_ACK before all items: {msg.type}")
        if msg.get_type() not in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            continue
        idx = int(msg.seq)
        if idx < 0 or idx >= len(route):
            raise RuntimeError("Invalid requested waypoint index")
        lat, lon = route[idx]
        link.mav.mission_item_int_send(
            target_sys, target_comp, idx,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            int(idx == 0), 1,
            0.0, 2.0, 0.0, float("nan"),
            lat, lon, 0.0, kind)
        messages += 1
        pending.discard(idx)
    if pending:
        raise TimeoutError(f"Autopilot did not request items: {sorted(pending)}")
    ack = receive(link, {"MISSION_ACK"}, 10)
    if int(ack.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission upload rejected: MAV_MISSION_RESULT {ack.type}")

    # The number of stored mission items can differ by one when a vehicle
    # treats index zero as home; verify the requested route items by position.
    link.mav.mission_request_list_send(target_sys, target_comp, kind)
    count = receive(link, {"MISSION_COUNT"}, 8)
    stored_count = int(count.count)
    if stored_count != len(route):
        raise RuntimeError(f"Readback count {stored_count}, expected {len(route)}")
    readback = []
    for idx in range(stored_count):
        link.mav.mission_request_int_send(target_sys, target_comp, idx, kind)
        item = receive(link, {"MISSION_ITEM_INT"}, 8)
        if int(item.seq) != idx:
            raise RuntimeError(f"Unexpected readback sequence {item.seq} != {idx}")
        if abs(int(item.x)-route[idx][0]) > 2 or abs(int(item.y)-route[idx][1]) > 2:
            raise RuntimeError(f"Readback coordinate mismatch at {idx}: got=({item.x},{item.y}) expected=({route[idx][0]},{route[idx][1]}) frame={item.frame} command={item.command}")
        readback.append({"sequence": idx, "latitude_1e7": int(item.x),
                         "longitude_1e7": int(item.y)})
    return {"mission_uploaded": True, "mission_readback_verified": True,
            "points": len(route), "item_requests_serviced": messages,
            "readback": readback, "armed": False, "mission_executed": False}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sitl-only", action="store_true")
    parser.add_argument("--port", type=int, default=5763)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.sitl_only:
        sys.exit("Refusing mission upload without explicit --sitl-only")
    if args.port < 1 or args.port > 65535:
        sys.exit("Invalid TCP port")
    link = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{args.port}",
        source_system=251, source_component=189,
        autoreconnect=False)
    hb = link.wait_heartbeat(timeout=12)
    if hb is None or hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        sys.exit("No ArduPilot heartbeat on SITL")
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        sys.exit("Refusing to upload test mission: SITL vehicle already armed")
    report = upload_and_verify(link)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("SITL_WAYPOINT_PROTOCOL_PASS points={} readback=verified "
          "armed=no mission_executed=no".format(report["points"]))


if __name__ == "__main__":
    try:
        main()
    except (TimeoutError, ValueError, RuntimeError, ConnectionError, OSError) as exc:
        sys.exit(f"SITL_WAYPOINT_PROTOCOL_FAIL: {exc}")
