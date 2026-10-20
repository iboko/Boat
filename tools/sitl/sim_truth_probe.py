#!/usr/bin/env python3
"""Probe true SITL physics pose, NOT autopilot EKF estimates. No control actions."""
import json
import math
import sys
import time
from pathlib import Path
from pymavlink import mavutil

output = Path(sys.argv[1])
link = mavutil.mavlink_connection("tcp:127.0.0.1:5763",
                                 source_system=250, source_component=187)
hb = link.wait_heartbeat(timeout=10)
if not hb or hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
    sys.exit("FAIL: no SITL autopilot heartbeat")
msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE
link.mav.command_long_send(link.target_system,link.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,msg_id,100000,0,0,0,0,0)
samples = []
deadline = time.monotonic()+7.0
while time.monotonic() < deadline:
    msg=link.recv_match(type=["SIM_STATE","COMMAND_ACK"],blocking=True,timeout=0.5)
    if msg is None:
        continue
    if msg.get_type()=="SIM_STATE":
        li=int(getattr(msg,"lat_int",0))
        loi=int(getattr(msg,"lon_int",0))
        lat=li*1e-7 if li else float(msg.lat)
        lon=loi*1e-7 if loi else float(msg.lon)
        if math.isfinite(lat) and math.isfinite(lon):
            samples.append({"lat":lat,"lon":lon,"lat_int":li,"lon_int":loi,
              "vn":float(msg.vn),"ve":float(msg.ve),"yaw":float(msg.yaw)})
        if len(samples)>=15:
            break
output.parent.mkdir(parents=True,exist_ok=True)
report={"source":"SIM_STATE physics truth, not EKF position",
        "sample_count":len(samples),"samples":samples[:4],
        "high_precision_coordinates":bool(samples and samples[0]["lat_int"])}
output.write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print("SITL_SIM_STATE_TRUTH", json.dumps(report),flush=True)
if len(samples)<8:
    sys.exit("FAIL: SIM_STATE truth stream unavailable")
