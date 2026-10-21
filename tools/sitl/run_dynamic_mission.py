#!/usr/bin/env python3
"""True Rover SITL dynamic waypoint test; all motors are simulated.

Python orchestrates the test and samples the true physics SIM_STATE (not EKF
position). A C++ companion publishes deliberately synthetic external odometry.
This test cannot be used against any physical autopilot or non-loopback host.
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

from mission_protocol_probe import (
    HOME_LAT, HOME_LON, M_PER_DEG_LAT, M_PER_DEG_LON,
    mission_waypoints, upload_and_verify,
)

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sitl-only", action="store_true")
    parser.add_argument("--bridge-mode",choices=("direct-truth","sensor-eskf","sensor-blackout"),default="direct-truth")
    parser.add_argument("--duration", type=int, default=135)
    parser.add_argument("--gazebo-truth-file", type=Path,
        help="require recent live BlueBoat Gazebo pose stream before mission writes/ARM")
    parser.add_argument("--cpp-bridge", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    require(args.sitl_only, "explicit --sitl-only is mandatory")
    require(20 <= args.duration <= 240, "duration must be 20..240 seconds")
    require(args.cpp_bridge.is_file(), "missing C++ dynamic bridge")
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    link = mavutil.mavlink_connection("tcp:127.0.0.1:5763",
        source_system=249, source_component=186, autoreconnect=False)
    hb = link.wait_heartbeat(timeout=12)
    require(hb and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            "no Rover SITL heartbeat")
    require(not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            "vehicle already armed before test")

    # Reject any non-SITL target BEFORE sending MISSION_CLEAR_ALL or ARM.
    # A loopback URL and the user's --sitl-only flag alone are insufficient.
    sim_id = mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE
    link.mav.command_long_send(link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        sim_id, 100000, 0, 0, 0, 0, 0)
    physics_proof = False
    proof_deadline = time.monotonic()+10.0
    while time.monotonic()<proof_deadline:
        proof=link.recv_match(type="SIM_STATE",blocking=True,timeout=0.35)
        if proof is None:
            continue
        lat_int=int(getattr(proof,"lat_int",0))
        lon_int=int(getattr(proof,"lon_int",0))
        if (lat_int and lon_int and
            abs(lat_int*1e-7-HOME_LAT)<0.0005 and
            abs(lon_int*1e-7-HOME_LON)<0.0005):
            physics_proof=True
            break
    require(physics_proof, "refusing any mission mutation: SITL physical truth not verified")
    print("SITL_PHYSICS_PROOF_BEFORE_MISSION_MUTATION",flush=True)

    if args.gazebo_truth_file is not None:
        # A SIM_STATE message by itself is NOT proof of Gazebo. The
        # independent gz topic monitor must be alive and reporting physics.
        gaze_verified = False
        gaze_deadline = time.monotonic() + 18.0
        while time.monotonic() < gaze_deadline:
            try:
                recent = [json.loads(item) for item in
                          args.gazebo_truth_file.read_text(encoding="utf-8").splitlines()[-10:]]
                gaze_verified = len(recent) >= 6 and all(
                    entry.get("source") == "Gazebo BlueBoat" and
                    all(math.isfinite(float(entry[field])) for field in
                        ("x", "y", "z", "monotonic_s"))
                    for entry in recent
                ) and (time.monotonic()-recent[-1]["monotonic_s"] < 4.0)
            except (OSError, KeyError, TypeError, ValueError, OverflowError):
                gaze_verified = False
            if gaze_verified:
                break
            time.sleep(0.3)
        require(gaze_verified,
                "refusing mission mutation: actual live Gazebo BlueBoat world poses absent")
        print("GAZEBO_BLUEBOAT_PHYSICS_PROOF_BEFORE_MISSION_MUTATION",flush=True)

    uploaded = upload_and_verify(link)
    require(uploaded["mission_readback_verified"], "mission readback not confirmed")
    print("SITL_DYNAMIC_MISSION_UPLOADED", flush=True)

    # Unlike LOCAL_POSITION_NED, SIM_STATE contains simulator physical truth.
    msg_sim = mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE
    msg_local = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    for msgid, interval in ((msg_sim, 100000), (msg_local, 200000),
                            (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 500000)):
        link.mav.command_long_send(link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, interval, 0, 0, 0, 0, 0)

    latlon = mission_waypoints()
    targets = [
        ((lat * 1e-7 - HOME_LAT) * M_PER_DEG_LAT,
         (lon * 1e-7 - HOME_LON) * M_PER_DEG_LON)
        for lat, lon in latlon
    ]
    closest = [float("inf")] * len(targets)
    last_ekf = None
    seen_seq = set()
    reached = set()
    messages = []
    states = []
    statuses = []
    max_dist_from_start = 0.0
    armed = False
    mode_auto = False
    ever_armed = False
    ever_auto = False
    command_sent = False
    command_at = None
    truth_ekf_errors = []
    last_truth = None
    last_truth_monotonic = None
    start = time.monotonic()
    begun = None
    last_publish = 0.0
    last_timestamp = 0.0
    previous_truth=None
    bridge = None
    bridge_stderr = None
    marker=out/"navigation_integrity_failed.txt"
    hold_requested_at=None
    hold_confirmed_at=None
    hold_request_sent=False
    report_path = out / "dynamic_mission_report.json"
    csv_file = (out / "dynamic_sitl_truth.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["wall_time_s","physics_n_m","physics_e_m","physics_vn_mps",
                     "physics_ve_mps","ekf_n_m","ekf_e_m","mission_seq",
                     "distance_from_start_m","closest_target_1_m",
                     "closest_target_2_m","closest_target_3_m"])
    bridge_stdout=(out/"dynamic_cpp_odometry.csv").open("w",encoding="utf-8")
    bridge_stderr=(out/"dynamic_bridge.log").open("w",encoding="utf-8")
    result={}
    failure=None
    try:
        bridge_args=[str(args.cpp_bridge.resolve())]
        if args.bridge_mode=="direct-truth":
            bridge_args.append("--sitl-dynamic-fixture")
        else:
            bridge_args.append("--sitl-sensor-fusion-fixture")
            if args.bridge_mode=="sensor-blackout":
                marker.unlink(missing_ok=True)
                bridge_args.extend(["--radar-blackout","--hold-marker",
                                    str(marker.resolve())])
        bridge=subprocess.Popen(
            bridge_args,
            stdin=subprocess.PIPE, stdout=bridge_stdout, stderr=bridge_stderr,
            text=True, bufsize=1)
        require(bridge.stdin is not None, "bridge stdin not available")
        warm_samples=0
        nav_seq=0
        start_deadline=start + args.duration
        while time.monotonic()<start_deadline:
            try:
                msg=link.recv_match(blocking=True,timeout=0.35)
            except (OSError, ConnectionError) as exc:
                raise RuntimeError("MAVLink SITL disconnected") from exc
            now=time.monotonic()
            if args.bridge_mode=="sensor-blackout" and marker.is_file() and not hold_request_sent:
                require(ever_armed and ever_auto,
                        "integrity violation occurred before armed AUTO")
                require(marker.read_text().startswith("INVALID_NAVIGATION"),
                        "invalid C++ navigation integrity report")
                link.set_mode("HOLD")
                hold_request_sent=True
                hold_requested_at=now
                print("SITL_NAVIGATION_FAILSAFE_HOLD_REQUESTED",flush=True)
            if hold_request_sent and hold_confirmed_at is None and now-hold_requested_at>6.0:
                raise RuntimeError("SITL did not acknowledge HOLD after navigation integrity loss")
            if msg is None:
                if begun is not None and now-last_truth_monotonic>2.0:
                    raise RuntimeError("physical truth stream lost >2 seconds")
                continue
            typ=msg.get_type()
            if typ=="LOCAL_POSITION_NED":
                last_ekf=(float(msg.x),float(msg.y))
                if last_truth is not None and last_truth_monotonic is not None and now-last_truth_monotonic<0.3:
                    truth_ekf_errors.append(math.hypot(last_ekf[0]-last_truth[0],last_ekf[1]-last_truth[1]))
            elif typ=="MISSION_CURRENT":
                nav_seq=int(msg.seq)
                seen_seq.add(nav_seq)
            elif typ=="MISSION_ITEM_REACHED":
                reached.add(int(msg.seq))
            elif typ=="STATUSTEXT":
                content=msg.text
                if isinstance(content,bytes):
                    content=content.decode("utf-8","replace")
                statuses.append(str(content)[:160])
                statuses=statuses[-80:]
            elif typ=="HEARTBEAT" and msg.get_srcSystem()==link.target_system:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_auto = bool((msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) and int(msg.custom_mode)==10)
                ever_armed = ever_armed or armed
                ever_auto = ever_auto or mode_auto
                if hold_request_sent and int(msg.custom_mode)==4 and not mode_auto:
                    hold_confirmed_at=now
                    print("SITL_NAVIGATION_FAILSAFE_HOLD_CONFIRMED",flush=True)
                    break
            elif typ=="SIM_STATE":
                lat_int=int(getattr(msg,"lat_int",0))
                lon_int=int(getattr(msg,"lon_int",0))
                require(lat_int != 0 and lon_int != 0,
                    "missing high-precision SIM_STATE coordinates, cannot evaluate waypoints")
                lat=lat_int*1e-7
                lon=lon_int*1e-7
                north=(lat-HOME_LAT)*M_PER_DEG_LAT
                east=(lon-HOME_LON)*M_PER_DEG_LON
                require(abs(north)<2000 and abs(east)<2000,
                        "SIM_STATE truth far outside survey datum")
                last_truth_monotonic=now
                last_truth=(north,east)
                if begun is None:
                    begun=now
                elapsed=now-begun
                if elapsed-last_publish>=0.095:
                    record=(
                        max(elapsed, last_timestamp+0.001),
                        lat,lon,float(msg.alt),
                        float(msg.vn),float(msg.ve),float(msg.vd),
                        float(msg.roll),float(msg.pitch),float(msg.yaw),
                        float(msg.xgyro),float(msg.ygyro),float(msg.zgyro))
                    require(all(math.isfinite(x) for x in record),
                            "non-finite truth message")
                    line=",".join(f"{v:.9f}" for v in record)+"\n"
                    try:
                        bridge.stdin.write(line)
                        bridge.stdin.flush()
                    except BrokenPipeError as exc:
                        raise RuntimeError("C++ truth-to-ODOMETRY bridge terminated") from exc
                    last_publish=elapsed
                    last_timestamp=record[0]
                    warm_samples+=1
                for j,pt in enumerate(targets):
                    closest[j]=min(closest[j],math.hypot(north-pt[0],east-pt[1]))
                max_dist_from_start=max(max_dist_from_start,
                    math.hypot(north-targets[0][0],east-targets[0][1]))
                if now-start>5.0 and int((now-start)*5)!=int((now-start-0.2)*5):
                    writer.writerow([
                        round(now-start,2),round(north,3),round(east,3),
                        round(float(msg.vn),3),round(float(msg.ve),3),
                        round(last_ekf[0],3) if last_ekf else "",
                        round(last_ekf[1],3) if last_ekf else "",
                        nav_seq,round(max_dist_from_start,3),
                        *[round(x,3) for x in closest[1:]],
                    ])
                if not command_sent and warm_samples>=25 and now-start>4.0:
                    link.set_mode("AUTO")
                    # Valid, non-forced arming command. No altered safety checks.
                    link.mav.command_long_send(link.target_system,
                        link.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,1,0,0,0,0,0,0)
                    print("SITL_AUTO_ARM_REQUESTED (ordinary non-forced)",flush=True)
                    command_sent=True
                    command_at=now  # confirmation requires HEARTBEAT, not the request itself
                if command_sent and not ever_armed and now-command_at>25.0:
                    raise RuntimeError("ordinary arming not confirmed; status: "+str(statuses[-10:]))
                if args.bridge_mode!="sensor-blackout" and max_dist_from_start>15.0 and all(x<2.5 for x in closest[1:]) and (len(targets)-1) in reached and ever_armed and ever_auto:
                    print("SITL_DYNAMIC_PHYSICS_WAYPOINTS_VISITED",flush=True)
                    break
            if bridge and bridge.poll() is not None:
                raise RuntimeError(f"C++ bridge exited early with code {bridge.returncode}")

        result={
            "fixture":("SITL physics truth -> simulated noisy IMU/shore radar/bottom DVL -> C++ 15-state ESKF -> MAVLink" if args.bridge_mode=="sensor-eskf" else "SITL physics truth -> C++ synthetic ODOMETRY"),
            "bridge_mode":args.bridge_mode,
            "hold_request_sent":hold_request_sent,
            "hold_confirmed":hold_confirmed_at is not None,
            "hold_confirmed_monotonic_s":hold_confirmed_at,
            "hold_requested_monotonic_s":hold_requested_at,
            "gazebo_verified_before_arm":args.gazebo_truth_file is not None,
            "hold_latency_s":(round(hold_confirmed_at-hold_requested_at,3) if hold_confirmed_at is not None else None),
            "integrity_marker_observed":marker.is_file() if args.bridge_mode=="sensor-blackout" else False,
            "gnss_from_ardupilot_parameters":"disabled in separate origin probe",
            "mission_loaded_and_readback":True,
            "mission_executed": bool(max_dist_from_start>15.0 and ever_armed and ever_auto),
            "armed_heartbeat_observed":ever_armed,
            "auto_heartbeat_observed":ever_auto,
            "ekf_physics_error_samples":len(truth_ekf_errors),
            "ekf_physics_mean_horizontal_error_m":(round(sum(truth_ekf_errors)/len(truth_ekf_errors),3) if truth_ekf_errors else None),
            "ekf_physics_p95_horizontal_error_m":(round(sorted(truth_ekf_errors)[int(0.95*(len(truth_ekf_errors)-1))],3) if truth_ekf_errors else None),
            "physics_distance_max_from_start_m":round(max_dist_from_start,3),
            "closest_true_distance_to_route_m":[round(x,3) for x in closest],
            "mission_sequences_seen":sorted(seen_seq),
            "mission_reached_events":sorted(reached),
            "C++_truth_samples_sent":warm_samples,
            "final_ekf_local_position_m":last_ekf,
            "autopilot_status_text":statuses,
            "recorded_ground_truth_source":"MAVLink SIM_STATE, not EKF",
        }
        require(warm_samples>=30,"too few dynamic truth samples")
        require(ever_armed and ever_auto,"Rover heartbeat did not confirm armed AUTO mode")
        require(len(truth_ekf_errors)>=20,"no paired EKF/physics trajectory measurements")
        if args.bridge_mode=="sensor-blackout":
            require(hold_request_sent and hold_confirmed_at is not None,
                    "loss of position did not move Rover into HOLD")
            require(hold_confirmed_at-hold_requested_at<=6.0,
                    "HOLD response exceeded 6 seconds")
            require((len(targets)-1) not in reached,
                    "mission finished before required navigation safety interruption")
            require(max_dist_from_start>0.5,
                    "vehicle never moved before safety test")
            result["pass"]=True
            print("SITL_BLACKOUT_HOLD_PASS "+json.dumps({
                "latency_s":result["hold_latency_s"],
                "moved_m":round(max_dist_from_start,2),
                "reached_before_hold":sorted(reached)}),flush=True)
        else:
            if args.bridge_mode=="sensor-eskf":
                p95=sorted(truth_ekf_errors)[int(0.95*(len(truth_ekf_errors)-1))]
                require(p95<5.0, f"ESKF SITL P95 position error too large: {p95:.2f} m")
            require(max_dist_from_start>15.0,
                    "no real vehicle movement seen in SITL physical truth")
            require(all(x<2.5 for x in closest[1:]),
                    "physical truth did not pass within 2.5m of each waypoint")
            require((len(targets)-1) in reached,
                    "ArduPilot did not confirm last waypoint reached")
            result["pass"]=True
            print("SITL_DYNAMIC_ROUTE_PASS "+json.dumps({
                "samples":warm_samples,
                "max_displacement_m":round(max_dist_from_start,2),
                "closest_m":result["closest_true_distance_to_route_m"],
                "mission_reached_events":sorted(reached),
                "ekf_physics_p95_m":result["ekf_physics_p95_horizontal_error_m"]}),
                  flush=True)
    except Exception as exc:
        failure=str(exc)
        result["pass"]=False
        result["failure"]=failure
        print("SITL_DYNAMIC_ROUTE_FAIL: "+failure, file=sys.stderr, flush=True)
    finally:
        try:
            # Actively stop simulated vehicle even on exception/timeout.
            link.set_mode("HOLD")
            link.mav.command_long_send(link.target_system, link.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
        except Exception:
            pass
        try:
            if bridge and bridge.stdin:
                bridge.stdin.close()
            if bridge:
                bridge.wait(timeout=5)
        except Exception:
            if bridge:
                bridge.kill()
                bridge.wait(timeout=3)
        if bridge is not None:
            result["bridge_exit_code"]=bridge.returncode
        csv_file.close()
        bridge_stdout.close()
        bridge_stderr.close()
        report_path.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    if failure:
        raise RuntimeError(failure)
    require(result["bridge_exit_code"]==0,
            "C++ dynamic bridge did not complete cleanly")


if __name__=="__main__":
    try:
        main()
    except Exception as exc:
        sys.exit("SITL_DYNAMIC_TEST_FAILED: "+str(exc))
