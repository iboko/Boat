#!/usr/bin/env bash
# FAIL-CLOSED headless Gazebo BlueBoat + real ArduPilot Rover JSON SITL.
# NEVER use physical hardware. Normal local-only MAVLink, fake IMU/radar/DVL.
set -euo pipefail
if [[ "$#" -ne 2 || "${1:-}" != "--sitl-only" ||
      ( "${2:-}" != "route" && "${2:-}" != "hold" ) ]]; then
  echo "Usage: bash tools/gazebo/run_blueboat_auto.sh --sitl-only route|hold" >&2
  exit 2
fi
MODE="$2"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${BLUEBOAT_OUTPUT_DIR:-$ROOT/blueboat-gazebo-output/$MODE}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
export SITL_MODELS_ROOT="${SITL_MODELS_ROOT:-$HOME/SITL_Models}"
export ASV_WAVE_SIM_ROOT="${ASV_WAVE_SIM_ROOT:-$HOME/gz_ws/src/asv_wave_sim}"
export ARDUPILOT_GAZEBO_BUILD="${ARDUPILOT_GAZEBO_BUILD:-$HOME/ardupilot_gazebo/build}"
export ASV_WAVE_SIM_PLUGIN_DIR="${ASV_WAVE_SIM_PLUGIN_DIR:-$HOME/gz_ws/install/lib}"
export BLUEBOAT_PREPARED_WORLD="$OUT/waves_with_blueboat.sdf"
export GZ_IP="127.0.0.1"
export GZ_SIM_RESOURCE_PATH="$SITL_MODELS_ROOT/Gazebo/models:$SITL_MODELS_ROOT/Gazebo/worlds:$ASV_WAVE_SIM_ROOT/gz-waves-models/models:$ASV_WAVE_SIM_ROOT/gz-waves-models/world_models:$ASV_WAVE_SIM_ROOT/gz-waves-models/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_BUILD:$ASV_WAVE_SIM_PLUGIN_DIR:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
# Gazebo finds the plugin .so through GZ_SIM_SYSTEM_PLUGIN_PATH, but
# its transitive Wave Sim shared libraries require the dynamic loader path.
export LD_LIBRARY_PATH="$ASV_WAVE_SIM_PLUGIN_DIR:$ARDUPILOT_GAZEBO_BUILD:${LD_LIBRARY_PATH:-}"

bash "$ROOT/tools/gazebo/check_blueboat.sh" --check-only |
  tee "$OUT/gazebo_preflight.log"
ARDUROVER_BINARY="${ARDUROVER_BINARY:-$ROOT/sitl-smoke-output/ardurover}"
SENSOR_BRIDGE="${SENSOR_BRIDGE_BINARY:-$ROOT/build-mavlink/boat_sitl_sensor}"
PRIMER="${STATIC_BRIDGE_BINARY:-$ROOT/build-mavlink/boat_sitl_bridge}"
for required in "$ARDUROVER_BINARY" "$SENSOR_BRIDGE" "$PRIMER"; do
  [[ -x "$required" ]] || { echo "Missing executable: $required" >&2; exit 3; }
done
# Prepare a mild but nonzero FFT wave baseline: original upstream
# world has wind_speed=5 and steepness=2, too rough for initial AHRS alignment.
# This is only the experiment checkout; document the actual water state.
python3 "$ROOT/tools/gazebo/tune_wave_scenario.py" \
  --model "$ASV_WAVE_SIM_ROOT/gz-waves-models/world_models/waves/model.sdf" \
  --wind-speed 1.0 --steepness 0.4 \
  | tee "$OUT/wave_scenario.log"
echo "Gazebo BlueBoat marine physics mode=$MODE; loopback only"
cleanup() {
  result=$?
  # Explicitly terminate the ArduRover binary PID, not merely its
  # launcher subshell. Gazebo's separate session also kills its children.
  kill "${TRUTH_PID:-}" "${EKF_PID:-}" "${ROVER_PID:-}" 2>/dev/null || true
  if [[ -n "${GZ_PID:-}" ]]; then
    kill -- "-$GZ_PID" 2>/dev/null || true
    kill "$GZ_PID" 2>/dev/null || true
    # Gazebo may block on JSON lockstep while a controller disconnects.
    # Bound shutdown; never wait forever for a stopped simulated world.
    for _ in $(seq 1 30); do
      if ! kill -0 "$GZ_PID" 2>/dev/null; then break; fi
      sleep 0.1
    done
    if kill -0 "$GZ_PID" 2>/dev/null; then
      echo "Killing unresponsive Gazebo simulator process group" >&2
      kill -KILL -- "-$GZ_PID" 2>/dev/null || true
      kill -KILL "$GZ_PID" 2>/dev/null || true
    fi
  fi
  wait "${TRUTH_PID:-}" 2>/dev/null || true
  wait "${EKF_PID:-}" 2>/dev/null || true
  wait "${ROVER_PID:-}" 2>/dev/null || true
  wait "${GZ_PID:-}" 2>/dev/null || true
  # Require that the next scenario may bind the same SITL endpoints.
  for _ in $(seq 1 30); do
    if ! (echo > /dev/tcp/127.0.0.1/5760) 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
  if [[ $result -ne 0 ]]; then
    echo "Gazebo BlueBoat integration FAILED. See $OUT" >&2
    for name in gazebo.log rover.log ekf_origin.log dynamic_bridge.log mission.log truth.log; do
      echo "--- $name" >&2
      tail -n 25 "$OUT/$name" >&2 2>/dev/null || true
    done
  fi
}
trap cleanup EXIT

# A dedicated process session lets cleanup kill the entire Gazebo process
# tree. Otherwise a second CI scenario can bind to a stale first world.
setsid gz sim -s -r -v3 "$BLUEBOAT_PREPARED_WORLD" > "$OUT/gazebo.log" 2>&1 &
GZ_PID=$!
echo "Waiting for REAL BlueBoat Gazebo model odometry topic"
READY=0
for _ in $(seq 1 120); do
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "Gazebo terminated without BlueBoat physics" >&2; exit 4
  fi
  if timeout 3 gz topic -l 2>/dev/null | grep -q '^/model/blueboat/odometry$'; then
    READY=1; break
  fi
  sleep 0.5
done
[[ "$READY" -eq 1 ]] || { echo "NO_BLUEBOAT_GAZEBO_ODOMETRY_TOPIC" >&2; exit 4; }

(
  cd "$OUT"
  # Official BlueBoat setup: JSON input at loopback 9002, skid-steer thrusters.
  # No real vehicle connection, motors only inside Gazebo.
  exec "$ARDUROVER_BINARY" --model JSON --home=-35.363261,149.165230,584,0 \
    --defaults="$ROOT/tools/gazebo/blueboat_nogps.parm" --wipe
) > "$OUT/rover.log" 2>&1 &
ROVER_PID=$!
python3 "$ROOT/tools/gazebo/gazebo_truth_monitor.py" \
  --seconds 420 --output "$OUT/gazebo_blueboat_truth.jsonl" \
  > "$OUT/truth.log" 2>&1 &
TRUTH_PID=$!
python3 - "$OUT/gazebo_blueboat_truth.jsonl" <<'PY'
import json,math,pathlib,sys,time
file=pathlib.Path(sys.argv[1])
for _ in range(150):
    try:
        obs=[json.loads(l) for l in file.read_text().splitlines()[-10:]]
        if len(obs)>=8 and all(o['source']=='Gazebo BlueBoat' and abs(float(o['z']))<3.0 for o in obs) and (
            time.monotonic()-obs[-1]['monotonic_s']<4):
            print('BLUEBOAT_GAZEBO_TRUTH_CONFIRMED',flush=True)
            break
    except (OSError,KeyError,ValueError,TypeError):
        pass
    time.sleep(0.2)
else:
    sys.exit("NO_LIVE_GAZEBO_BLUEBOAT_POSES before launching SITL")
PY

python3 "$ROOT/tools/sitl/ekf_origin_probe.py" 5760 "$OUT" 210 \
  > "$OUT/ekf_origin.log" 2>&1 &
EKF_PID=$!
python3 - "$OUT" <<'PY'
import json,pathlib,sys,time
root=pathlib.Path(sys.argv[1])
for _ in range(160):
    try:
        origin=json.loads((root/'origin_result.json').read_text())
        params=json.loads((root/'parameters.json').read_text())['parameters']
        if origin.get('origin_set_confirmed') and params['GPS1_TYPE']==0 and (
                params['GPS2_TYPE']==0 and params['EK3_SRC1_POSXY']==6):
            print('BLUEBOAT_NO_GNSS_EKF_ORIGIN_CONFIRMED',flush=True);break
    except (OSError,ValueError,KeyError):pass
    time.sleep(0.25)
else:sys.exit('FAIL: no confirmed no-GNSS origin for actual BlueBoat Gazebo')
PY

"$PRIMER" --sitl-fixture --port 5762 --seconds 8 \
  | tee "$OUT/initial_external_nav.log"
grep -q 'packets=80 ' "$OUT/initial_external_nav.log"
if [[ "$MODE" == "hold" ]]; then
  BRIDGE_MODE="sensor-blackout"
else
  BRIDGE_MODE="sensor-eskf"
fi
python3 "$ROOT/tools/sitl/run_dynamic_mission.py" \
  --sitl-only --bridge-mode "$BRIDGE_MODE" --duration 220 \
  --gazebo-truth-file "$OUT/gazebo_blueboat_truth.jsonl" \
  --cpp-bridge "$SENSOR_BRIDGE" --output-dir "$OUT" \
  | tee "$OUT/mission.log"
if [[ "$MODE" == "hold" ]]; then
  grep -q SITL_BLACKOUT_HOLD_PASS "$OUT/mission.log"
  echo "Observing 20 s of physical BlueBoat drift after HOLD"
  sleep 22
else
  grep -q SITL_DYNAMIC_ROUTE_PASS "$OUT/mission.log"
fi
# Fail if monitored gz source died and could not capture post-HOLD motion.
kill -0 "$TRUTH_PID" 2>/dev/null || true
python3 "$ROOT/tools/gazebo/verify_blueboat_trial.py" \
  --truth "$OUT/gazebo_blueboat_truth.jsonl" \
  --mission "$OUT/dynamic_mission_report.json" \
  --mode "$MODE" --output "$OUT/gazebo_blueboat_verification.json"
echo "BLUEBOAT_GAZEBO_VALIDATED_$MODE (actual Gazebo physics, not Rover substitute)"
