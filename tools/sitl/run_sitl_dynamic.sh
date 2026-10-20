#!/usr/bin/env bash
# SITL-only dynamic route. This intentionally exercises virtual propulsion;
# it must NEVER contact any non-loopback autopilot.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_DIR="${RUN_DIR:-$ROOT/sitl-dynamic-output}"
mkdir -p "$RUN_DIR"
BINARY="${ARDUROVER_BINARY:-$ROOT/sitl-smoke-output/ardurover}"
BRIDGE="${BOAT_DYNAMIC_BRIDGE_BINARY:-$ROOT/build-mavlink/boat_sitl_dynamic}"
test -x "$BINARY" || { echo "Missing SITL binary"; exit 2; }
test -x "$BRIDGE" || { echo "Missing C++ SITL dynamic bridge"; exit 2; }

cleanup() {
    local status=$?
    kill "${CONSOLE_PID:-}" "${SITL_PID:-}" 2>/dev/null || true
    wait "${CONSOLE_PID:-}" 2>/dev/null || true
    wait "${SITL_PID:-}" 2>/dev/null || true
    if [[ "$status" -ne 0 ]]; then
        echo "==== Failure diagnostics ===="
        for name in rover.log console.log dynamic_bridge.log; do
            echo "---- $name"
            tail -n 70 "$RUN_DIR/$name" 2>/dev/null || true
        done
        cat "$RUN_DIR/dynamic_mission_report.json" 2>/dev/null || true
    fi
}
trap cleanup EXIT
(
  cd "$RUN_DIR"
  stdbuf -oL -eL "$BINARY" --model rover \
    --home=-35.363261,149.165230,584,0 \
    --defaults="$ROOT/tools/sitl/rover_nogps.parm" --wipe \
    > "$RUN_DIR/rover.log" 2>&1
) &
SITL_PID=$!
python3 "$ROOT/tools/sitl/ekf_origin_probe.py" 5760 "$RUN_DIR" 170 \
    > "$RUN_DIR/console.log" 2>&1 &
CONSOLE_PID=$!
python3 - "$RUN_DIR" <<'PY'
import json, pathlib, sys, time
root=pathlib.Path(sys.argv[1])
deadline=time.monotonic()+35
while time.monotonic()<deadline:
    try:
        source=json.loads((root/"origin_result.json").read_text())
        params=json.loads((root/"parameters.json").read_text())["parameters"]
        if source.get("origin_set_confirmed") and params["GPS1_TYPE"]==0 and params["GPS2_TYPE"]==0 and params["EK3_SRC1_POSXY"]==6:
            print("DYNAMIC_SITL_GNSS_DISABLED_AND_ORIGIN_CONFIRMED",flush=True)
            break
    except (FileNotFoundError,KeyError,ValueError):
        time.sleep(0.2)
else:
    print((root/"console.log").read_text(errors="replace"),file=sys.stderr)
    sys.exit("SITL EKF origin not confirmed before dynamic test")
PY
echo "Priming EKF3 with valid origin-aligned stationary external odometry"
"$ROOT/build-mavlink/boat_sitl_bridge" --sitl-fixture --port 5762 --seconds 8 \
    | tee "$RUN_DIR/initial_external_nav.log"
grep -q 'packets=80 ' "$RUN_DIR/initial_external_nav.log"
echo "Launching physical Rover simulation with true SIM_STATE -> C++ external odometry"
python3 "$ROOT/tools/sitl/run_dynamic_mission.py" --sitl-only \
    --duration 135 --cpp-bridge "$BRIDGE" --output-dir "$RUN_DIR" \
    | tee "$RUN_DIR/dynamic_runner.log"
grep -q SITL_DYNAMIC_ROUTE_PASS "$RUN_DIR/dynamic_runner.log"
echo "PASS dynamic route from independent SIM_STATE physical truth"
