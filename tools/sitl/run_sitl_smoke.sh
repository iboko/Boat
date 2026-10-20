#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_DIR="${RUN_DIR:-$ROOT/sitl-smoke-output}"
mkdir -p "$RUN_DIR"
BINARY="${ARDUROVER_BINARY:-$RUN_DIR/ardurover}"
BRIDGE="${BOAT_BRIDGE_BINARY:-$ROOT/build-mavlink/boat_sitl_bridge}"
if [[ ! -x "$BINARY" ]]; then echo "Missing Rover SITL executable: $BINARY"; exit 2; fi
if [[ ! -x "$BRIDGE" ]]; then echo "Missing Boat bridge: $BRIDGE"; exit 2; fi

cleanup() {
    local status=$?
    if [[ "$status" -ne 0 ]]; then
        echo "===== Rover process diagnostics ====="
        ps -ef | grep -E "ardurover|run_sitl|console_keeper" | grep -v grep || true
        echo "===== Rover binary diagnostics ====="
        file "$BINARY" || true
        ldd "$BINARY" || true
        echo "===== TCP sockets ====="
        (ss -lnt || true)
        echo "===== Rover SITL diagnostics ====="
        cat "$RUN_DIR/rover_sitl.log" 2>/dev/null || true
        echo "===== SERIAL0 console diagnostics ====="
        cat "$RUN_DIR/console.log" 2>/dev/null || true
        echo "===== Bridge diagnostics ====="
        cat "$RUN_DIR/bridge.log" 2>/dev/null || true
    fi
    if [[ -n "${CONSOLE_PID:-}" ]]; then kill "$CONSOLE_PID" 2>/dev/null || true; fi
    if [[ -n "${SITL_PID:-}" ]]; then kill "$SITL_PID" 2>/dev/null || true; fi
    wait "${CONSOLE_PID:-}" 2>/dev/null || true
    wait "${SITL_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching official ArduRover SITL binary (no arming, no waypoints)"
(
    cd "$RUN_DIR"
    stdbuf -oL -eL "$BINARY" --model rover --home=-35.363261,149.165230,584,0 \
        --defaults="$ROOT/tools/sitl/rover_nogps.parm" --wipe \
        > "$RUN_DIR/rover_sitl.log" 2>&1
) &
SITL_PID=$!
python3 "$ROOT/tools/sitl/console_keeper.py" 5760 "$RUN_DIR/parameters.json" > "$RUN_DIR/console.log" 2>&1 &
CONSOLE_PID=$!

echo "Waiting for companion TCP 5762"
python3 - <<'PY'
import socket
import time
import sys
deadline = time.monotonic() + 35
while time.monotonic() < deadline:
    try:
        with socket.create_connection(("127.0.0.1", 5762), timeout=0.5):
            break
    except OSError:
        time.sleep(0.25)
else:
    print("Companion TCP port did not open", file=sys.stderr)
    sys.exit(4)
PY

echo "Verifying GNSS-denied EKF3 source selection from actual Rover telemetry"
python3 - "$RUN_DIR/parameters.json" "$RUN_DIR/console.log" <<'PY'
import pathlib, sys, time
path=pathlib.Path(sys.argv[1])
log=pathlib.Path(sys.argv[2])
for _ in range(200):
    if path.is_file() and path.stat().st_size>0:
        break
    time.sleep(0.2)
else:
    print("Live SITL parameter verification failed. Diagnostics:",file=sys.stderr)
    if log.exists():
        print(log.read_text(errors="replace"),file=sys.stderr)
    sys.exit(5)
print(path.read_text(),flush=True)
PY

echo "Running 10Hz no-GNSS fixture on C++ companion"
"$BRIDGE" --sitl-fixture --port 5762 --seconds 8 | tee "$RUN_DIR/bridge.log"
grep -q 'packets=80 ' "$RUN_DIR/bridge.log"
grep -q 'GNSS=denied' "$RUN_DIR/bridge.log"
grep -q 'GNSS_DISABLED_CONFIRMED' "$RUN_DIR/console.log"
echo "PASS: real Rover SITL heartbeat and C++ 10Hz ODOMETRY transport (NOT a mission test)"
