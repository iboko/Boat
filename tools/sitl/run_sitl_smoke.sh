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
    if [[ -n "${CONSOLE_PID:-}" ]]; then kill "$CONSOLE_PID" 2>/dev/null || true; fi
    if [[ -n "${SITL_PID:-}" ]]; then kill "$SITL_PID" 2>/dev/null || true; fi
    wait "${CONSOLE_PID:-}" 2>/dev/null || true
    wait "${SITL_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching official ArduRover SITL binary (no arming, no waypoints)"
(
    cd "$RUN_DIR"
    "$BINARY" --model boat --home=-35.363261,149.165230,584,0 \
        --defaults="$ROOT/tools/sitl/rover_nogps.parm" --wipe \
        > "$RUN_DIR/rover_sitl.log" 2>&1
) &
SITL_PID=$!
python3 "$ROOT/tools/sitl/console_keeper.py" 5760 > "$RUN_DIR/console.log" 2>&1 &
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

echo "Running 10Hz no-GNSS fixture on C++ companion"
"$BRIDGE" --sitl-fixture --port 5762 --seconds 8 | tee "$RUN_DIR/bridge.log"
grep -q 'packets=80 ' "$RUN_DIR/bridge.log"
grep -q 'GNSS=denied' "$RUN_DIR/bridge.log"
echo "PASS: real Rover SITL heartbeat and C++ 10Hz ODOMETRY transport (NOT a mission test)"
