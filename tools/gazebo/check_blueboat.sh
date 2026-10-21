#!/usr/bin/env bash
# Dependency preflight ONLY. Never arms motors or starts ArduPilot / Gazebo.
set -euo pipefail
if [[ "${1:-}" != "--check-only" || "$#" -ne 1 ]]; then
  echo "Usage: SITL_MODELS_ROOT=... ASV_WAVE_SIM_ROOT=... ARDUPILOT_GAZEBO_BUILD=... bash tools/gazebo/check_blueboat.sh --check-only" >&2
  exit 2
fi
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SITL_MODELS_ROOT="${SITL_MODELS_ROOT:-$HOME/SITL_Models}"
ASV_WAVE_SIM_ROOT="${ASV_WAVE_SIM_ROOT:-$HOME/asv_wave_sim}"
ARDUPILOT_GAZEBO_BUILD="${ARDUPILOT_GAZEBO_BUILD:-$HOME/ardupilot_gazebo/build}"
ASV_WAVE_SIM_PLUGIN_DIR="${ASV_WAVE_SIM_PLUGIN_DIR:-$HOME/gz_ws/install/lib}"
WAVES_WORLD="$ASV_WAVE_SIM_ROOT/gz-waves-models/worlds/waves.sdf"
BLUEBOAT_MODEL="$SITL_MODELS_ROOT/Gazebo/models/blueboat/model.sdf"
for cmd in gz python3; do
  command -v "$cmd" >/dev/null || { echo "Missing executable: $cmd" >&2; exit 1; }
done
[[ -f "$BLUEBOAT_MODEL" ]] || { echo "Missing BlueBoat model: $BLUEBOAT_MODEL" >&2; exit 1; }
[[ -f "$WAVES_WORLD" ]] || { echo "Missing upstream Wave Sim world: $WAVES_WORLD" >&2; exit 1; }
[[ -d "$ARDUPILOT_GAZEBO_BUILD" ]] || { echo "Missing compiled ArduPilot Gazebo plugin build directory" >&2; exit 1; }
if ! find -L "$ARDUPILOT_GAZEBO_BUILD" -maxdepth 4 -type f \( -name '*.so' -o -name '*.dylib' \) | grep -q .; then
  echo "No compiled Gazebo plugin shared libraries: $ARDUPILOT_GAZEBO_BUILD" >&2
  exit 1
fi
[[ -d "$ASV_WAVE_SIM_PLUGIN_DIR" ]] || {
  echo "Missing ASV Wave Sim compiled hydrodynamics plugin directory" >&2; exit 1;
}
if ! find -L "$ASV_WAVE_SIM_PLUGIN_DIR" -maxdepth 6 -type f -name '*hydrodynamics*.so' | grep -q .; then
  echo "No compiled ASV Wave Sim hydrodynamics shared library was found" >&2
  exit 1
fi
grep -q 'gz-waves1-hydrodynamics-system' "$BLUEBOAT_MODEL" || {
  echo "BlueBoat model does not reference expected wave hydrodynamics plugin" >&2
  exit 1
}
grep -q 'filename="ArduPilotPlugin"' "$BLUEBOAT_MODEL" || {
  echo "ArduPilot JSON bridge missing in BlueBoat SDF" >&2; exit 1;
}
grep -q 'cmd_thrust' "$BLUEBOAT_MODEL" || {
  echo "BlueBoat SDF missing commanded propulsor channels" >&2; exit 1;
}
OUTPUT="${BLUEBOAT_PREPARED_WORLD:-$ROOT/build-blueboat/waves_with_blueboat.sdf}"
python3 "$ROOT/tools/gazebo/prepare_blueboat_world.py" \
  --upstream-world "$WAVES_WORLD" --output "$OUTPUT"
echo "PREPARED_GAZEBO_BLUEBOAT_WORLD=$OUTPUT"
echo "Required for manual physics-only launch (not executed):"
printf 'export GZ_SIM_RESOURCE_PATH="%s:%s:%s:${GZ_SIM_RESOURCE_PATH:-}"\n' \
 "$SITL_MODELS_ROOT/Gazebo/models" "$ASV_WAVE_SIM_ROOT/gz-waves-models/models" "$ASV_WAVE_SIM_ROOT/gz-waves-models/worlds"
printf 'export GZ_SIM_SYSTEM_PLUGIN_PATH="%s:%s:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"\n' \
 "$ARDUPILOT_GAZEBO_BUILD" "$ASV_WAVE_SIM_PLUGIN_DIR"
printf 'gz sim -s -r -v4 "%s"\n' "$OUTPUT"
echo "Important: this did NOT run Gazebo, hydrodynamics, ArduPilot or Boat's ESKF."
