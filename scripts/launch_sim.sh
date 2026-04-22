#!/usr/bin/env bash
# launch_sim.sh — Bring up the ur_simulator full stack (MuJoCo + rosbridge +
# web dashboard) for a given UR arm, from our parent workspace.
#
# Usage:
#   scripts/launch_sim.sh                          # defaults: ur5e, effort mode
#   scripts/launch_sim.sh ur15                     # ur15, effort mode
#   scripts/launch_sim.sh ur5e position            # ur5e, position mode
#   scripts/launch_sim.sh ur5e effort path/to/controllers.yaml
#
# Effort mode is the default because crisp and our own joint impedance
# controller both need effort command interfaces.
#
# This wraps third_party/ur_simulator/launch_all.sh:
#  - Calls scripts/kill_sim.sh first (stale processes / ports).
#  - Sources ROS 2 Humble and OUR parent install/ (so crisp_controllers and
#    our own packages are discoverable alongside the sim).
#  - Runs the sim's own launch_all.sh which handles dashboard + rosbridge.

# Intentionally NOT `set -u`: ROS 2 setup.bash touches unbound vars
# (AMENT_TRACE_SETUP_FILES, COLCON_*, etc.) and would abort the script.
set -e

ROBOT="${1:-ur5e}"
CONTROL_MODE="${2:-effort}"
CONTROLLERS_FILE="${3:-}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="$REPO_ROOT/third_party/ur_simulator"

case "$ROBOT" in
    ur3|ur3e|ur5|ur5e|ur7e|ur8long|ur10|ur10e|ur12e|ur15|ur16e|ur18|ur20|ur30) ;;
    *) echo "unknown robot: $ROBOT" >&2; exit 2 ;;
esac
case "$CONTROL_MODE" in
    position|effort) ;;
    *) echo "unknown control_mode: $CONTROL_MODE" >&2; exit 2 ;;
esac

# 1. Clear stale state (ports 9090 rosbridge + 8000 dashboard + sim procs).
"$REPO_ROOT/scripts/kill_sim.sh"

# 2. Source envs.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"

# 3. Ensure the sim has its config file (first-run auto-generates from template
#    but we also want to force the arm type we were asked for).
CFG="$SIM_DIR/config/config.yaml"
if [[ ! -f "$CFG" ]]; then
    cp "$SIM_DIR/config/config.template.yaml" "$CFG"
fi
# Best-effort patch of ur_type in the YAML.
if grep -q '^ur_type:' "$CFG"; then
    sed -i "s/^ur_type:.*/ur_type: ${ROBOT}/" "$CFG"
else
    echo "ur_type: ${ROBOT}" >> "$CFG"
fi

# 4. Delegate to the simulator's own launcher. We run it from the sim dir so
#    its relative paths (URDF generation, dashboard static files) resolve.
cd "$SIM_DIR"
EXTRA=()
if [[ -n "$CONTROLLERS_FILE" ]]; then
    EXTRA+=(--controllers_file "$CONTROLLERS_FILE")
fi

echo "[launch_sim] robot=$ROBOT control_mode=$CONTROL_MODE"
exec ./launch_all.sh --simulator mujoco --control_mode "$CONTROL_MODE" "${EXTRA[@]}" "$CFG"
