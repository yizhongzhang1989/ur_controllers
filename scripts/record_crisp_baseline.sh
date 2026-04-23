#!/usr/bin/env bash
# record_crisp_baseline.sh — Record a short regulation-hold baseline for one
# crisp role × UR arm combination and emit a manifest describing it.
#
# Usage:
#   scripts/record_crisp_baseline.sh <role> <robot> [duration_s]
#
#   role   := joint | cartesian | gravity
#   robot  := ur5e | ur15
#
# Output:
#   evaluation/baselines/crisp/<role>_<robot>.bag/           (gitignored, ROS2 bag dir)
#   evaluation/baselines/crisp/<role>_<robot>.manifest.yaml  (committed)
#
# What this does, end-to-end:
#   1. scripts/kill_sim.sh to clear stale state.
#   2. scripts/launch_sim.sh <robot> effort in background; wait until
#      joint_state_broadcaster and forward_effort_controller are active.
#   3. ros2 launch bringup/launch/crisp_bringup.launch.py robot=<robot>
#      mode=<mode>; wait until the role controller is active and
#      forward_effort_controller is inactive (strict swap).
#   4. For the joint role only, publish the current /joint_states positions
#      on /target_joint at 20 Hz (regulation-hold target). The cartesian and
#      gravity roles capture their own hold target at on_activate.
#   5. ros2 bag record -o <bag> <topics> for <duration_s> seconds.
#   6. Write the manifest YAML next to the bag.
#   7. Tear everything down (bag recorder, target pub, crisp bring-up, sim,
#      kill_sim.sh again).
#
# The bag payload is gitignored; only the manifest is committed. This matches
# docs/ROADMAP.md M2: "gitignored payload; commit a manifest .yaml of what was
# recorded."
#
# This script is NOT part of scripts/run_tests.sh. It is a one-shot baseline
# recorder and is intentionally kept out of the test gate.

set -e
set -o pipefail

ROLE="${1:-}"
ROBOT="${2:-}"
DURATION_S="${3:-10}"

usage() {
    sed -n '2,40p' "$0"
    exit 2
}

case "$ROLE" in
    joint|cartesian|gravity) ;;
    *) echo "bad role: '$ROLE'" >&2; usage ;;
esac
case "$ROBOT" in
    ur5e|ur15) ;;
    *) echo "bad robot: '$ROBOT'" >&2; usage ;;
esac
if ! [[ "$DURATION_S" =~ ^[0-9]+$ ]] || (( DURATION_S < 1 )); then
    echo "bad duration_s: '$DURATION_S' (want a positive integer)" >&2
    exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -f install/setup.bash ]]; then
    echo "install/setup.bash missing — build the workspace first." >&2
    exit 3
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble not found at /opt/ros/humble." >&2
    exit 3
fi

# Role → controller name (must match crisp_bringup.launch.py).
case "$ROLE" in
    joint)     CTRL="joint_impedance_controller" ;;
    cartesian) CTRL="cartesian_impedance_controller" ;;
    gravity)   CTRL="gravity_compensation" ;;
esac

OUT_DIR="$REPO_ROOT/evaluation/baselines/crisp"
BAG_DIR="$OUT_DIR/${ROLE}_${ROBOT}.bag"
MANIFEST="$OUT_DIR/${ROLE}_${ROBOT}.manifest.yaml"
LOG_DIR="$REPO_ROOT/.auto_dev_logs"
SIM_LOG="$LOG_DIR/record_sim_${ROLE}_${ROBOT}.log"
BRINGUP_LOG="$LOG_DIR/record_bringup_${ROLE}_${ROBOT}.log"
BAG_LOG="$LOG_DIR/record_bag_${ROLE}_${ROBOT}.log"
TARGET_LOG="$LOG_DIR/record_target_${ROLE}_${ROBOT}.log"

mkdir -p "$OUT_DIR" "$LOG_DIR"
# Remove any prior bag dir for this combo (ros2 bag refuses to overwrite).
rm -rf "$BAG_DIR"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

SIM_PGID=""
BRINGUP_PGID=""
TARGET_PGID=""
BAG_PGID=""

cleanup() {
    local rc=$?
    # Kill in reverse dependency order.
    for pgid in "$BAG_PGID" "$TARGET_PGID" "$BRINGUP_PGID" "$SIM_PGID"; do
        if [[ -n "$pgid" ]]; then
            kill -TERM -- "-$pgid" 2>/dev/null || true
        fi
    done
    # Give them a moment, then SIGKILL survivors.
    sleep 3
    for pgid in "$BAG_PGID" "$TARGET_PGID" "$BRINGUP_PGID" "$SIM_PGID"; do
        if [[ -n "$pgid" ]]; then
            kill -KILL -- "-$pgid" 2>/dev/null || true
        fi
    done
    "$REPO_ROOT/scripts/kill_sim.sh" >/dev/null 2>&1 || true
    exit "$rc"
}
trap cleanup EXIT INT TERM

spawn_pg() {
    # $1 = log file, rest = argv. Returns PGID on stdout.
    local logf="$1"; shift
    setsid bash -c 'exec "$@" >>"'"$logf"'" 2>&1' _ "$@" &
    local pid=$!
    # Under setsid the child becomes its own process group leader
    # with PGID == PID.
    echo "$pid"
}

say() { printf '[record_crisp_baseline] %s\n' "$*"; }

# --- 1. kill stale state -----------------------------------------------------
say "clearing stale sim / ports"
"$REPO_ROOT/scripts/kill_sim.sh" >/dev/null 2>&1 || true

# --- 2. launch sim -----------------------------------------------------------
say "launching sim: $ROBOT effort"
SIM_PGID=$(spawn_pg "$SIM_LOG" bash "$REPO_ROOT/scripts/launch_sim.sh" "$ROBOT" "effort")

# Helper: state of controller $1, via ros2 control list_controllers.
controller_state() {
    NO_COLOR=1 timeout 20 ros2 control list_controllers 2>/dev/null \
        | sed 's/\x1b\[[0-9;]*m//g' \
        | awk -v n="$1" '$1==n { print $NF }' \
        | head -n1
}

# Helper: wait up to $1 seconds for predicate function $2 to return 0.
wait_for() {
    local timeout="$1" pred="$2"
    local deadline=$(( $(date +%s) + timeout ))
    while (( $(date +%s) < deadline )); do
        if "$pred"; then return 0; fi
        sleep 2
    done
    return 1
}

sim_ready() {
    [[ "$(controller_state joint_state_broadcaster)" == "active" ]] \
        && [[ "$(controller_state forward_effort_controller)" == "active" ]]
}

say "waiting for sim baseline (joint_state_broadcaster + forward_effort_controller active)..."
if ! wait_for 150 sim_ready; then
    say "sim never became ready; see $SIM_LOG"
    exit 4
fi
say "sim baseline ready"

# --- 3. crisp bring-up -------------------------------------------------------
MODE="$ROLE"
say "launching crisp_bringup robot=$ROBOT mode=$MODE"
BRINGUP_PGID=$(spawn_pg "$BRINGUP_LOG" bash -c \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && \
     exec ros2 launch bringup/launch/crisp_bringup.launch.py \
         robot:=$ROBOT mode:=$MODE")

crisp_ready() {
    [[ "$(controller_state "$CTRL")" == "active" ]] \
        && [[ "$(controller_state forward_effort_controller)" == "inactive" ]]
}
say "waiting for $CTRL to become the sole active effort commander..."
if ! wait_for 90 crisp_ready; then
    say "crisp swap never landed; see $BRINGUP_LOG"
    exit 5
fi
say "$CTRL active, forward_effort_controller inactive"

# --- 4. optional target publisher (joint role only) --------------------------
TOPICS=(/joint_states "/$CTRL/tau_d")
case "$ROLE" in
    joint)
        say "reading current /joint_states to use as /target_joint..."
        Q_YAML="$(python3 - <<'PY'
import rclpy, sys, time
from sensor_msgs.msg import JointState
rclpy.init()
node = rclpy.create_node("_record_crisp_target_reader")
got = {"msg": None}
def cb(msg):
    if msg.name and len(msg.name) == len(msg.position):
        got["msg"] = msg
node.create_subscription(JointState, "/joint_states", cb, 10)
deadline = time.monotonic() + 10
while time.monotonic() < deadline and got["msg"] is None:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
m = got["msg"]
if m is None:
    sys.exit("no /joint_states message received")
names = list(m.name)
pos = [float(p) for p in m.position]
name_s = "[" + ", ".join(f"\"{n}\"" for n in names) + "]"
pos_s  = "[" + ", ".join(f"{p:.6f}" for p in pos) + "]"
print("{name: " + name_s + ", position: " + pos_s + "}")
PY
)"
        if [[ -z "$Q_YAML" ]]; then
            say "failed to read /joint_states; aborting"
            exit 6
        fi
        say "publishing /target_joint at 20 Hz (regulation hold)"
        TARGET_PGID=$(spawn_pg "$TARGET_LOG" bash -c \
            "source /opt/ros/humble/setup.bash && source install/setup.bash && \
             exec ros2 topic pub --rate 20 /target_joint sensor_msgs/msg/JointState \"$Q_YAML\"")
        TOPICS+=("/target_joint")
        # Let the controller settle on the target before recording.
        sleep 1.5
        ;;
    cartesian|gravity)
        # CartesianController captures its hold target at on_activate. No
        # publisher needed; the arm should hold the pose it had when the swap
        # landed.
        sleep 1.5
        ;;
esac

# --- 5. bag recorder ---------------------------------------------------------
say "recording ${DURATION_S}s to $BAG_DIR; topics=${TOPICS[*]}"
BAG_PGID=$(spawn_pg "$BAG_LOG" bash -c \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && \
     exec ros2 bag record -o '$BAG_DIR' ${TOPICS[*]}")

# Wait for the bag directory to exist (recorder has initialised) before
# timing the window — otherwise the first ~1–2 s is wasted on startup.
for _ in $(seq 1 20); do
    [[ -d "$BAG_DIR" ]] && break
    sleep 0.5
done
if [[ ! -d "$BAG_DIR" ]]; then
    say "bag recorder did not create $BAG_DIR; see $BAG_LOG"
    exit 7
fi

sleep "$DURATION_S"

# Stop the recorder gracefully so metadata.yaml is flushed.
say "stopping bag recorder"
if [[ -n "$BAG_PGID" ]]; then
    kill -INT -- "-$BAG_PGID" 2>/dev/null || true
    # Wait up to 10 s for clean shutdown.
    for _ in $(seq 1 20); do
        if ! kill -0 -- "-$BAG_PGID" 2>/dev/null; then break; fi
        sleep 0.5
    done
    # Belt and braces.
    kill -TERM -- "-$BAG_PGID" 2>/dev/null || true
    BAG_PGID=""
fi

if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
    say "warning: $BAG_DIR/metadata.yaml missing; bag may be incomplete"
fi

# --- 6. manifest -------------------------------------------------------------
# Pull message counts from the bag metadata if we can.
MSG_COUNT_YAML=""
if [[ -f "$BAG_DIR/metadata.yaml" ]]; then
    MSG_COUNT_YAML="$(python3 - "$BAG_DIR/metadata.yaml" <<'PY'
import sys, yaml
md = yaml.safe_load(open(sys.argv[1])) or {}
info = md.get("rosbag2_bagfile_information", {})
topics = info.get("topics_with_message_count", []) or []
lines = []
for t in topics:
    meta = t.get("topic_metadata", {}) or {}
    name = meta.get("name", "")
    typ = meta.get("type", "")
    count = t.get("message_count", 0)
    lines.append(f"  - name: \"{name}\"\n    type: \"{typ}\"\n    message_count: {count}")
print("\n".join(lines) if lines else "  []")
PY
)"
fi

NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
REL_BAG="$(python3 -c "import os,sys; print(os.path.relpath('$BAG_DIR','$REPO_ROOT'))")"

say "writing manifest $MANIFEST"
{
    echo "# Manifest for a crisp_controllers regulation-hold baseline rosbag."
    echo "# Generated by scripts/record_crisp_baseline.sh; bag payload is"
    echo "# gitignored, only this manifest is committed."
    echo "schema_version: 1"
    echo "scenario: regulation_hold"
    echo "role: \"$ROLE\""
    echo "controller_name: \"$CTRL\""
    echo "robot: \"$ROBOT\""
    echo "duration_s: $DURATION_S"
    echo "recorded_at: \"$NOW_UTC\""
    echo "bag:"
    echo "  path: \"$REL_BAG\""
    echo "  storage_id: sqlite3"
    echo "  committed: false  # payload is gitignored"
    echo "sim:"
    echo "  backend: mujoco"
    echo "  control_mode: effort"
    echo "  ur_type: \"$ROBOT\""
    echo "  # ur_simulator does not expose a separate RNG seed; the initial"
    echo "  # joint configuration is determined by its config.yaml."
    echo "  seed: null"
    echo "bringup:"
    echo "  launch: bringup/launch/crisp_bringup.launch.py"
    echo "  args:"
    echo "    robot: \"$ROBOT\""
    echo "    mode: \"$MODE\""
    echo "  config: \"bringup/config/crisp_${CTRL%_controller}.${ROBOT}.yaml\""
    echo "topics:"
    for t in "${TOPICS[@]}"; do
        echo "  - \"$t\""
    done
    if [[ -n "$MSG_COUNT_YAML" ]]; then
        echo "bag_contents:"
        echo "$MSG_COUNT_YAML"
    fi
    echo "recorder:"
    echo "  script: scripts/record_crisp_baseline.sh"
    echo "  version: 1"
    echo "notes: >"
    case "$ROLE" in
        joint)
            echo "  Regulation hold: target equals the initial /joint_states sample,"
            echo "  published on /target_joint at 20 Hz. The joint_impedance_controller"
            echo "  role uses nullspace PD with projector_type=none, i.e. direct joint PD."
            ;;
        cartesian)
            echo "  Regulation hold: on_activate the CartesianController captures the"
            echo "  current tool0 pose as its Cartesian target and the current joint"
            echo "  config as the nullspace target. No external target is published."
            ;;
        gravity)
            echo "  Regulation hold: task and nullspace stiffnesses are zero, so the"
            echo "  role is a pure pinocchio gravity + Coriolis feed-forward. No"
            echo "  external target is published."
            ;;
    esac
} > "$MANIFEST"

say "DONE  role=$ROLE  robot=$ROBOT  bag=$REL_BAG  manifest=$MANIFEST"
