#!/usr/bin/env bash
# kill_sim.sh — Ensure nothing is holding the simulator's ports or processes.
#
# Kills stale ur_simulator / ros2_control / rosbridge / dashboard processes and
# frees the rosbridge (9090) and dashboard (8000) TCP ports. Safe to call
# before every simulator launch.

set -u

PORTS=("${@:-9090 8000}")
# Allow explicit override: `kill_sim.sh 9090 8000 12345`
if [[ $# -eq 0 ]]; then
    PORTS=(9090 8000)
fi

echo "[kill_sim] killing stale simulator processes..."
# Patterns that cover both MuJoCo and Gazebo backends of ur_simulator plus
# rosbridge, the dashboard HTTP server, and the ros2 CLI daemon (which
# caches discovery state and must be reset across sim generations).
PATTERNS=(
    "ur_sim_mujoco.launch.py"
    "ur_sim_effort.launch.py"
    "ur_sim_control.launch.py"
    "mujoco_ros2_control"
    "ros2_control_node"
    "ign gazebo"
    "gz sim"
    "parameter_bridge"
    "robot_state_publisher"
    "rosbridge_websocket"
    "ur_web_dashboard/server.py"
    "gravity_compensation.py"
    "_ros2_daemon"
    "ros2cli.daemon"
)
for p in "${PATTERNS[@]}"; do
    pkill -9 -f "$p" 2>/dev/null || true
done

echo "[kill_sim] freeing ports: ${PORTS[*]}"
for port in "${PORTS[@]}"; do
    # fuser is in psmisc; fall back to lsof if missing.
    if command -v fuser >/dev/null 2>&1; then
        fuser -k -TERM "${port}/tcp" 2>/dev/null || true
        sleep 0.2
        fuser -k -KILL "${port}/tcp" 2>/dev/null || true
    elif command -v lsof >/dev/null 2>&1; then
        pids=$(lsof -ti ":${port}" 2>/dev/null || true)
        [[ -n "$pids" ]] && kill -9 $pids 2>/dev/null || true
    fi
done

# brief settle so subsequent launches don't race against teardown
sleep 1
echo "[kill_sim] done"
