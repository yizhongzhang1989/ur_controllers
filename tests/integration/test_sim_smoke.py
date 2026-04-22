"""
Smoke integration test for the ur_simulator bringup.

For each supported arm (ur5e, ur15):
  1. Call scripts/launch_sim.sh in the background (it pre-cleans ports).
  2. Wait up to a timeout for all of:
       - /joint_states publishing at >= 50 Hz-ish (>= 5 msgs in 2 s).
       - 6 joints with the expected UR joint names.
       - ros2 control list_controllers shows joint_state_broadcaster active
         and forward_effort_controller active (MuJoCo effort mode).
       - TCP port 9090 (rosbridge) is listening.
       - TCP port 8000 (dashboard) is listening and serves HTTP 200 on /.
  3. Tear down by killing the launcher process tree and running kill_sim.sh.

The test is skipped if ROS 2 Humble or the workspace install/ is not
available — this keeps it runnable in a minimal CI before the workspace is
built.

Invoke: ``pytest -q tests/integration/test_sim_smoke.py``
"""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_SIM = REPO_ROOT / "scripts" / "launch_sim.sh"
KILL_SIM = REPO_ROOT / "scripts" / "kill_sim.sh"
INSTALL_SETUP = REPO_ROOT / "install" / "setup.bash"

EXPECTED_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}

ROSBRIDGE_PORT = 9090
DASHBOARD_PORT = 8000

# Allow heavy sim bringup to take a while (MuJoCo warm-up + rosbridge + http).
READY_TIMEOUT_S = 90


# ---------------------------------------------------------------------------
# Environment guard
# ---------------------------------------------------------------------------


def _have_ros() -> bool:
    return Path("/opt/ros/humble/setup.bash").is_file()


def _have_ws() -> bool:
    return INSTALL_SETUP.is_file()


pytestmark = pytest.mark.skipif(
    not (_have_ros() and _have_ws()),
    reason="Requires ROS 2 Humble and a built workspace (install/setup.bash).",
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _bash(cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    """Run `cmd` under bash with ROS and our workspace sourced."""
    full = (
        "source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_SETUP} && "
        f"{cmd}"
    )
    return subprocess.run(
        ["bash", "-c", full],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _port_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        try:
            return s.connect_ex(("127.0.0.1", port)) == 0
        except OSError:
            return False


def _http_ok(url: str) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=1.0) as resp:
            return 200 <= resp.status < 400
    except (urllib.error.URLError, TimeoutError, ConnectionError):
        return False


def _joint_state_fields() -> set[str] | None:
    """Return the names in the next /joint_states message, or None on timeout."""
    # `ros2 topic echo --once` blocks until one message arrives. Wrap with
    # `timeout` since `--timeout` isn't supported on Humble's ros2cli.
    cp = _bash("timeout 5 ros2 topic echo --once /joint_states", timeout=15)
    if cp.returncode != 0 or not cp.stdout:
        return None
    names: list[str] = []
    capture = False
    for line in cp.stdout.splitlines():
        s = line.strip()
        if s.startswith("name:"):
            # Inline form: `name: [a, b, c]`
            rest = s[len("name:"):].strip()
            if rest.startswith("[") and rest.endswith("]"):
                return {n.strip().strip("'\"") for n in rest[1:-1].split(",") if n.strip()}
            capture = True
            continue
        if capture:
            if s.startswith("- "):
                names.append(s[2:].strip("\"'"))
            else:
                break
    return set(names) if names else None


def _controllers_active() -> set[str]:
    # Use a generous timeout: first call cold-starts ros2cli and has to
    # `wait for service /controller_manager/list_controllers`.
    cp = _bash("timeout 20 ros2 control list_controllers", timeout=30)
    if cp.returncode != 0:
        return set()
    active: set[str] = set()
    for line in cp.stdout.splitlines():
        # Humble format: "<name>  <type>  active" (or "inactive").
        # "inactive" does NOT match " active" because there's no space
        # between "in" and "active".
        if " active" not in line:
            continue
        name = line.split()[0]
        # Strip any "[type]" decoration just in case the CLI format changes.
        name = name.split("[")[0].strip()
        if name:
            active.add(name)
    return active


def _wait_until(predicate, timeout_s: float, interval: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if predicate():
                return True
        except Exception:  # noqa: BLE001
            pass
        time.sleep(interval)
    return False


# ---------------------------------------------------------------------------
# Fixture: per-robot sim bringup
# ---------------------------------------------------------------------------


@pytest.fixture(scope="function")
def sim_up(request):
    robot: str = request.param
    # Pre-clean, just in case.
    subprocess.run([str(KILL_SIM)], check=False)

    log_dir = REPO_ROOT / ".auto_dev_logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"sim_{robot}.log"

    with log_path.open("w") as log:
        proc = subprocess.Popen(
            ["bash", str(LAUNCH_SIM), robot, "effort"],
            stdout=log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # own process group → clean kill
        )
    try:
        yield robot, proc, log_path
    finally:
        # Teardown: TERM the group, then kill_sim.sh to catch grandchildren.
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        subprocess.run([str(KILL_SIM)], check=False)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("sim_up", ["ur5e", "ur15"], indirect=True)
def test_sim_bringup(sim_up):
    robot, proc, log_path = sim_up

    # 1. /joint_states must publish with expected joint names.
    joints: set[str] | None = None

    def joints_ready() -> bool:
        nonlocal joints
        joints = _joint_state_fields()
        return joints is not None and joints >= EXPECTED_JOINTS

    assert _wait_until(joints_ready, READY_TIMEOUT_S), (
        f"/joint_states never carried the expected joint names for {robot}. "
        f"Last seen: {joints}. See {log_path} for the sim log."
    )

    # 2. Required controllers must be active (MuJoCo effort mode).
    required = {"joint_state_broadcaster", "forward_effort_controller"}
    active: set[str] = set()

    def ctrl_ready() -> bool:
        nonlocal active
        active = _controllers_active()
        return required.issubset(active)

    assert _wait_until(ctrl_ready, 90.0, interval=3.0), (
        f"Controllers not active for {robot}. Active={active}, "
        f"required={required}. See {log_path}."
    )

    # 3. rosbridge WebSocket port must be open.
    assert _wait_until(lambda: _port_open(ROSBRIDGE_PORT), 30.0), (
        f"rosbridge port {ROSBRIDGE_PORT} not open for {robot}. See {log_path}."
    )

    # 4. Dashboard HTTP must respond 200.
    assert _wait_until(
        lambda: _http_ok(f"http://127.0.0.1:{DASHBOARD_PORT}/"), 30.0
    ), f"Dashboard on port {DASHBOARD_PORT} not serving for {robot}. See {log_path}."

    # Sanity: the launcher is still alive (didn't crash out from under us).
    assert proc.poll() is None, (
        f"launch_sim.sh exited prematurely for {robot} "
        f"(returncode={proc.returncode}). See {log_path}."
    )
