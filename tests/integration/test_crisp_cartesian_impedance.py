"""Integration test for crisp cartesian_impedance_controller in sim.

For each UR arm in {ur5e, ur15}:

  1. Bring up the MuJoCo sim (``scripts/launch_sim.sh <robot> effort``)
     and wait until ``joint_state_broadcaster`` and
     ``forward_effort_controller`` are active.
  2. ``ros2 launch bringup/launch/crisp_bringup.launch.py robot:=<robot>
     mode:=cartesian`` — spawns ``cartesian_impedance_controller`` as
     type ``crisp_controllers/CartesianController`` using the per-arm
     YAML under ``bringup/config/``, then atomically swaps
     ``forward_effort_controller`` for it.
  3. Assert the swap landed: ``cartesian_impedance_controller`` is
     active and ``forward_effort_controller`` is inactive.
  4. On activation the CartesianController captures the current tool0
     pose as its target (see
     ``third_party/crisp_controllers/src/cartesian_controller.cpp::on_activate``)
     and the nullspace ``q_target`` to the current joint config, so the
     default behaviour is "hold here". Sample ``/joint_states`` over a
     fixed window and assert max per-joint deviation from the initial
     configuration stays within a generous tolerance — this verifies
     the controller is actually holding the arm under non-zero
     Cartesian stiffness (and not crashing or commanding wild torques).

Teardown kills the crisp bring-up process, then the sim launcher,
then the port/process reaper as a belt-and-braces cleanup.

Skipped when ROS 2 Humble or the built install/ is missing.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_SIM = REPO_ROOT / "scripts" / "launch_sim.sh"
KILL_SIM = REPO_ROOT / "scripts" / "kill_sim.sh"
INSTALL_SETUP = REPO_ROOT / "install" / "setup.bash"
CRISP_BRINGUP_LAUNCH = REPO_ROOT / "bringup" / "launch" / "crisp_bringup.launch.py"

EXPECTED_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Timeouts sized for MuJoCo cold-start + pinocchio model load in crisp.
SIM_READY_TIMEOUT_S = 120
CRISP_READY_TIMEOUT_S = 60
# Max |q - q0| allowed (rad) during cartesian hold. Generous: the
# controller only has to avoid diverging under nominal k_pos ~ 400 and
# k_rot ~ 30, not track perfectly.
REGULATION_TOLERANCE_RAD = 0.15
# How long we observe /joint_states during regulation.
REGULATION_WINDOW_S = 5.0


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
# Helpers (kept in sync with test_crisp_joint_impedance.py; duplicated
# rather than factored out so each test file can be read in isolation).
# ---------------------------------------------------------------------------


_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def _bash(cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
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


def _controller_states() -> dict[str, str]:
    cp = _bash("NO_COLOR=1 timeout 20 ros2 control list_controllers", timeout=30)
    states: dict[str, str] = {}
    if cp.returncode != 0:
        return states
    for raw in cp.stdout.splitlines():
        line = _ANSI_RE.sub("", raw).strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        name = parts[0].split("[")[0]
        state = parts[-1]
        if state in ("active", "inactive", "unconfigured", "finalized"):
            states[name] = state
    return states


def _joint_state_positions() -> dict[str, float] | None:
    cp = _bash("timeout 5 ros2 topic echo --once /joint_states", timeout=15)
    if cp.returncode != 0 or not cp.stdout:
        return None

    names: list[str] | None = None
    positions: list[float] | None = None
    current_list: list | None = None
    for raw in cp.stdout.splitlines():
        line = raw.rstrip()
        stripped = line.strip()
        if stripped.startswith("name:"):
            rest = stripped[len("name:"):].strip()
            if rest.startswith("[") and rest.endswith("]"):
                names = [n.strip().strip("'\"") for n in rest[1:-1].split(",") if n.strip()]
                current_list = None
            else:
                names = []
                current_list = names
            continue
        if stripped.startswith("position:"):
            rest = stripped[len("position:"):].strip()
            if rest.startswith("[") and rest.endswith("]"):
                positions = [float(x) for x in rest[1:-1].split(",") if x.strip()]
                current_list = None
            else:
                positions = []
                current_list = positions
            continue
        if stripped.startswith(("velocity:", "effort:", "header:", "---")):
            current_list = None
            continue
        if current_list is not None and stripped.startswith("- "):
            val = stripped[2:].strip().strip("\"'")
            if current_list is names:
                current_list.append(val)
            else:
                try:
                    current_list.append(float(val))
                except ValueError:
                    pass
    if not names or not positions or len(names) != len(positions):
        return None
    return dict(zip(names, positions))


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


def _spawn_in_pgid(argv: list[str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(exist_ok=True, parents=True)
    log = log_path.open("w")
    return subprocess.Popen(
        argv,
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def _kill_pg(proc: subprocess.Popen | None, timeout: float = 10.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


# ---------------------------------------------------------------------------
# Fixture: sim + crisp cartesian-impedance bring-up
# ---------------------------------------------------------------------------


@pytest.fixture(scope="function")
def crisp_cartesian_up(request):
    robot: str = request.param
    subprocess.run([str(KILL_SIM)], check=False)

    log_dir = REPO_ROOT / ".auto_dev_logs"
    sim_log = log_dir / f"crisp_cart_sim_{robot}.log"
    bringup_log = log_dir / f"crisp_cart_bringup_{robot}.log"

    sim_proc = _spawn_in_pgid(
        ["bash", str(LAUNCH_SIM), robot, "effort"], sim_log
    )
    bringup_proc: subprocess.Popen | None = None
    try:
        def sim_ready() -> bool:
            states = _controller_states()
            return (
                states.get("joint_state_broadcaster") == "active"
                and states.get("forward_effort_controller") == "active"
            )

        assert _wait_until(sim_ready, SIM_READY_TIMEOUT_S, interval=2.0), (
            f"Sim never reached the expected baseline for {robot}. "
            f"Last states={_controller_states()}. See {sim_log}."
        )

        bringup_argv = [
            "bash",
            "-c",
            (
                "source /opt/ros/humble/setup.bash && "
                f"source {INSTALL_SETUP} && "
                f"ros2 launch {CRISP_BRINGUP_LAUNCH} "
                f"robot:={robot} mode:=cartesian"
            ),
        ]
        bringup_proc = _spawn_in_pgid(bringup_argv, bringup_log)

        yield robot, sim_proc, bringup_proc, sim_log, bringup_log
    finally:
        _kill_pg(bringup_proc)
        _kill_pg(sim_proc)
        subprocess.run([str(KILL_SIM)], check=False)


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("crisp_cartesian_up", ["ur5e", "ur15"], indirect=True)
def test_crisp_cartesian_impedance_hold_pose(crisp_cartesian_up):
    robot, _sim_proc, _bringup_proc, sim_log, bringup_log = crisp_cartesian_up

    # 1. Wait for the swap to land.
    def crisp_active() -> bool:
        states = _controller_states()
        return (
            states.get("cartesian_impedance_controller") == "active"
            and states.get("forward_effort_controller") == "inactive"
        )

    assert _wait_until(crisp_active, CRISP_READY_TIMEOUT_S, interval=2.0), (
        f"cartesian_impedance_controller did not become the sole active "
        f"effort commander for {robot}. States={_controller_states()}. "
        f"See {bringup_log} (bring-up) and {sim_log} (sim)."
    )

    # 2. Capture the initial joint configuration. CartesianController's
    #    on_activate already set target_pose to the current tool0 pose
    #    and q_target to the current joint vector, so no extra target
    #    publish is needed — the controller should just hold here.
    q0 = _joint_state_positions()
    assert q0 is not None, (
        f"No /joint_states message after crisp activation ({robot})."
    )
    for j in EXPECTED_JOINTS:
        assert j in q0, f"joint {j!r} missing from /joint_states: {q0}"

    # Give the controller a moment to settle after the swap transient.
    time.sleep(1.5)

    # 3. Sample for REGULATION_WINDOW_S, assert bounded drift.
    deadline = time.monotonic() + REGULATION_WINDOW_S
    worst = 0.0
    samples = 0
    while time.monotonic() < deadline:
        q = _joint_state_positions()
        if q is None:
            continue
        samples += 1
        for j in EXPECTED_JOINTS:
            err = abs(q[j] - q0[j])
            if err > worst:
                worst = err
        time.sleep(0.25)

    assert samples >= 3, (
        f"Only {samples} /joint_states samples in {REGULATION_WINDOW_S}s "
        f"for {robot}; /joint_states appears to have stalled. "
        f"See {sim_log}."
    )
    assert worst < REGULATION_TOLERANCE_RAD, (
        f"crisp cartesian_impedance_controller drifted {worst:.3f} rad "
        f"from the initial pose on {robot} (tolerance "
        f"{REGULATION_TOLERANCE_RAD}). See {sim_log} and {bringup_log}."
    )
