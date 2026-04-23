"""Integration test for cartesian_motion_controller in sim.

For each UR arm in {ur5e, ur15}:

  1. Bring up the MuJoCo sim in **position mode**
     (``scripts/launch_sim.sh <robot> position``) and wait until
     ``joint_state_broadcaster`` and ``joint_trajectory_controller``
     are active.
  2. ``ros2 launch bringup/launch/cartesian_bringup.launch.py
     robot:=<robot>`` — spawns ``cartesian_motion_controller`` as type
     ``cartesian_motion_controller/CartesianMotionController`` using
     the per-arm YAML under ``bringup/config/``, then atomically swaps
     ``joint_trajectory_controller`` for it.
  3. Assert the swap landed: ``cartesian_motion_controller`` is active
     and ``joint_trajectory_controller`` is inactive (same structural
     safety invariant as ADR-0007's effort-mode swap).
  4. Do NOT publish a Cartesian target. ``on_activate`` in the upstream
     plugin seeds ``m_target_frame = m_current_frame`` (motion-controller
     ``.cpp:91``) so the controller holds pose until a ``/target_frame``
     PoseStamped is published. Sample ``/joint_states`` over a fixed
     window and assert max per-joint deviation from the pose at the
     moment of the swap stays within the same 0.15 rad regulation
     tolerance already used by the crisp and simple_jimp tests — that
     shared threshold is what lets the M5 comparison harness flip
     between controllers (and arms) with one flag.

Teardown kills the cartesian bring-up process, then the sim launcher,
then the port/process reaper.

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
BRINGUP_LAUNCH = (
    REPO_ROOT / "bringup" / "launch" / "cartesian_bringup.launch.py"
)

EXPECTED_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CONTROLLER_NAME = "cartesian_motion_controller"
DISPLACED_CONTROLLER = "joint_trajectory_controller"

# Timeouts sized for MuJoCo cold-start + KDL model load in cartesian_
# controller_base's on_configure.
SIM_READY_TIMEOUT_S = 120
CONTROLLER_READY_TIMEOUT_S = 60
# Max |q - q_target| allowed (rad) during regulation hold. Matches the
# crisp and simple_jimp tests so M5 can compare apples-to-apples.
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
# Helpers (copied shape-for-shape from test_simple_jimp_regulation /
# test_crisp_joint_impedance so failure-mode behaviour stays identical
# across the three controller integration tests).
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
        stripped = raw.strip()
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


def _collect_joint_state_samples(duration_s: float) -> list[dict[str, float]]:
    """Subscribe to ``/joint_states`` via rclpy for ``duration_s`` seconds.

    Mirrors the implementation in ``test_simple_jimp_regulation`` and
    ``test_crisp_joint_impedance`` — a single persistent subscription
    avoids the per-sample ``ros2 topic echo`` startup cost that makes a
    strict sample-count assertion flaky.
    """
    import rclpy  # lazy import: ROS env is only guaranteed at test time
    from sensor_msgs.msg import JointState

    own_context = not rclpy.ok()
    if own_context:
        rclpy.init()
    node = rclpy.create_node("_integration_jstate_sampler")
    samples: list[dict[str, float]] = []

    def _cb(msg) -> None:  # sensor_msgs/JointState
        if msg.name and len(msg.name) == len(msg.position):
            samples.append(
                {n: float(p) for n, p in zip(msg.name, msg.position)}
            )

    node.create_subscription(JointState, "/joint_states", _cb, 10)
    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if own_context:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
    return samples


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
# Fixture: sim (position mode) + cartesian_motion_controller bring-up
# ---------------------------------------------------------------------------


@pytest.fixture(scope="function")
def cartesian_motion_up(request):
    robot: str = request.param
    subprocess.run([str(KILL_SIM)], check=False)

    log_dir = REPO_ROOT / ".auto_dev_logs"
    sim_log = log_dir / f"cartesian_sim_{robot}.log"
    bringup_log = log_dir / f"cartesian_bringup_{robot}.log"

    sim_proc = _spawn_in_pgid(
        ["bash", str(LAUNCH_SIM), robot, "position"], sim_log
    )
    bringup_proc: subprocess.Popen | None = None
    try:
        def sim_ready() -> bool:
            states = _controller_states()
            return (
                states.get("joint_state_broadcaster") == "active"
                and states.get(DISPLACED_CONTROLLER) == "active"
            )

        assert _wait_until(sim_ready, SIM_READY_TIMEOUT_S, interval=2.0), (
            f"Sim never reached the expected position-mode baseline for "
            f"{robot}. Last states={_controller_states()}. See {sim_log}."
        )

        bringup_argv = [
            "bash",
            "-c",
            (
                "source /opt/ros/humble/setup.bash && "
                f"source {INSTALL_SETUP} && "
                f"ros2 launch {BRINGUP_LAUNCH} "
                f"robot:={robot}"
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


@pytest.mark.parametrize("cartesian_motion_up", ["ur5e", "ur15"], indirect=True)
def test_cartesian_motion_regulation(cartesian_motion_up):
    robot, _sim_proc, _bringup_proc, sim_log, bringup_log = cartesian_motion_up

    # 1. Wait for the JTC → cartesian_motion_controller swap to land.
    def controller_active() -> bool:
        states = _controller_states()
        return (
            states.get(CONTROLLER_NAME) == "active"
            and states.get(DISPLACED_CONTROLLER) == "inactive"
        )

    assert _wait_until(controller_active, CONTROLLER_READY_TIMEOUT_S, interval=2.0), (
        f"{CONTROLLER_NAME} did not become the sole active position commander "
        f"for {robot}. States={_controller_states()}. "
        f"See {bringup_log} (bring-up) and {sim_log} (sim)."
    )

    # 2. Capture the hold pose — the plugin seeded its internal target to
    #    whatever m_current_frame was at on_activate, so |q - q0| must
    #    stay bounded without us publishing anything on /target_frame.
    q0 = _joint_state_positions()
    assert q0 is not None, f"No /joint_states message after activation ({robot})."
    for j in EXPECTED_JOINTS:
        assert j in q0, f"joint {j!r} missing from /joint_states: {q0}"

    # 3. Give the controller a moment to settle on its self-seeded target,
    #    then sample /joint_states over a fixed window and assert bounded
    #    error.
    time.sleep(1.5)
    observed = _collect_joint_state_samples(REGULATION_WINDOW_S)
    samples = len(observed)
    worst = 0.0
    for q in observed:
        for j in EXPECTED_JOINTS:
            if j in q:
                err = abs(q[j] - q0[j])
                if err > worst:
                    worst = err

    assert samples >= 3, (
        f"Only {samples} /joint_states samples in {REGULATION_WINDOW_S}s "
        f"for {robot}; /joint_states appears to have stalled. "
        f"See {sim_log}."
    )
    assert worst < REGULATION_TOLERANCE_RAD, (
        f"cartesian_motion_controller drifted {worst:.3f} rad from the "
        f"hold pose on {robot} (tolerance {REGULATION_TOLERANCE_RAD}). "
        f"See {sim_log} and {bringup_log}."
    )
