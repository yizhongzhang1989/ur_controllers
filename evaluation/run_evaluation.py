#!/usr/bin/env python3
"""Run an M5 evaluation scenario against a controller on the simulator.

This is the runner referenced by ROADMAP M5 bullet 2. It takes:

* a validated scenario YAML (schema v1, see
  ``evaluation/scenarios/README.md``),
* a controller name (one of the bring-ups already wired in M2–M4), and
* a robot (``ur5e`` | ``ur15``),

then:

1. Brings up the simulator (``scripts/launch_sim.sh <robot> <mode>``) and
   waits until its baseline controllers are active.
2. Brings up the chosen controller (``ros2 launch
   bringup/launch/<bringup>.launch.py robot:=<robot>``) and waits until
   the strict swap (ADR-0007) has landed.
3. Captures the initial ``/joint_states`` sample as ``q0``.
4. Generates a reference trajectory from the scenario via
   ``_reference.generate_joint_reference`` (joint scenarios) or holds a
   constant cartesian setpoint (cartesian regulation), and publishes it on
   the controller's target topic at a fixed rate.
5. Records ``/joint_states`` and the controller's ``~/tau_d`` (when
   available) to CSV for ``duration_s`` seconds.
6. Tears everything down and writes a manifest YAML next to the CSVs.

Output layout:

    evaluation/runs/<scenario>__<controller>__<robot>__<UTC-timestamp>/
        manifest.yaml        # what was run
        target.csv           # planned reference (t_s + per-joint commands)
        joint_states.csv     # observed /joint_states
        tau_d.csv            # observed ~/tau_d (joint-impedance controllers)

Metrics (M5 bullet 3) and the comparison report (M5 bullet 4) are out of
scope for this script — they consume these CSVs.

A ``--dry-run`` flag bypasses ROS entirely: the script loads the
scenario, generates the reference trajectory, writes ``target.csv`` and
``manifest.yaml``, and exits 0. Useful for CI without a sim and as the
unit-test surface for the trajectory + manifest plumbing.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO_ROOT / "evaluation" / "scenarios"
RUNS_DIR_DEFAULT = REPO_ROOT / "evaluation" / "runs"

# ---------------------------------------------------------------------------
# Lazy-loaded sibling modules. Loaded by file so this script works without
# evaluation/ being a python package on sys.path.
# ---------------------------------------------------------------------------


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_VALIDATE = _load_module("_evaluation_scenario_validate", SCENARIO_DIR / "validate.py")
_REFERENCE = _load_module("_evaluation_reference", REPO_ROOT / "evaluation" / "_reference.py")


# ---------------------------------------------------------------------------
# Controller registry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ControllerSpec:
    """How to bring up a controller and where to publish its targets."""

    name: str  # CLI key, e.g. ``crisp_joint_impedance``
    sim_mode: str  # ``effort`` | ``position`` (matches scripts/launch_sim.sh)
    bringup_launch: str  # path under bringup/launch/
    bringup_args: tuple[str, ...]  # extra ``arg:=value`` pairs (excluding robot)
    controller_node: str  # ros2_control controller name (for ~/tau_d topic)
    displaced_controller: str  # what gets deactivated by the strict swap
    target_topic: str  # joint-target topic (joint controllers)
    has_tau_d: bool  # publishes ``<controller_node>/tau_d``
    target_space: str  # ``joint`` | ``cartesian``
    config_stem: str  # bringup/config/<stem>.<robot>.yaml


_CONTROLLERS: dict[str, ControllerSpec] = {
    "crisp_joint_impedance": ControllerSpec(
        name="crisp_joint_impedance",
        sim_mode="effort",
        bringup_launch="bringup/launch/crisp_bringup.launch.py",
        bringup_args=("mode:=joint",),
        controller_node="joint_impedance_controller",
        displaced_controller="forward_effort_controller",
        target_topic="/joint_impedance_controller/target_joint",
        has_tau_d=True,
        target_space="joint",
        config_stem="crisp_joint_impedance",
    ),
    "crisp_cartesian_impedance": ControllerSpec(
        name="crisp_cartesian_impedance",
        sim_mode="effort",
        bringup_launch="bringup/launch/crisp_bringup.launch.py",
        bringup_args=("mode:=cartesian",),
        controller_node="cartesian_impedance_controller",
        displaced_controller="forward_effort_controller",
        target_topic="/cartesian_impedance_controller/target_pose",
        has_tau_d=True,
        target_space="cartesian",
        config_stem="crisp_cartesian_impedance",
    ),
    "crisp_gravity_compensation": ControllerSpec(
        name="crisp_gravity_compensation",
        sim_mode="effort",
        bringup_launch="bringup/launch/crisp_bringup.launch.py",
        bringup_args=("mode:=gravity",),
        controller_node="gravity_compensation",
        displaced_controller="forward_effort_controller",
        # Gravity-comp role takes no external target (regulation hold only).
        target_topic="",
        has_tau_d=True,
        target_space="joint",
        config_stem="crisp_gravity_compensation",
    ),
    "simple_joint_impedance": ControllerSpec(
        name="simple_joint_impedance",
        sim_mode="effort",
        bringup_launch="bringup/launch/simple_jimp_bringup.launch.py",
        bringup_args=(),
        controller_node="simple_joint_impedance_controller",
        displaced_controller="forward_effort_controller",
        target_topic="/simple_joint_impedance_controller/target_joint",
        has_tau_d=True,
        target_space="joint",
        config_stem="simple_joint_impedance",
    ),
    "cartesian_motion": ControllerSpec(
        name="cartesian_motion",
        sim_mode="position",
        bringup_launch="bringup/launch/cartesian_bringup.launch.py",
        bringup_args=(),
        controller_node="cartesian_motion_controller",
        displaced_controller="joint_trajectory_controller",
        target_topic="/cartesian_motion_controller/target_frame",
        has_tau_d=False,
        target_space="cartesian",
        config_stem="cartesian_motion",
    ),
}


def known_controllers() -> list[str]:
    return sorted(_CONTROLLERS)


def get_controller(name: str) -> ControllerSpec:
    if name not in _CONTROLLERS:
        raise KeyError(f"unknown controller {name!r}; known: {known_controllers()}")
    return _CONTROLLERS[name]


def check_compatibility(spec: ControllerSpec, scenario: dict) -> list[str]:
    """Return a list of error strings if the controller cannot run this
    scenario (empty list = compatible)."""
    errors: list[str] = []
    target_space = scenario.get("target", {}).get("space")
    stype = scenario.get("scenario_type")
    if target_space != spec.target_space:
        errors.append(
            f"controller {spec.name!r} expects target.space=={spec.target_space!r}"
            f", scenario has {target_space!r}"
        )
    if spec.target_space == "cartesian" and stype != "regulation":
        errors.append(
            f"cartesian controller {spec.name!r} only supports "
            f"scenario_type==regulation in schema v1, got {stype!r}"
        )
    if spec.name == "crisp_gravity_compensation" and stype != "regulation":
        errors.append(
            "crisp_gravity_compensation has no external target topic; only "
            "scenario_type==regulation (hold) is meaningful"
        )
    return errors


# ---------------------------------------------------------------------------
# Output layout + manifest
# ---------------------------------------------------------------------------


def make_run_dir(runs_root: Path, scenario_name: str, controller: str, robot: str) -> Path:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = runs_root / f"{scenario_name}__{controller}__{robot}__{ts}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def write_target_csv_joint(path: Path, samples, joints: list[str]) -> None:
    """Write a target CSV for joint scenarios."""
    with path.open("w", encoding="utf-8") as f:
        f.write("t_s," + ",".join(joints) + "\n")
        for s in samples:
            row = [f"{s.t_s:.6f}"] + [f"{p:.9f}" for p in s.position]
            f.write(",".join(row) + "\n")


def write_target_csv_cartesian(
    path: Path,
    duration_s: float,
    rate_hz: float,
    position_xyz_m: list[float],
    orientation_xyzw: list[float],
) -> None:
    """Write a target CSV for cartesian regulation: the same pose every row."""
    n = int(duration_s * rate_hz) + 1
    with path.open("w", encoding="utf-8") as f:
        f.write("t_s,px,py,pz,qx,qy,qz,qw\n")
        for i in range(n):
            t = i / rate_hz
            f.write(
                f"{t:.6f},"
                + ",".join(f"{x:.9f}" for x in position_xyz_m)
                + ","
                + ",".join(f"{x:.9f}" for x in orientation_xyzw)
                + "\n"
            )


def write_manifest(
    path: Path,
    *,
    scenario_path: Path,
    scenario: dict,
    controller: ControllerSpec,
    robot: str,
    rate_hz: float,
    initial: list[float] | None,
    artefacts: dict[str, str],
    started_at_utc: str,
    finished_at_utc: str,
    dry_run: bool,
) -> None:
    """Write the per-run manifest. Mirrors the shape of
    ``evaluation/baselines/crisp/*.manifest.yaml`` so existing tooling can
    grok it."""
    payload = {
        "schema_version": 1,
        "kind": "evaluation_run",
        "scenario": {
            "name": scenario.get("name"),
            "type": scenario.get("scenario_type"),
            "path": str(scenario_path.relative_to(REPO_ROOT))
            if scenario_path.is_absolute() and _is_relative_to(scenario_path, REPO_ROOT)
            else str(scenario_path),
            "duration_s": scenario.get("duration_s"),
            "metrics": scenario.get("metrics"),
            "pass_criteria": scenario.get("pass_criteria"),
        },
        "controller": {
            "name": controller.name,
            "controller_node": controller.controller_node,
            "sim_mode": controller.sim_mode,
            "bringup_launch": controller.bringup_launch,
            "bringup_args": list(controller.bringup_args),
            "config": f"bringup/config/{controller.config_stem}.{robot}.yaml",
            "target_topic": controller.target_topic,
            "displaced_controller": controller.displaced_controller,
        },
        "robot": robot,
        "publish_rate_hz": rate_hz,
        "initial_joint_positions": initial,
        "artefacts": artefacts,
        "started_at_utc": started_at_utc,
        "finished_at_utc": finished_at_utc,
        "runner": {
            "script": "evaluation/run_evaluation.py",
            "version": 1,
            "dry_run": dry_run,
        },
    }
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)


def _is_relative_to(child: Path, parent: Path) -> bool:
    try:
        child.relative_to(parent)
        return True
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# Live (ROS) helpers — only imported when not in dry-run.
# ---------------------------------------------------------------------------

CANONICAL_JOINTS = list(_VALIDATE.CANONICAL_JOINTS)
SIM_READY_TIMEOUT_S = 150
CONTROLLER_READY_TIMEOUT_S = 120


def _bash(cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    full = (
        "source /opt/ros/humble/setup.bash && "
        f"source {REPO_ROOT}/install/setup.bash && "
        f"{cmd}"
    )
    return subprocess.run(
        ["bash", "-c", full],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def _spawn_pg(argv: list[str], log_path: Path) -> subprocess.Popen:
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


def _controller_states() -> dict[str, str]:
    import re as _re

    cp = _bash("NO_COLOR=1 timeout 20 ros2 control list_controllers", timeout=30)
    states: dict[str, str] = {}
    if cp.returncode != 0:
        return states
    ansi = _re.compile(r"\x1b\[[0-9;]*m")
    for raw in cp.stdout.splitlines():
        line = ansi.sub("", raw).strip()
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


def _record_topics_to_csv(
    duration_s: float,
    js_csv: Path,
    tau_d_csv: Path | None,
    tau_d_topic: str | None,
) -> None:
    """Subscribe to /joint_states (and optionally tau_d) for ``duration_s``
    seconds and stream samples into CSV files, in canonical UR joint order.

    Live-only path; imports rclpy lazily.
    """
    import rclpy
    from sensor_msgs.msg import JointState

    own = not rclpy.ok()
    if own:
        rclpy.init()
    node = rclpy.create_node("_evaluation_recorder")

    js_f = js_csv.open("w", encoding="utf-8")
    js_f.write("t_s," + ",".join(CANONICAL_JOINTS) + "\n")
    tau_f = None
    if tau_d_csv is not None and tau_d_topic:
        tau_f = tau_d_csv.open("w", encoding="utf-8")
        tau_f.write("t_s," + ",".join(CANONICAL_JOINTS) + "\n")

    t_start = time.monotonic()

    def _row(msg, fh) -> None:
        if not msg.name:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in CANONICAL_JOINTS):
            return
        vec = msg.position if fh is js_f else msg.effort
        if not vec or len(vec) < len(msg.name):
            return
        t = time.monotonic() - t_start
        vals = [vec[idx[j]] for j in CANONICAL_JOINTS]
        fh.write(f"{t:.6f}," + ",".join(f"{v:.9f}" for v in vals) + "\n")

    node.create_subscription(JointState, "/joint_states", lambda m: _row(m, js_f), 50)
    if tau_f is not None:
        node.create_subscription(JointState, tau_d_topic, lambda m: _row(m, tau_f), 50)

    try:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        js_f.close()
        if tau_f is not None:
            tau_f.close()
        node.destroy_node()
        if own:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


def _read_initial_joint_positions(timeout_s: float = 10.0) -> list[float] | None:
    """One /joint_states sample, reordered to CANONICAL_JOINTS."""
    import rclpy
    from sensor_msgs.msg import JointState

    own = not rclpy.ok()
    if own:
        rclpy.init()
    node = rclpy.create_node("_evaluation_q0_reader")
    got: dict[str, float] = {}

    def _cb(msg) -> None:
        if not msg.name or len(msg.name) != len(msg.position):
            return
        for n, p in zip(msg.name, msg.position):
            got[n] = float(p)

    node.create_subscription(JointState, "/joint_states", _cb, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(j in got for j in CANONICAL_JOINTS):
                return [got[j] for j in CANONICAL_JOINTS]
    finally:
        node.destroy_node()
        if own:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass
    return None


def _publish_joint_reference_loop(
    target_topic: str, samples, log_path: Path, rate_hz: float
) -> subprocess.Popen:
    """Fork a python child that streams the pre-computed reference samples
    on ``target_topic`` at ``rate_hz``. Each sample is sent at its own
    ``t_s`` offset; once all samples are sent, the child latches the last
    one until killed.

    Done in a child process so the recorder's spin loop in the parent is
    not contended by another rclpy node sharing a context.
    """
    payload_path = log_path.with_suffix(".samples.csv")
    with payload_path.open("w", encoding="utf-8") as f:
        for s in samples:
            f.write(f"{s.t_s}," + ",".join(f"{p}" for p in s.position) + "\n")
    script = (
        "import rclpy, sys, time\n"
        "from sensor_msgs.msg import JointState\n"
        f"joints = {CANONICAL_JOINTS!r}\n"
        f"path = {str(payload_path)!r}\n"
        f"topic = {target_topic!r}\n"
        f"rate = float({rate_hz})\n"
        "samples = []\n"
        "with open(path) as fh:\n"
        "    for line in fh:\n"
        "        parts = line.strip().split(',')\n"
        "        if not parts or not parts[0]:\n"
        "            continue\n"
        "        t = float(parts[0])\n"
        "        pos = [float(x) for x in parts[1:]]\n"
        "        samples.append((t, pos))\n"
        "rclpy.init()\n"
        "node = rclpy.create_node('_evaluation_target_pub')\n"
        "pub = node.create_publisher(JointState, topic, 10)\n"
        "msg = JointState()\n"
        "msg.name = list(joints)\n"
        "t0 = time.monotonic()\n"
        "for t, pos in samples:\n"
        "    while time.monotonic() - t0 < t:\n"
        "        time.sleep(max(0.0, t - (time.monotonic() - t0)))\n"
        "    msg.position = pos\n"
        "    pub.publish(msg)\n"
        "# Latch the last sample at the publish rate until killed.\n"
        "period = 1.0 / rate\n"
        "while True:\n"
        "    pub.publish(msg)\n"
        "    time.sleep(period)\n"
    )
    argv = [
        "bash",
        "-c",
        (
            "source /opt/ros/humble/setup.bash && "
            f"source {REPO_ROOT}/install/setup.bash && "
            f"exec python3 -c {repr(script)}"
        ),
    ]
    return _spawn_pg(argv, log_path)


# ---------------------------------------------------------------------------
# Live orchestration
# ---------------------------------------------------------------------------


def _run_live(
    *,
    scenario: dict,
    scenario_path: Path,
    controller: ControllerSpec,
    robot: str,
    run_dir: Path,
    rate_hz: float,
) -> int:
    started_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    log_dir = REPO_ROOT / ".auto_dev_logs"
    log_dir.mkdir(exist_ok=True)
    sim_log = log_dir / f"eval_sim_{controller.name}_{robot}.log"
    bringup_log = log_dir / f"eval_bringup_{controller.name}_{robot}.log"
    target_log = log_dir / f"eval_target_{controller.name}_{robot}.log"

    subprocess.run([str(REPO_ROOT / "scripts" / "kill_sim.sh")], check=False)
    sim_proc = _spawn_pg(
        ["bash", str(REPO_ROOT / "scripts" / "launch_sim.sh"), robot, controller.sim_mode],
        sim_log,
    )
    bringup_proc: subprocess.Popen | None = None
    target_proc: subprocess.Popen | None = None
    try:
        if controller.sim_mode == "effort":
            sim_baseline = "forward_effort_controller"
        else:
            sim_baseline = "joint_trajectory_controller"

        def _sim_ready() -> bool:
            states = _controller_states()
            return (
                states.get("joint_state_broadcaster") == "active"
                and states.get(sim_baseline) == "active"
            )

        if not _wait_until(_sim_ready, SIM_READY_TIMEOUT_S, interval=2.0):
            raise RuntimeError(
                f"Sim never reached the expected baseline (joint_state_broadcaster"
                f" + {sim_baseline} active) for {robot}. See {sim_log}."
            )

        bringup_argv = [
            "bash",
            "-c",
            (
                "source /opt/ros/humble/setup.bash && "
                f"source {REPO_ROOT}/install/setup.bash && "
                f"ros2 launch {REPO_ROOT / controller.bringup_launch} "
                f"robot:={robot} " + " ".join(controller.bringup_args)
            ),
        ]
        bringup_proc = _spawn_pg(bringup_argv, bringup_log)

        def _ctrl_active() -> bool:
            states = _controller_states()
            return (
                states.get(controller.controller_node) == "active"
                and states.get(controller.displaced_controller) == "inactive"
            )

        if not _wait_until(_ctrl_active, CONTROLLER_READY_TIMEOUT_S, interval=2.0):
            raise RuntimeError(
                f"Controller {controller.controller_node!r} did not become "
                f"the sole active commander for {robot}. See {bringup_log}."
            )

        initial = _read_initial_joint_positions(timeout_s=15.0)
        if initial is None:
            raise RuntimeError(
                f"Could not read initial /joint_states for {robot}; sim may "
                f"have stalled. See {sim_log}."
            )

        if controller.target_space == "joint" and controller.target_topic:
            samples = _REFERENCE.generate_joint_reference(scenario, initial, rate_hz=rate_hz)
            write_target_csv_joint(run_dir / "target.csv", samples, CANONICAL_JOINTS)
            target_proc = _publish_joint_reference_loop(
                controller.target_topic, samples, target_log, rate_hz
            )
            time.sleep(1.0)
        elif controller.target_space == "cartesian":
            # cartesian regulation: write the constant target CSV; do not
            # publish (cartesian_motion_controller and crisp's cartesian
            # role both seed their target from the current pose at
            # on_activate, which is what the regulation-hold scenario
            # asks for).
            hold = scenario.get("command", {}).get("hold")
            if isinstance(hold, dict):
                pos = hold["position_xyz_m"]
                ori = hold["orientation_xyzw"]
            else:
                # ``hold: initial`` — the pose is whatever the controller
                # latched at activation. Record zeros as a placeholder.
                pos = [0.0, 0.0, 0.0]
                ori = [0.0, 0.0, 0.0, 1.0]
            write_target_csv_cartesian(
                run_dir / "target.csv",
                float(scenario["duration_s"]),
                rate_hz,
                pos,
                ori,
            )
        else:
            # gravity-comp: regulation only, no target topic.
            samples = _REFERENCE.generate_joint_reference(scenario, initial, rate_hz=rate_hz)
            write_target_csv_joint(run_dir / "target.csv", samples, CANONICAL_JOINTS)

        tau_d_csv = run_dir / "tau_d.csv" if controller.has_tau_d else None
        tau_d_topic = f"/{controller.controller_node}/tau_d" if controller.has_tau_d else None
        _record_topics_to_csv(
            duration_s=float(scenario["duration_s"]),
            js_csv=run_dir / "joint_states.csv",
            tau_d_csv=tau_d_csv,
            tau_d_topic=tau_d_topic,
        )

        finished_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        artefacts = {
            "target_csv": "target.csv",
            "joint_states_csv": "joint_states.csv",
        }
        if controller.has_tau_d:
            artefacts["tau_d_csv"] = "tau_d.csv"
        write_manifest(
            run_dir / "manifest.yaml",
            scenario_path=scenario_path,
            scenario=scenario,
            controller=controller,
            robot=robot,
            rate_hz=rate_hz,
            initial=initial,
            artefacts=artefacts,
            started_at_utc=started_at,
            finished_at_utc=finished_at,
            dry_run=False,
        )
        return 0
    finally:
        _kill_pg(target_proc)
        _kill_pg(bringup_proc)
        _kill_pg(sim_proc)
        subprocess.run([str(REPO_ROOT / "scripts" / "kill_sim.sh")], check=False)


# ---------------------------------------------------------------------------
# Dry-run orchestration
# ---------------------------------------------------------------------------


def _run_dry(
    *,
    scenario: dict,
    scenario_path: Path,
    controller: ControllerSpec,
    robot: str,
    run_dir: Path,
    rate_hz: float,
    initial: list[float],
) -> int:
    started_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    artefacts: dict[str, str] = {"target_csv": "target.csv"}
    if controller.target_space == "joint":
        # random_waypoints with bounds=='urdf' is not resolvable in dry-run
        # (no URDF queried). Reject up-front rather than silently emit a
        # broken target.csv.
        if scenario.get("scenario_type") == "random_waypoints":
            bounds = scenario.get("command", {}).get("bounds")
            if bounds == "urdf":
                raise RuntimeError(
                    "dry-run: random_waypoints with bounds='urdf' requires a "
                    "live URDF; provide explicit bounds.lower/upper or run "
                    "without --dry-run."
                )
        samples = _REFERENCE.generate_joint_reference(scenario, initial, rate_hz=rate_hz)
        write_target_csv_joint(run_dir / "target.csv", samples, CANONICAL_JOINTS)
    else:  # cartesian
        hold = scenario.get("command", {}).get("hold")
        if isinstance(hold, dict):
            pos = hold["position_xyz_m"]
            ori = hold["orientation_xyzw"]
        else:
            pos = [0.0, 0.0, 0.0]
            ori = [0.0, 0.0, 0.0, 1.0]
        write_target_csv_cartesian(
            run_dir / "target.csv",
            float(scenario["duration_s"]),
            rate_hz,
            pos,
            ori,
        )
    finished_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    write_manifest(
        run_dir / "manifest.yaml",
        scenario_path=scenario_path,
        scenario=scenario,
        controller=controller,
        robot=robot,
        rate_hz=rate_hz,
        initial=initial,
        artefacts=artefacts,
        started_at_utc=started_at,
        finished_at_utc=finished_at,
        dry_run=True,
    )
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _parse_initial(s: str | None) -> list[float] | None:
    if s is None:
        return None
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            f"--initial-joints: expected 6 comma-separated floats, got {len(parts)}"
        )
    return [float(p) for p in parts]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run an evaluation scenario against a controller (M5)."
    )
    parser.add_argument("--scenario", required=True, type=Path, help="Path to scenario YAML")
    parser.add_argument(
        "--controller",
        required=True,
        choices=known_controllers(),
        help="Controller bring-up to use",
    )
    parser.add_argument("--robot", required=True, choices=["ur5e", "ur15"])
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=RUNS_DIR_DEFAULT,
        help=f"Parent dir for run outputs (default: {RUNS_DIR_DEFAULT})",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=_REFERENCE.DEFAULT_RATE_HZ,
        help="Reference publish rate (default: %(default).0f Hz)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Skip ROS launch+record; just emit target.csv + manifest.yaml.",
    )
    parser.add_argument(
        "--initial-joints",
        type=str,
        default=None,
        help="Dry-run only: comma-separated 6-vector q0 (default: zeros).",
    )
    args = parser.parse_args(argv)

    errs = _VALIDATE.validate_file(args.scenario)
    if errs:
        print(f"{args.scenario}: INVALID scenario", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        return 2

    with args.scenario.open("r", encoding="utf-8") as f:
        scenario = yaml.safe_load(f)

    if args.robot not in scenario.get("robots", []):
        print(
            f"{args.scenario}: robot {args.robot!r} is not listed under "
            f"scenario.robots ({scenario.get('robots')})",
            file=sys.stderr,
        )
        return 2

    controller = get_controller(args.controller)
    compat_errs = check_compatibility(controller, scenario)
    if compat_errs:
        print(
            f"controller {args.controller!r} is not compatible with " f"{args.scenario}:",
            file=sys.stderr,
        )
        for e in compat_errs:
            print(f"  - {e}", file=sys.stderr)
        return 2

    run_dir = make_run_dir(args.out_dir, scenario["name"], args.controller, args.robot)

    if args.dry_run:
        initial = _parse_initial(args.initial_joints) or [0.0] * 6
        rc = _run_dry(
            scenario=scenario,
            scenario_path=args.scenario,
            controller=controller,
            robot=args.robot,
            run_dir=run_dir,
            rate_hz=float(args.rate_hz),
            initial=initial,
        )
        print(f"DRY-RUN OK: {run_dir}")
        return rc

    rc = _run_live(
        scenario=scenario,
        scenario_path=args.scenario,
        controller=controller,
        robot=args.robot,
        run_dir=run_dir,
        rate_hz=float(args.rate_hz),
    )
    if rc == 0:
        print(f"OK: {run_dir}")
    return rc


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
