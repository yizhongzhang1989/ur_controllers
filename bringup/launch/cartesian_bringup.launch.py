"""Bring up ``cartesian_motion_controller`` on top of the already-running
sim (position mode).

This launch file does NOT start the simulator — call
``scripts/launch_sim.sh <robot> position`` first. Unlike the effort-mode
crisp / simple_jimp bring-ups, the cartesian motion controller commands
joint **position** interfaces, so it must coexist with / displace the
sim's ``joint_trajectory_controller`` rather than
``forward_effort_controller``. See ``docs/cartesian_controllers.md``
"Bring-up implications" and ADR-0007 (same swap pattern, different
controller pair).

Steps:

  1. Query ``/robot_state_publisher`` for its ``robot_description``
     parameter (published with transient-local QoS by the sim's RSP
     node) and serialise it into a temp YAML of the form
     ``cartesian_motion_controller.ros__parameters.robot_description``.
     This is necessary because our sim hands ``robot_description`` to
     the controller manager via the ``/robot_description`` topic rather
     than as a CM-level launch parameter (see
     ``third_party/ur_simulator/src/ur_sim_config/launch/
     ur_sim_mujoco.launch.py`` line ~129), and on Humble the CM only
     auto-propagates ``robot_description`` to newly-loaded controllers
     when it was itself given the param at CM-node construction time —
     the topic path keeps the CM-local copy but does not push it down
     to individual controllers. ``CartesianControllerBase::on_configure``
     reads the string from the controller's own node param
     (``cartesian_controller_base.cpp:137`` on Humble), so we set it
     there directly. The crisp controllers dodge this by querying
     ``/controller_manager`` themselves (``cartesian_controller.cpp:242``)
     — not an option for us since we must not patch the read-only
     ``cartesian_controllers`` submodule (ADR-0003).
  2. Spawn ``cartesian_motion_controller`` ``--inactive`` with
     ``--controller-type
     cartesian_motion_controller/CartesianMotionController`` and both
     param files: the per-arm YAML under ``bringup/config/`` plus the
     temp ``robot_description`` YAML from step 1. Multiple
     ``--param-file`` flags are additive on the controller_manager
     spawner.
  2. Once the spawner has exited cleanly, atomically swap
     ``joint_trajectory_controller`` → ``cartesian_motion_controller``
     via ``ros2 control switch_controllers --strict``. Strict mode means
     the whole bring-up fails if either side of the swap is rejected so
     a broken config surfaces immediately rather than silently leaving
     the arm with two (or zero) position commanders.

Args:
  robot:  ``ur5e`` | ``ur15``  — picks the per-arm YAML under
          ``bringup/config/cartesian_motion.<robot>.yaml``.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "bringup" / "config"

CONTROLLER_NAME = "cartesian_motion_controller"
CONTROLLER_TYPE = "cartesian_motion_controller/CartesianMotionController"
CONFIG_STEM = "cartesian_motion"
# In position-mode sim bring-up, joint_trajectory_controller owns the
# position command interfaces; see the controller-activation block in
# third_party/ur_simulator/src/ur_sim_config/launch/ur_sim_mujoco.launch.py.
DISPLACED_CONTROLLER = "joint_trajectory_controller"


def _fetch_robot_description() -> str:
    """Return the ``robot_description`` string currently published by
    ``/robot_state_publisher``.

    Opens a short-lived rclpy context, calls ``get_parameters`` on the
    RSP node, and returns the value. Raises ``RuntimeError`` on any
    failure (service not reachable, empty value, etc.) so the launch
    fails loudly rather than spawning a controller that would then
    refuse to configure with "robot_description is empty".
    """
    import rclpy  # lazy: requires the ROS env to be sourced at launch time
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter

    own_context = not rclpy.ok()
    if own_context:
        rclpy.init()
    node = rclpy.create_node("_cartesian_bringup_rd_fetcher")
    try:
        client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(
                "cartesian_bringup: /robot_state_publisher not reachable; "
                "is the sim running (scripts/launch_sim.sh <robot> position)?"
            )
        req = GetParameters.Request()
        req.names = ["robot_description"]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(
                "cartesian_bringup: timed out fetching robot_description "
                "from /robot_state_publisher."
            )
        values = future.result().values
        if not values or values[0].type != Parameter.Type.STRING.value:
            raise RuntimeError(
                "cartesian_bringup: robot_description is missing or not a "
                "string on /robot_state_publisher."
            )
        xml = values[0].string_value
        if not xml:
            raise RuntimeError(
                "cartesian_bringup: /robot_state_publisher returned an " "empty robot_description."
            )
        return xml
    finally:
        node.destroy_node()
        if own_context:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


def _write_robot_description_param_file(robot_description: str) -> Path:
    """Write a ``{CONTROLLER_NAME}.ros__parameters.robot_description``
    YAML to a named temp file and return its path.

    A named file (not ``NamedTemporaryFile(delete=True)``) is used so
    the spawner can open it by path after this function returns. The
    file is left on disk — temp-file cleanup on reboot is good enough
    and avoids racy delete-on-exit semantics at launch teardown.
    """
    payload = {
        CONTROLLER_NAME: {
            "ros__parameters": {
                "robot_description": robot_description,
            }
        }
    }
    fd, path = tempfile.mkstemp(prefix="cartesian_bringup_rd_", suffix=".yaml")
    with os.fdopen(fd, "w") as fh:
        yaml.safe_dump(payload, fh)
    return Path(path)


def launch_setup(context, *_args, **_kwargs):
    robot = LaunchConfiguration("robot").perform(context)

    config_path = CONFIG_DIR / f"{CONFIG_STEM}.{robot}.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(
            f"cartesian_bringup: missing config {config_path}. "
            f"Add it under bringup/config/ before using robot={robot}."
        )

    robot_description = _fetch_robot_description()
    rd_param_file = _write_robot_description_param_file(robot_description)

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        name=f"{CONTROLLER_NAME}_spawner",
        arguments=[
            CONTROLLER_NAME,
            "--controller-manager",
            "/controller_manager",
            "--controller-type",
            CONTROLLER_TYPE,
            "--param-file",
            str(config_path),
            "--param-file",
            str(rd_param_file),
            "--inactive",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    switch = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--strict",
            "--deactivate",
            DISPLACED_CONTROLLER,
            "--activate",
            CONTROLLER_NAME,
        ],
        output="screen",
    )

    delay_switch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner,
            on_exit=[switch],
        )
    )

    return [spawner, delay_switch]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="ur5e",
                choices=["ur5e", "ur15"],
                description="UR arm variant. Must match the running sim.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


# Makes ``python3 bringup/launch/cartesian_bringup.launch.py`` a no-op;
# this file is only meaningful via ``ros2 launch``.
if __name__ == "__main__":  # pragma: no cover
    os.execvp("ros2", ["ros2", "launch", __file__])
