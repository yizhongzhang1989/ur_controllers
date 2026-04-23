"""Bring up a crisp_controllers role on top of the already-running sim.

This launch file does NOT start the simulator — call
``scripts/launch_sim.sh <robot> effort`` first. It assumes the
controller_manager is up with ``forward_effort_controller`` active (the
default in MuJoCo effort mode) and takes over the effort command
interfaces by:

  1. Spawning the role's controller ``--inactive`` with
     ``--controller-type crisp_controllers/CartesianController`` and the
     per-arm param file under ``bringup/config/``.
  2. Once the spawner has exited cleanly, atomically switching
     ``forward_effort_controller`` → role via
     ``ros2 control switch_controllers --strict``. Strict mode means the
     whole bring-up fails if either side of the swap is rejected, so a
     broken config surfaces immediately rather than silently leaving the
     arm with two (or zero) effort commanders.

Args:
  robot:  ``ur5e`` | ``ur15``  — picks the per-arm YAML under
          ``bringup/config/crisp_joint_impedance.<robot>.yaml``.
  mode:   ``joint`` | ``cartesian`` | ``gravity`` — picks the crisp
          role (see ``docs/crisp_controllers.md``).
"""

from __future__ import annotations

import os
from pathlib import Path

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

# Supported (mode -> (controller_name, role_yaml_stem)) mapping.
_ROLE_YAML_STEM = {
    "joint": ("joint_impedance_controller", "crisp_joint_impedance"),
    "cartesian": ("cartesian_impedance_controller", "crisp_cartesian_impedance"),
    "gravity": ("gravity_compensation", "crisp_gravity_compensation"),
}


def launch_setup(context, *_args, **_kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    if mode not in _ROLE_YAML_STEM:
        raise RuntimeError(
            f"crisp_bringup: mode={mode!r} is not wired yet. "
            f"Supported: {sorted(_ROLE_YAML_STEM)}."
        )
    controller_name, stem = _ROLE_YAML_STEM[mode]
    config_path = CONFIG_DIR / f"{stem}.{robot}.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(
            f"crisp_bringup: missing config {config_path}. "
            f"Add it under bringup/config/ before using robot={robot} "
            f"mode={mode}."
        )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        name=f"{controller_name}_spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-type",
            "crisp_controllers/CartesianController",
            "--param-file",
            str(config_path),
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
            "forward_effort_controller",
            "--activate",
            controller_name,
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
            DeclareLaunchArgument(
                "mode",
                default_value="joint",
                choices=["joint", "cartesian", "gravity"],
                description="crisp controller role to bring up.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


# Makes ``python3 bringup/launch/crisp_bringup.launch.py`` a no-op; this
# file is only meaningful via ``ros2 launch``.
if __name__ == "__main__":  # pragma: no cover
    os.execvp("ros2", ["ros2", "launch", __file__])
