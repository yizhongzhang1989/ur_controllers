"""Bring up our in-tree simple_joint_impedance_controller on top of the sim.

Mirrors ``bringup/launch/crisp_bringup.launch.py`` so the M5 comparison
harness can flip between controllers with a single flag (ROADMAP M3
last item). Does NOT start the simulator — call
``scripts/launch_sim.sh <robot> effort`` first. It assumes the
controller_manager is up with ``forward_effort_controller`` active (the
default in MuJoCo effort mode) and takes over the effort command
interfaces by:

  1. Spawning ``simple_joint_impedance_controller`` ``--inactive`` with
     ``--controller-type
     simple_joint_impedance_controller/SimpleJointImpedanceController``
     and the per-arm param file under ``bringup/config/``.
  2. Once the spawner has exited cleanly, atomically switching
     ``forward_effort_controller`` → our controller via
     ``ros2 control switch_controllers --strict`` so a broken config
     surfaces immediately rather than silently leaving the arm with two
     (or zero) effort commanders (ADR-0007).

Args:
  robot:  ``ur5e`` | ``ur15``  — picks the per-arm YAML under
          ``bringup/config/simple_joint_impedance.<robot>.yaml``.
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

CONTROLLER_NAME = "simple_joint_impedance_controller"
CONTROLLER_TYPE = "simple_joint_impedance_controller/SimpleJointImpedanceController"
CONFIG_STEM = "simple_joint_impedance"


def launch_setup(context, *_args, **_kwargs):
    robot = LaunchConfiguration("robot").perform(context)

    config_path = CONFIG_DIR / f"{CONFIG_STEM}.{robot}.yaml"
    if not config_path.is_file():
        raise FileNotFoundError(
            f"simple_jimp_bringup: missing config {config_path}. "
            f"Add it under bringup/config/ before using robot={robot}."
        )

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


# Makes ``python3 bringup/launch/simple_jimp_bringup.launch.py`` a no-op;
# this file is only meaningful via ``ros2 launch``.
if __name__ == "__main__":  # pragma: no cover
    os.execvp("ros2", ["ros2", "launch", __file__])
