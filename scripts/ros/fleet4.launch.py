# pyright: reportMissingImports=false
"""Launch isolated Nav2 stacks in the adapter's explicit fleet namespaces."""
from __future__ import annotations

import json
import os
from pathlib import Path
import sys
from typing import Mapping

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.ros.g2_mission import fleet_identity


def launch_namespaces(environ: Mapping[str, str] | None = None) -> tuple[str, ...]:
    values = os.environ if environ is None else environ
    declared = json.loads(values["FACTORY_OPS_FLEET_NAMESPACES"])
    if not isinstance(declared, list):
        raise ValueError("fleet launch namespaces must be a JSON list")
    namespaces, _ = fleet_identity(declared)
    return namespaces


def generate_launch_description():
    namespaces = launch_namespaces()
    params = os.environ["FACTORY_OPS_NAV2_PARAMS"]
    from launch import LaunchDescription
    from launch_ros.actions import Node, SetParameter

    specs = [
        ("nav2_map_server", "map_server", "map_server"),
        ("nav2_map_server", "map_server", "navigation_map_server"),
        ("nav2_amcl", "amcl", "amcl"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_smoother", "smoother_server", "smoother_server"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
    ]
    actions = [SetParameter("use_sim_time", True)]
    for namespace in namespaces:
        actions.extend(Node(
            package=package, executable=executable, namespace=namespace, name=name,
            output="screen", parameters=[params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            arguments=["--ros-args", "--log-level", "info"],
        ) for package, executable, name in specs)
        actions.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=namespace, name="lifecycle_manager_fleet4", output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 20.0,
                         "node_names": [name for _, _, name in specs]}],
        ))
    return LaunchDescription(actions)
