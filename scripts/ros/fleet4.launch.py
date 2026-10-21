# pyright: reportMissingImports=false
"""Launch isolated Nav2 stacks in the adapter's explicit fleet namespaces."""
from __future__ import annotations

import json
import math
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


def startup_stagger_s(environ: Mapping[str, str] | None = None) -> float:
    """Opt-in wall-time spacing, not a lifecycle retry or readiness override."""
    values = os.environ if environ is None else environ
    raw = values.get("FACTORY_OPS_NAV2_STARTUP_STAGGER_S", "0")
    try:
        seconds = float(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("Nav2 startup stagger must be finite seconds in [0, 5]") from error
    if not isinstance(raw, str) or not math.isfinite(seconds) or not 0. <= seconds <= 5.:
        raise ValueError("Nav2 startup stagger must be finite seconds in [0, 5]")
    return seconds


def generate_launch_description():
    namespaces = launch_namespaces()
    stagger = startup_stagger_s()
    if stagger*(len(namespaces)-1) > 15.:
        raise ValueError("Nav2 final namespace start delay must not exceed 15 wall seconds")
    params = os.environ["FACTORY_OPS_NAV2_PARAMS"]
    from launch import LaunchDescription
    from launch_ros.actions import Node, SetParameter
    if stagger:
        from launch.actions import TimerAction

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
    for index, namespace in enumerate(namespaces):
        stack = [Node(
            package=package, executable=executable, namespace=namespace, name=name,
            output="screen", parameters=[params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            arguments=["--ros-args", "--log-level", "info"],
        ) for package, executable, name in specs]
        stack.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=namespace, name="lifecycle_manager_fleet4", output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 20.0,
                         "node_names": [name for _, _, name in specs]}],
        ))
        if stagger and index:
            # TimerAction is a one-shot asyncio/wall timer and cancels on shutdown.
            # No ROSTimer, service timeout change or bypass of G3's all-active gate.
            actions.append(TimerAction(period=float(index*stagger), actions=stack, cancel_on_shutdown=True))
        else:
            actions.extend(stack)
    return LaunchDescription(actions)
