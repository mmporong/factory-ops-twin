# pyright: reportMissingImports=false
"""Two isolated Nav2 stacks sharing only /clock and factory evidence topics."""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    params = os.environ["FACTORY_OPS_NAV2_PARAMS"]
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
    for namespace in ("carter1", "carter2"):
        actions.extend(
            Node(
                package=package, executable=executable, namespace=namespace, name=name,
                output="screen", parameters=[params],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                arguments=["--ros-args", "--log-level", "info"],
            )
            for package, executable, name in specs
        )
        actions.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            namespace=namespace, name="lifecycle_manager_g2", output="screen",
            parameters=[{
                "use_sim_time": True, "autostart": True, "bond_timeout": 20.0,
                "node_names": [name for _, _, name in specs],
            }],
        ))
    return LaunchDescription(actions)
