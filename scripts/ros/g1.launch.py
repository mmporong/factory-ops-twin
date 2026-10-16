# pyright: reportMissingImports=false
"""One robot, explicit Nav2 nodes, no truth localization or hidden velocity source."""
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
    return LaunchDescription([
        SetParameter("use_sim_time", True),
        *[Node(package=package, executable=executable, name=name, output="screen",
               parameters=[params], arguments=["--ros-args", "--log-level", "info"])
          for package, executable, name in specs],
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_g1", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "bond_timeout": 20.0, "node_names": [name for _, _, name in specs]}]),
    ])
