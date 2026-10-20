"""Authored via poses constrain planner requests, never substitute an admitted Path."""
from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

from .navigation import GoalPose


def guided_planner_scope(*, runtime_profile: str, development: bool, robot_count: int,
                         material_enabled: bool, sensor_backends: Sequence[str]) -> bool:
    return (runtime_profile == "local45_development" and development is True and robot_count == 4
            and material_enabled is True and set(sensor_backends) == {"physx_collision_planar"})


def authored_guides(chunk: Any, *, spacing_m: float = .5) -> tuple[GoalPose, ...]:
    """Densify the authored reference only, excluding its initial anchor."""
    if type(spacing_m) not in (int, float) or not math.isfinite(spacing_m) or not 0 < spacing_m <= .5:
        raise ValueError("authored guide spacing must be finite in (0, .5] metres")
    points = tuple(tuple(point) for point in chunk.points)
    if not points or any(len(point) != 2 or any(type(value) not in (int, float) or not math.isfinite(value)
                                               for value in point) for point in points):
        raise ValueError("authored reference points must be finite XY pairs")
    goal = chunk.goal_pose
    if (goal.frame_id != "map" or (goal.x, goal.y) != points[-1]):
        raise ValueError("authored guide endpoint must match the chunk goal")
    guides = []
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        if length == 0:
            continue
        steps = math.ceil(length/spacing_m)
        if len(guides)+steps > 2048:
            raise ValueError("authored guide request exceeds bounded pose count")
        yaw = math.atan2(end[1]-start[1], end[0]-start[0])
        for index in range(1, steps+1):
            xy = end if index == steps else tuple(start[i]+(end[i]-start[i])*index/steps for i in range(2))
            guides.append(GoalPose("map", xy[0], xy[1], yaw))
    if guides:
        guides[-1] = goal
    else:
        guides = [goal]
    return tuple(guides)


def guided_request(chunk: Any, start: Mapping[str, Any], stamp: Any, *,
                   goal_type: Any, pose_type: Any) -> Any:
    """Use the first observed estimated start, but derive all via poses from authored geometry."""
    if any(type(start.get(key)) not in (int, float) or not math.isfinite(start[key])
           for key in ("x_m", "y_m", "yaw_rad")):
        raise ValueError("planner start must be a finite estimated pose")

    def pose(goal: GoalPose) -> Any:
        message = pose_type()
        message.header.frame_id, message.header.stamp = "map", stamp
        message.pose.position.x, message.pose.position.y = goal.x, goal.y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(goal.yaw/2), math.cos(goal.yaw/2)
        return message

    request = goal_type()
    request.start = pose(GoalPose("map", start["x_m"], start["y_m"], start["yaw_rad"]))
    request.goals = [pose(goal) for goal in authored_guides(chunk)]
    request.planner_id, request.use_start = "GridBased", True
    return request
