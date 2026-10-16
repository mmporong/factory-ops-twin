from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from math import inf, isfinite

from .model import Assignment, Robot, Task

TravelCost = Mapping[tuple[str, str], float] | Callable[[Robot, Task], float | None]
Feasibility = Callable[[Task], bool]


def _cost(costs: TravelCost, robot: Robot, task: Task) -> float:
    value = costs(robot, task) if callable(costs) else costs.get((robot.robot_id, task.pickup_station))
    if value is None:
        return inf
    value = float(value)
    return value if isfinite(value) and value >= 0 else inf


def _candidates(
    tasks: Sequence[Task], robots: Sequence[Robot], feasible: Feasibility | None
) -> tuple[list[Task], list[Robot]]:
    ready = [
        task
        for task in tasks
        if task.status.value == "ready" and (feasible is None or feasible(task))
    ]
    available = [robot for robot in robots if robot.available and robot.payload_id is None and robot.task_id is None]
    return ready, available


def choose_fifo(
    tasks: Sequence[Task],
    robots: Sequence[Robot],
    travel_cost: TravelCost,
    *,
    feasible: Feasibility | None = None,
) -> Assignment | None:
    """Choose the oldest feasible task, then its nearest feasible robot.

    An unreachable or caller-ineligible old task does not block later work.
    """
    ready, available = _candidates(tasks, robots, feasible)
    if not ready or not available:
        return None
    for task in sorted(ready, key=lambda item: (item.ready_time, item.order_id, item.stage.value, item.task_id)):
        ranked = sorted((_cost(travel_cost, robot, task), robot.robot_id, robot) for robot in available)
        cost, _, robot = ranked[0]
        if cost != inf:
            return Assignment("fifo", task.task_id, robot.robot_id, cost, task.attempt)
    return None


def choose_eta(
    tasks: Sequence[Task],
    robots: Sequence[Robot],
    travel_cost: TravelCost,
    *,
    speed: float,
    dwell: float,
    feasible: Feasibility | None = None,
) -> Assignment | None:
    """Choose the feasible task/robot pair with minimum travel ETA plus common dwell."""
    if not isfinite(speed) or speed <= 0 or not isfinite(dwell) or dwell < 0:
        raise ValueError("speed must be positive and dwell nonnegative")
    ready, available = _candidates(tasks, robots, feasible)
    ranked: list[tuple[float, float, str, str, str, str, Task, Robot]] = []
    for task in ready:
        for robot in available:
            distance = _cost(travel_cost, robot, task)
            if distance == inf:
                continue
            eta = distance / speed + dwell
            ranked.append(
                (eta, task.ready_time, task.order_id, task.stage.value, robot.robot_id, task.task_id, task, robot)
            )
    if not ranked:
        return None
    eta, _, _, _, _, _, task, robot = min(ranked)
    return Assignment("eta", task.task_id, robot.robot_id, eta, task.attempt)
