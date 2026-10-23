"""Bounded, fair callback dispatch for node-owned ROS executors."""
from __future__ import annotations

import math
import time
from typing import Any, Callable, Iterable


class CallbackDrain:
    """Give every executor one nonblocking attempt per complete round.

    ``spin_once`` does not report whether it executed a callback, so all
    accounting deliberately describes dispatch attempts rather than callbacks.
    The wall budget is checked only after a complete fair round. A slow callback
    can therefore overrun the authored cooperative budget by one round.

    Optional priority executors own TF/clock and time-sensitive odometry. Their prefix consumes
    at most half the configured round/time allowance, followed by complete fair
    rounds; it does not add another drain budget or a background spin thread.
    """

    def __init__(
        self,
        executors: Iterable[Any],
        *,
        max_rounds: int,
        wall_budget_s: float,
        now_wall: Callable[[], float] = time.monotonic,
        priority_executors: Iterable[Any] = (),
    ) -> None:
        regular = tuple(executors)
        priority = tuple(priority_executors)
        owned = priority + regular
        if not owned:
            raise ValueError("callback drain requires at least one executor")
        if type(max_rounds) is not int or not 1 <= max_rounds <= 256:
            raise ValueError("callback drain max_rounds must be an integer from 1 through 256")
        if (isinstance(wall_budget_s, bool) or not isinstance(wall_budget_s, (int, float))
                or not math.isfinite(wall_budget_s) or not 0 < wall_budget_s <= 0.1):
            raise ValueError("callback drain wall_budget_s must be finite and in (0, 0.1]")
        if not callable(now_wall):
            raise ValueError("callback drain wall clock must be callable")
        if any(not callable(getattr(executor, "spin_once", None)) for executor in owned):
            raise ValueError("callback drain executors must provide spin_once")
        if len({id(executor) for executor in owned}) != len(owned):
            raise ValueError("callback drain cannot own the same executor twice")
        self._executors = owned
        self._priority = priority
        self.max_rounds = max_rounds
        self.wall_budget_s = float(wall_budget_s)
        self._now_wall = now_wall
        self._calls = 0
        self._rounds = 0
        self._spin_attempts = 0
        self._budget_reached_calls = 0
        self._priority_rounds = 0
        self._last_wall: float | None = None

    def _wall(self, previous: float | None = None) -> float:
        value = self._now_wall()
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError("callback drain wall clock must be finite")
        current = float(value)
        lower_bound = self._last_wall if previous is None else previous
        if lower_bound is not None and current < lower_bound:
            raise ValueError("callback drain wall clock moved backward")
        self._last_wall = current
        return current

    def drain(self) -> dict[str, Any]:
        start = self._wall()
        previous = start
        rounds = 0
        attempts = 0
        budget_reached = False
        elapsed = 0.0
        priority_rounds = 0
        # Spend at most half the EXISTING rounds/time on priority observation nodes.
        # Reserve a complete fair round for every IO executor, including when a
        # critical callback overruns its cooperative slice. Nothing is preempted.
        if self._priority:
            for _ in range(self.max_rounds // 2):
                for executor in self._priority:
                    executor.spin_once(timeout_sec=0.0)
                    attempts += 1
                rounds += 1
                priority_rounds += 1
                boundary = self._wall(previous)
                previous = boundary
                elapsed = boundary-start
                if elapsed >= self.wall_budget_s/2:
                    break
        for _ in range(self.max_rounds-rounds):
            for executor in self._executors:
                executor.spin_once(timeout_sec=0.0)
                attempts += 1
            rounds += 1
            boundary = self._wall(previous)
            previous = boundary
            elapsed = boundary - start
            if elapsed >= self.wall_budget_s:
                budget_reached = True
                break
        self._calls += 1
        self._rounds += rounds
        self._spin_attempts += attempts
        self._budget_reached_calls += int(budget_reached)
        self._priority_rounds += priority_rounds
        result = {
            "rounds": rounds,
            "spin_attempts": attempts,
            "wall_elapsed_s": elapsed,
            "wall_budget_reached": budget_reached,
        }
        if self._priority:
            result["priority_rounds"] = priority_rounds
        return result

    def snapshot(self) -> dict[str, Any]:
        result = {
            "executor_count": len(self._executors),
            "max_rounds_per_call": self.max_rounds,
            "wall_budget_s": self.wall_budget_s,
            "calls": self._calls,
            "rounds": self._rounds,
            "spin_attempts": self._spin_attempts,
            "wall_budget_reached_calls": self._budget_reached_calls,
            "spin_attempt_semantics": "nonblocking executor attempts; actual callback count is not observable",
            "budget_semantics": "cooperative boundary checked after each complete fair round",
        }
        if self._priority:
            result.update(priority_executor_count=len(self._priority), priority_rounds=self._priority_rounds,
                priority_semantics="TF/clock/odometry prefix uses at most half existing rounds/time; then complete fair rounds; total bounds unchanged")
        return result
