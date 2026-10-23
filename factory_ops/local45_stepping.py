"""SDK-free contract for one native physics step and a render-only refresh."""
from __future__ import annotations

import math
import time
from numbers import Integral, Real
from typing import Any


def physics_state(world: Any) -> tuple[int, float]:
    frame, stamp = world.current_time_step_index, world.current_time
    if (isinstance(frame, bool) or not isinstance(frame, Integral) or frame < 0
            or isinstance(stamp, bool) or not isinstance(stamp, Real)
            or not math.isfinite(stamp) or stamp < 0):
        raise RuntimeError("invalid native World physics counter/time")
    return int(frame), float(stamp)


class Local45Stepper:
    """Local cadence counts schedule renders; reported frames come from World.

    A failed step/render poisons this instance: callers must not continue a
    partially advanced experiment. No app.update or timeline clock fallback.
    """

    def __init__(self, world: Any, physics_dt_s: float, render_every_steps: int = 6):
        if (isinstance(physics_dt_s, bool) or not isinstance(physics_dt_s, Real)
                or not math.isfinite(physics_dt_s) or physics_dt_s <= 0
                or type(render_every_steps) is not int or render_every_steps not in (1, 6)):
            raise ValueError("positive physics dt and render factor 1 or 6 required")
        self.world = world
        self.dt = float(physics_dt_s)
        self.factor = render_every_steps
        self._last = physics_state(world)
        self._calls = 0
        self._failed = False
        self._wall_costs = {"physics_calls": 0, "physics_wall_s": 0.0,
                            "render_calls": 0, "render_wall_s": 0.0}

    def wall_costs(self) -> dict[str, Any]:
        """Inclusive native call costs; no timing or physics state is changed."""
        return dict(self._wall_costs)

    def _native_call(self, phase: str, callback: Any) -> None:
        started = time.perf_counter()
        try:
            callback()
        finally:
            self._wall_costs[phase + "_calls"] += 1
            self._wall_costs[phase + "_wall_s"] += time.perf_counter() - started

    def step(self) -> dict[str, Any]:
        if self._failed:
            raise RuntimeError("stepper is failed; a new experiment is required")
        try:
            before = physics_state(self.world)
            if before != self._last:
                raise RuntimeError("physics advanced outside the owned stepper")
            self._native_call("physics", lambda: self.world.step(render=False))
            after = physics_state(self.world)
            if (after[0] - before[0] != 1
                    or not math.isclose(after[1] - before[1], self.dt, rel_tol=1e-7, abs_tol=1e-9)):
                raise RuntimeError("native physics delta differs from one configured step"
                    f"; before={before!r}; after={after!r}"
                    f"; frame_delta={after[0]-before[0]!r}; time_delta_s={after[1]-before[1]!r}"
                    f"; configured_dt_s={self.dt!r}")
            self._calls += 1
            rendered = self._calls % self.factor == 0
            if rendered:
                self._native_call("render", self.world.render)
                if physics_state(self.world) != after:
                    raise RuntimeError("render-only refresh advanced native physics")
            self._last = after
            return {"physics_frame_before": before[0], "physics_frame": after[0],
                    "sim_time_before_s": before[1], "sim_time_s": after[1], "rendered": rendered}
        except BaseException:
            self._failed = True
            raise
