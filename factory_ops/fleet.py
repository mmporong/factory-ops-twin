"""Process-local multi-AMR mission contract for G3 fleet experiments.

The coordinator commits only navigation results supplied by an adapter.  It
does not run ROS, reserve traffic resources, synthesize action completion, or
claim that a physical/Isaac Sim run occurred.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from typing import Any, Literal
from uuid import uuid4

from .layout import EXPECTED_POCKETS, EXPECTED_STATIONS
from .navigation import (
    SUCCEEDED,
    GoalPose,
    NavigationError,
    NavigationIntent,
    NavigationReceipt,
    NavigationResult,
    validate_goal_pose,
)
from .operations import (
    ApplyResult,
    Command,
    EventId,
    Ledger,
    LedgerConfig,
    OrderState,
    Stage,
    Task,
    Transition,
    choose_eta,
    choose_fifo,
)
from .operations.dispatch import TravelCost


FleetPolicy = Literal["fifo", "eta"]
TaskFeasibility = Callable[[Task], bool]
SOURCE_STATIONS = frozenset({"S1", "S2", "S3"})
ROBOT_ANCHORS = frozenset(EXPECTED_STATIONS) | frozenset(EXPECTED_POCKETS)


def _finite(value: float, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise NavigationError(f"{label} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class FleetOrder:
    order_id: str
    payload_id: str
    source_station: str
    ready_time: float

    def __post_init__(self) -> None:
        if not self.order_id or not self.payload_id or not self.source_station:
            raise NavigationError("order, payload and source station are required")
        if self.source_station not in SOURCE_STATIONS:
            raise NavigationError("source station must be S1, S2, or S3")
        if _finite(self.ready_time, "ready_time") < 0:
            raise NavigationError("ready_time must be nonnegative")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class FleetMission:
    """Coordinate a declared non-empty AMR fleet in logical time."""

    def __init__(
        self,
        run_id: str,
        orders: Sequence[FleetOrder],
        *,
        robot_stations: Mapping[str, str],
        policy: FleetPolicy = "fifo",
        ledger_config: LedgerConfig | None = None,
        goal_timeout_s: float = 30.0,
        station_goals: Mapping[str, GoalPose] | None = None,
        eta_speed: float = 0.6,
        eta_dwell: float = 0.0,
        horizon: float | None = None,
    ) -> None:
        if not robot_stations or any(
            not isinstance(key, str) or not key.strip() or key != key.strip()
            or not isinstance(value, str) or not value.strip() or value != value.strip()
            for key, value in robot_stations.items()
        ):
            raise NavigationError("fleet requires named robots with starting stations")
        if len(set(robot_stations.values())) != len(robot_stations):
            raise NavigationError("robot starting stations must be distinct")
        if policy not in ("fifo", "eta"):
            raise NavigationError("policy must be fifo or eta")
        timeout = _finite(goal_timeout_s, "goal_timeout_s")
        speed = _finite(eta_speed, "eta_speed")
        dwell = _finite(eta_dwell, "eta_dwell")
        if timeout <= 0 or speed <= 0 or dwell < 0:
            raise NavigationError("timeout/speed must be positive and dwell nonnegative")
        limit = None if horizon is None else _finite(horizon, "horizon")
        if limit is not None and limit < 0:
            raise NavigationError("horizon must be nonnegative")
        declared = tuple(orders)
        if not declared or any(not isinstance(order, FleetOrder) for order in declared):
            raise NavigationError("at least one FleetOrder is required")
        if len({item.order_id for item in declared}) != len(declared):
            raise NavigationError("order IDs must be unique")
        if len({item.payload_id for item in declared}) != len(declared):
            raise NavigationError("payload IDs must be unique")
        default_goals = {
            station: GoalPose("map", xy[0], xy[1], 0.0)
            for station, xy in EXPECTED_STATIONS.items()
        }
        goals = dict(default_goals if station_goals is None else station_goals)
        anchors = ROBOT_ANCHORS | goals.keys()
        if any(station not in anchors for station in robot_stations.values()):
            raise NavigationError("robot starting station must be a known station or parking pocket")
        required_stations = {item.source_station for item in declared} | {"CI", "CO", "OUT"}
        if not required_stations.issubset(goals):
            raise NavigationError(f"station_goals must include {sorted(required_stations)}")
        if any(not isinstance(goal, GoalPose) for goal in goals.values()):
            raise NavigationError("station goals must be GoalPose values")

        self.run_id = run_id
        self.policy = policy
        self.goal_timeout_s = timeout
        self.station_goals = goals
        self.eta_speed = speed
        self.eta_dwell = dwell
        self.horizon = limit
        self.ledger = Ledger(run_id, ledger_config)
        for robot_id, station in sorted(robot_stations.items()):
            self.ledger.add_robot(robot_id, station)
        self.sim_time = 0.0
        self.active_intents: dict[str, NavigationIntent] = {}
        self.failed = False
        self.failure: dict[str, Any] | None = None
        self.completed = False
        self.censored = False
        self.censor: dict[str, Any] | None = None
        self._orders = tuple(sorted(declared, key=lambda item: (item.ready_time, item.order_id)))
        self._released_order_ids: set[str] = set()
        self._producer_seq = 0
        self._journal_seq = 0
        self._journal: list[dict[str, Any]] = []
        self._result_fingerprints: dict[str, str] = {}
        self._receipts: dict[str, NavigationReceipt] = {}
        self._committed_intents: dict[str, NavigationIntent] = {}
        self._release_due(0.0)

    @property
    def navigation_journal(self) -> list[dict[str, Any]]:
        return deepcopy(self._journal)

    @property
    def released_order_count(self) -> int:
        return len(self._released_order_ids)

    @property
    def expected_order_count(self) -> int:
        return len(self._orders)

    @property
    def declared_orders(self) -> tuple[FleetOrder, ...]:
        return self._orders

    @property
    def completed_order_count(self) -> int:
        return sum(order.state is OrderState.COMPLETE for order in self.ledger.orders.values())

    def advance(self, sim_time: float) -> list[ApplyResult]:
        """Advance the shared logical clock, release orders, and progress CELL1."""
        self._guard_operable()
        target = self._validate_time(sim_time)
        if self.horizon is not None and target > self.horizon:
            raise NavigationError("sim_time exceeds configured horizon")
        expired = sorted(
            (intent for intent in self.active_intents.values() if target > intent.deadline),
            key=lambda item: (item.deadline, item.robot_id),
        )
        if expired:
            self.sim_time = target
            self._fail("navigation timeout", expired[0].robot_id, None)
            raise NavigationError("active navigation goal timed out")

        results: list[ApplyResult] = []
        for order in self._orders:
            if order.order_id in self._released_order_ids or order.ready_time > target:
                continue
            results.extend(self.ledger.advance(order.ready_time))
            self.ledger.add_order(
                order.order_id,
                order.payload_id,
                order.source_station,
                ready_time=order.ready_time,
            )
            self._released_order_ids.add(order.order_id)
            self._record("order_released", {"order": order.to_dict()})
        results.extend(self.ledger.advance(target))
        self.sim_time = target
        self._record("clock_advanced", {"target_sim_time": target})
        self._update_completed()
        return results

    def dispatch(
        self,
        now: float,
        travel_cost: TravelCost,
        *,
        eligible_robot_ids: Sequence[str] | None = None,
        feasible: TaskFeasibility | None = None,
    ) -> list[NavigationIntent]:
        """Assign all currently possible robot/task pairs under one policy."""
        if eligible_robot_ids is not None and (
            isinstance(eligible_robot_ids, (str, bytes))
            or not isinstance(eligible_robot_ids, Sequence)
            or any(not isinstance(robot_id, str) or not robot_id for robot_id in eligible_robot_ids)
        ):
            raise NavigationError("eligible robot IDs must be a sequence of nonempty strings")
        eligible = set(self.ledger.robots) if eligible_robot_ids is None else set(eligible_robot_ids)
        unknown = eligible - self.ledger.robots.keys()
        if unknown:
            raise NavigationError(f"unknown eligible robot IDs: {sorted(unknown)}")
        if not callable(travel_cost) and not isinstance(travel_cost, Mapping):
            raise NavigationError("travel_cost must be a mapping or callable")
        if feasible is not None and not callable(feasible):
            raise NavigationError("feasible must be callable")
        self.advance(now)

        issued: list[NavigationIntent] = []
        while True:
            tasks = self.ledger.ready_tasks
            robots = [
                robot
                for robot_id, robot in sorted(self.ledger.robots.items())
                if robot_id in eligible and robot_id not in self.active_intents
            ]

            def shared_feasible(task: Task) -> bool:
                return self.ledger.can_assign(task) and (feasible is None or feasible(task))

            try:
                if self.policy == "fifo":
                    assignment = choose_fifo(tasks, robots, travel_cost, feasible=shared_feasible)
                else:
                    assignment = choose_eta(
                        tasks,
                        robots,
                        travel_cost,
                        speed=self.eta_speed,
                        dwell=self.eta_dwell,
                        feasible=shared_feasible,
                    )
            except Exception as exc:
                reason = f"dispatch selection callback failed: {type(exc).__name__}: {exc}"
                self._fail(reason, None, None)
                raise NavigationError("dispatch selection callback failed") from exc
            if assignment is None:
                break
            task = self.ledger.tasks[assignment.task_id]
            self._commit(task, Transition.ASSIGN, assignment.robot_id, None)
            intent = self._issue(task, assignment.robot_id, Transition.PICKUP, task.pickup_station)
            issued.append(intent)
            self._record("task_dispatched", {"assignment": asdict(assignment)})
        return issued

    def submit_result(
        self,
        robot_id: str,
        result: NavigationResult,
        *,
        error_code: int,
    ) -> NavigationReceipt:
        if type(error_code) is not int:
            raise NavigationError("error_code must be an integer")
        body = {"robot_id": robot_id, "error_code": error_code, "result": result.to_dict()}
        fingerprint = json.dumps(body, sort_keys=True, separators=(",", ":"))
        previous = self._result_fingerprints.get(result.result_id)
        if previous is not None:
            if previous != fingerprint:
                self._record("result_rejected", {"reason": "result_id reused with different result", **body})
                raise NavigationError("result_id reused with different result")
            receipt = self._receipts.get(result.result_id)
            if receipt is None:
                raise NavigationError("duplicate rejected result")
            return NavigationReceipt(
                receipt.accepted,
                True,
                receipt.result_id,
                receipt.goal_id,
                receipt.committed_transition,
                receipt.ledger_seq,
            )

        self._guard_operable()
        self._result_fingerprints[result.result_id] = fingerprint
        intent = self.active_intents.get(robot_id)
        if intent is None:
            self._reject(body, "foreign robot or robot has no active goal")
        assert intent is not None
        if result.goal_id != intent.goal_id:
            self._reject(body, "result does not match active goal_id")
        if (result.task_id, result.station, result.attempt) != (
            intent.task_id,
            intent.station,
            intent.attempt,
        ):
            self._reject(body, "result binding does not match active task/station/attempt")
        if result.received_sim_time < self.sim_time:
            self._fail("result sim time is older than mission time", robot_id, result)
            raise NavigationError("result sim time is older than mission time")
        if result.received_sim_time > intent.deadline:
            self.sim_time = result.received_sim_time
            self._fail("navigation timeout", robot_id, result)
            raise NavigationError("navigation result arrived after deadline")
        if result.status != SUCCEEDED or error_code != 0:
            self.sim_time = result.received_sim_time
            reason = "navigation action failed" if result.status != SUCCEEDED else "navigation transport error"
            self._fail(reason, robot_id, result, error_code=error_code)
            raise NavigationError(reason)
        pose_error = validate_goal_pose(intent, result)
        if pose_error is not None:
            self.sim_time = result.received_sim_time
            self._fail(pose_error, robot_id, result)
            raise NavigationError(pose_error)

        # A valid result is also a logical-clock observation.  Release due
        # orders, advance CELL1, and enforce every robot deadline/horizon before
        # committing the reported handoff.  Failed results never enter this
        # path, so they preserve the prior ledger state.
        self.advance(result.received_sim_time)
        task = self.ledger.tasks[intent.task_id]
        applied = self._commit(task, intent.transition, robot_id, intent.station)
        del self.active_intents[robot_id]
        if intent.transition is Transition.PICKUP:
            self._issue(task, robot_id, Transition.DROP, task.drop_station)
        elif intent.stage is Stage.INBOUND:
            self.ledger.advance(self.sim_time)
        else:
            self._commit(task, Transition.ORDER_COMPLETE, robot_id, None)
        self._update_completed()

        receipt = NavigationReceipt(
            True,
            False,
            result.result_id,
            result.goal_id,
            intent.transition.value,
            applied.ledger_seq,
        )
        self._receipts[result.result_id] = receipt
        self._committed_intents[result.result_id] = intent
        self._record("result_accepted", {**body, "receipt": receipt.to_dict()})
        return receipt

    def owns_handoff(self, intent: NavigationIntent, receipt: NavigationReceipt) -> bool:
        original = self._receipts.get(receipt.result_id)
        return (original is not None and original.accepted
                and self._committed_intents.get(receipt.result_id) == intent
                and original.to_dict() == {**receipt.to_dict(), "duplicate": False})

    def fail(self, reason: str, *, sim_time: float, robot_id: str | None = None) -> None:
        """Fail-stop an external runtime error without inventing a result."""
        self._guard_operable()
        if not isinstance(reason, str) or not reason.strip():
            raise NavigationError("failure reason is required")
        target = self._validate_time(sim_time)
        self.sim_time = target
        self._fail(reason, robot_id, None)

    def finish_incomplete(
        self,
        *,
        horizon: float | None = None,
        reason: str = "evaluation horizon reached",
    ) -> None:
        """Censor an unfinished run without converting missing work to success."""
        self._guard_operable()
        target = self.horizon if horizon is None else _finite(horizon, "horizon")
        if target is None:
            raise NavigationError("finish_incomplete requires a horizon")
        if not reason.strip():
            raise NavigationError("censor reason is required")
        self.advance(target)
        if self.completed:
            raise NavigationError("completed mission cannot be censored")
        self.censored = True
        self.censor = {
            "reason": reason,
            "horizon": target,
            "completed_order_count": self.completed_order_count,
            "expected_order_count": self.expected_order_count,
        }
        self._record("mission_censored", deepcopy(self.censor))

    def export(self) -> dict[str, Any]:
        return {
            "scope": "process-local synthetic adapter contract; no ROS, Isaac Sim, or physical result claim",
            "policy": self.policy,
            "sim_time": self.sim_time,
            "failed": self.failed,
            "completed": self.completed,
            "censored": self.censored,
            "failure": deepcopy(self.failure),
            "censor": deepcopy(self.censor),
            "released_order_count": self.released_order_count,
            "expected_order_count": self.expected_order_count,
            "completed_order_count": self.completed_order_count,
            "active_intents": {
                key: value.to_dict() for key, value in sorted(self.active_intents.items())
            },
            "events": self.ledger.events,
            "snapshot": self.ledger.snapshot(),
            "navigation_journal": self.navigation_journal,
            "replay_matches": self.ledger.replay_matches(),
        }

    def _release_due(self, target: float) -> None:
        for order in self._orders:
            if order.order_id in self._released_order_ids or order.ready_time > target:
                continue
            self.ledger.add_order(
                order.order_id,
                order.payload_id,
                order.source_station,
                ready_time=order.ready_time,
            )
            self._released_order_ids.add(order.order_id)
            self._record("order_released", {"order": order.to_dict()})

    def _commit(
        self,
        task: Task,
        transition: Transition,
        robot_id: str,
        station: str | None,
    ) -> ApplyResult:
        self._producer_seq += 1
        order = self.ledger.orders[task.order_id]
        complete = transition is Transition.ORDER_COMPLETE
        return self.ledger.apply(
            Command(
                EventId(self.run_id, "fleet-mission", self._producer_seq),
                self.sim_time,
                task.order_id,
                task.payload_id,
                task.stage,
                transition,
                order.state_version,
                task_id=None if complete else task.task_id,
                station_id=station,
                robot_id=None if complete else robot_id,
                attempt=task.attempt,
            )
        )

    def _issue(
        self,
        task: Task,
        robot_id: str,
        transition: Transition,
        station: str,
    ) -> NavigationIntent:
        if robot_id in self.active_intents:
            raise NavigationError("robot already has an active navigation goal")
        intent = NavigationIntent(
            str(uuid4()),
            task.task_id,
            task.order_id,
            task.payload_id,
            task.stage,
            robot_id,
            task.attempt,
            station,
            transition,
            self.sim_time,
            self.station_goals[station],
            self.sim_time + self.goal_timeout_s,
        )
        self.active_intents[robot_id] = intent
        self._record("intent_issued", {"intent": intent.to_dict()})
        return intent

    def _validate_time(self, sim_time: float) -> float:
        target = _finite(sim_time, "sim_time")
        if target < self.sim_time:
            raise NavigationError("sim_time must be monotonic")
        return target

    def _guard_operable(self) -> None:
        if self.failed:
            raise NavigationError("mission is fail-stopped")
        if self.completed:
            raise NavigationError("mission is complete")
        if self.censored:
            raise NavigationError("mission is censored")

    def _update_completed(self) -> None:
        self.completed = (
            self.released_order_count == self.expected_order_count
            and self.completed_order_count == self.expected_order_count
        )

    def _reject(self, body: dict[str, Any], reason: str) -> None:
        self._record("result_rejected", {"reason": reason, **body})
        raise NavigationError(reason)

    def _fail(
        self,
        reason: str,
        robot_id: str | None,
        result: NavigationResult | None,
        *,
        error_code: int | None = None,
    ) -> None:
        active_intents = {
            key: value.to_dict() for key, value in sorted(self.active_intents.items())
        }
        evidence = {
            "reason": reason,
            "sim_time": self.sim_time,
            "robot_id": robot_id,
            "intent": None
            if robot_id is None or robot_id not in self.active_intents
            else self.active_intents[robot_id].to_dict(),
            "result": None if result is None else result.to_dict(),
            "error_code": error_code,
            "ledger_seq": self.ledger.ledger_seq,
            "active_intents": active_intents,
        }
        self.failed = True
        self.failure = evidence
        self.active_intents.clear()
        self._record("mission_failed", evidence)

    def _record(self, kind: str, body: dict[str, Any]) -> None:
        self._journal_seq += 1
        self._journal.append({"journal_seq": self._journal_seq, "kind": kind, **deepcopy(body)})


__all__ = ["FleetMission", "FleetOrder", "FleetPolicy", "TaskFeasibility"]
