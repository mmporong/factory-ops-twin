"""Process-local boundary between Nav2 observations and the logical ledger.

This module does not run ROS, move a robot, synthesize action results, or prove a
physical navigation run.  A caller must submit a real action result together
with a fresh localization pose in the ``map`` frame.  Only validated evidence
is allowed to commit pickup/drop transitions to :class:`Ledger`.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import json
import math
from typing import Any, Mapping
from uuid import UUID, uuid4

from .layout import EXPECTED_STATIONS
from .operations import ApplyResult, Command, EventId, Ledger, LedgerConfig, OrderState, Stage, Transition


SUCCEEDED = 4
CANCELED = 5
ABORTED = 6


class NavigationError(ValueError):
    """Raised when navigation evidence violates the mission contract."""


def _finite(value: float, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise NavigationError(f"{label} must be finite")
    return result


def _uuid(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise NavigationError(f"{label} must be a UUID string")
    try:
        UUID(value)
    except (ValueError, AttributeError) as exc:
        raise NavigationError(f"{label} must be a UUID string") from exc
    return value


def _angle_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True, slots=True)
class GoalPose:
    frame_id: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if self.frame_id != "map":
            raise NavigationError("goal frame must be map")
        _finite(self.x, "goal x")
        _finite(self.y, "goal y")
        _finite(self.yaw, "goal yaw")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class LocalizationPose:
    frame_id: str
    x: float
    y: float
    yaw: float
    stamp: float

    def __post_init__(self) -> None:
        if self.frame_id != "map":
            raise NavigationError("localization pose frame must be map")
        _finite(self.x, "pose x")
        _finite(self.y, "pose y")
        _finite(self.yaw, "pose yaw")
        if _finite(self.stamp, "pose stamp") < 0:
            raise NavigationError("pose stamp must be nonnegative")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class NavigationIntent:
    goal_id: str
    task_id: str
    order_id: str
    payload_id: str
    stage: Stage
    robot_id: str
    attempt: int
    station: str
    transition: Transition
    issued_sim_time: float
    goal: GoalPose
    deadline: float

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["stage"] = self.stage.value
        value["transition"] = self.transition.value
        return value


@dataclass(frozen=True, slots=True)
class NavigationResult:
    result_id: str
    goal_id: str
    task_id: str
    station: str
    attempt: int
    status: int
    received_sim_time: float
    pose: LocalizationPose

    def __post_init__(self) -> None:
        _uuid(self.result_id, "result_id")
        _uuid(self.goal_id, "goal_id")
        if not self.task_id or not self.station:
            raise NavigationError("result task_id and station are required")
        if type(self.attempt) is not int or self.attempt < 0:
            raise NavigationError("result attempt must be a nonnegative integer")
        if type(self.status) is not int or not 0 <= self.status <= 6:
            raise NavigationError("result status must be a ROS action status integer")
        if _finite(self.received_sim_time, "received_sim_time") <= 0:
            raise NavigationError("received_sim_time must be positive")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class NavigationReceipt:
    accepted: bool
    duplicate: bool
    result_id: str
    goal_id: str
    committed_transition: str | None
    ledger_seq: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class SingleOrderMission:
    """Validate one S1 -> CI -> CO -> OUT mission in logical simulation time.

    Dedupe state and the navigation journal are process-local.  The class is an
    adapter contract for a later ROS client; it is not evidence that Nav2 or a
    physical robot ran.
    """

    def __init__(
        self,
        run_id: str,
        *,
        robot_id: str = "carter1",
        order_id: str = "O1",
        payload_id: str = "P1",
        goal_timeout_s: float = 30.0,
        station_goals: Mapping[str, GoalPose] | None = None,
    ) -> None:
        timeout = _finite(goal_timeout_s, "goal_timeout_s")
        if timeout <= 0:
            raise NavigationError("goal_timeout_s must be positive")
        if not robot_id or not order_id or not payload_id:
            raise NavigationError("robot, order and payload IDs are required")
        default_goals = {
            station: GoalPose("map", xy[0], xy[1], 0.0)
            for station, xy in EXPECTED_STATIONS.items()
        }
        goals = dict(default_goals if station_goals is None else station_goals)
        required = {"S1", "CI", "CO", "OUT"}
        if not required.issubset(goals):
            raise NavigationError(f"station_goals must include {sorted(required)}")
        if any(not isinstance(goals[name], GoalPose) for name in required):
            raise NavigationError("station goals must be GoalPose values")

        self.run_id = run_id
        self.robot_id = robot_id
        self.order_id = order_id
        self.payload_id = payload_id
        self.goal_timeout_s = timeout
        self.station_goals = goals
        self.ledger = Ledger(run_id, LedgerConfig(service_time=20.0))
        self.ledger.add_robot(robot_id, "S1")
        self.ledger.add_order(order_id, payload_id, "S1", ready_time=0.0)
        self.sim_time = 0.0
        self.active_intent: NavigationIntent | None = None
        self.failed = False
        self.completed = False
        self.failure: dict[str, Any] | None = None
        self._producer_seq = 0
        self._journal_seq = 0
        self._journal: list[dict[str, Any]] = []
        self._result_fingerprints: dict[str, str] = {}
        self._receipts: dict[str, NavigationReceipt] = {}

    @property
    def navigation_journal(self) -> list[dict[str, Any]]:
        return deepcopy(self._journal)

    def start(self) -> NavigationIntent:
        self._guard_operable()
        if self.active_intent is not None or self.ledger.orders[self.order_id].state is not OrderState.SOURCE_WAITING:
            raise NavigationError("mission has already started")
        self._commit(Stage.INBOUND, Transition.ASSIGN, f"{self.order_id}:in", None)
        return self._issue(Stage.INBOUND, Transition.PICKUP, f"{self.order_id}:in", "S1")

    def advance(self, sim_time: float) -> NavigationIntent | None:
        """Advance logical simulation time; this never uses a CPU wall timer."""
        self._guard_operable()
        target = self._validate_monotonic_time(sim_time)
        if self.active_intent is not None:
            if target > self.active_intent.deadline:
                self.sim_time = target
                self._fail("navigation timeout", None)
                raise NavigationError("active navigation goal timed out")
            self.sim_time = target
            self._record("clock_advanced", {"target_sim_time": target})
            return None

        self.sim_time = target
        order = self.ledger.orders[self.order_id]
        if order.state is OrderState.PROCESSING:
            self.ledger.advance(target)
            if self.ledger.orders[self.order_id].state is OrderState.OUTPUT_WAITING:
                task_id = f"{self.order_id}:out"
                self._commit(Stage.OUTBOUND, Transition.ASSIGN, task_id, None)
                return self._issue(Stage.OUTBOUND, Transition.PICKUP, task_id, "CO")
        else:
            self._record("clock_advanced", {"target_sim_time": target})
        return None

    def submit_result(self, result: NavigationResult) -> NavigationReceipt:
        fingerprint = json.dumps(result.to_dict(), sort_keys=True, separators=(",", ":"))
        previous_fingerprint = self._result_fingerprints.get(result.result_id)
        if previous_fingerprint is not None:
            if previous_fingerprint != fingerprint:
                self._record("result_rejected", {"reason": "result_id reused with different result", "result": result.to_dict()})
                raise NavigationError("result_id reused with different result")
            prior = self._receipts.get(result.result_id)
            if prior is None:
                raise NavigationError("duplicate rejected result")
            return NavigationReceipt(
                prior.accepted, True, prior.result_id, prior.goal_id,
                prior.committed_transition, prior.ledger_seq,
            )

        self._guard_operable()
        self._result_fingerprints[result.result_id] = fingerprint
        intent = self.active_intent
        if intent is None:
            self._reject(result, "unsolicited result")
        assert intent is not None
        if result.goal_id != intent.goal_id:
            self._reject(result, "result does not match active goal_id")
        if (result.task_id, result.station, result.attempt) != (intent.task_id, intent.station, intent.attempt):
            self._reject(result, "result binding does not match active task/station/attempt")
        if result.status != SUCCEEDED:
            label = {CANCELED: "navigation canceled", ABORTED: "navigation aborted"}.get(
                result.status, "invalid terminal navigation status"
            )
            self._fail(label, result)
            raise NavigationError(label)
        if result.received_sim_time < self.sim_time:
            self._fail("result sim time is older than mission time", result)
            raise NavigationError("result sim time is older than mission time")
        if result.received_sim_time > intent.deadline:
            self.sim_time = result.received_sim_time
            self._fail("navigation timeout", result)
            raise NavigationError("navigation result arrived after deadline")
        pose_error = self._pose_error(intent, result)
        if pose_error is not None:
            self._fail(pose_error, result)
            raise NavigationError(pose_error)

        self.sim_time = result.received_sim_time
        applied = self._commit(intent.stage, intent.transition, intent.task_id, intent.station)
        self.active_intent = None
        if intent.transition is Transition.PICKUP:
            task = self.ledger.tasks[intent.task_id]
            self._issue(intent.stage, Transition.DROP, intent.task_id, task.drop_station)
        elif intent.stage is Stage.INBOUND:
            self.ledger.advance(self.sim_time)
        else:
            self._commit(Stage.OUTBOUND, Transition.ORDER_COMPLETE, intent.task_id, None)
            self.completed = True

        receipt = NavigationReceipt(
            True, False, result.result_id, result.goal_id, intent.transition.value, applied.ledger_seq
        )
        self._receipts[result.result_id] = receipt
        self._record("result_accepted", {"result": result.to_dict(), "receipt": receipt.to_dict()})
        return receipt

    def export(self) -> dict[str, Any]:
        """Return JSON-serializable logical evidence and deterministic replay status."""
        active = None if self.active_intent is None else self.active_intent.to_dict()
        return {
            "scope": "process-local logical mission; no ROS or physical navigation claim",
            "sim_time": self.sim_time,
            "failed": self.failed,
            "completed": self.completed,
            "failure": deepcopy(self.failure),
            "active_intent": active,
            "events": self.ledger.events,
            "snapshot": self.ledger.snapshot(),
            "navigation_journal": self.navigation_journal,
            "replay_matches": self.ledger.replay_matches(),
        }

    def fail(self, reason: str, *, sim_time: float) -> None:
        """Fail-stop an external transport/runtime error without a fabricated result."""
        self._guard_operable()
        if not isinstance(reason, str) or not reason.strip():
            raise NavigationError("failure reason is required")
        target = self._validate_monotonic_time(sim_time)
        self.sim_time = target
        self._fail(reason, None)

    def _validate_monotonic_time(self, sim_time: float) -> float:
        target = _finite(sim_time, "sim_time")
        if target < self.sim_time:
            raise NavigationError("sim_time must be monotonic")
        return target

    def _guard_operable(self) -> None:
        if self.failed:
            raise NavigationError("mission is fail-stopped")
        if self.completed:
            raise NavigationError("mission is complete")

    def _issue(self, stage: Stage, transition: Transition, task_id: str, station: str) -> NavigationIntent:
        if self.active_intent is not None:
            raise NavigationError("only one navigation goal may be active")
        task = self.ledger.tasks[task_id]
        intent = NavigationIntent(
            str(uuid4()), task_id, self.order_id, self.payload_id, stage,
            self.robot_id, task.attempt, station, transition, self.sim_time,
            self.station_goals[station], self.sim_time + self.goal_timeout_s,
        )
        self.active_intent = intent
        self._record("intent_issued", {"intent": intent.to_dict()})
        return intent

    def _commit(
        self, stage: Stage, transition: Transition, task_id: str, station: str | None
    ) -> ApplyResult:
        self._producer_seq += 1
        order = self.ledger.orders[self.order_id]
        return self.ledger.apply(
            Command(
                EventId(self.run_id, "single-order-mission", self._producer_seq),
                self.sim_time,
                self.order_id,
                self.payload_id,
                stage,
                transition,
                order.state_version,
                task_id=task_id if transition is not Transition.ORDER_COMPLETE else None,
                station_id=station,
                robot_id=self.robot_id if transition is not Transition.ORDER_COMPLETE else None,
                attempt=self.ledger.tasks[task_id].attempt,
            )
        )

    def _pose_error(self, intent: NavigationIntent, result: NavigationResult) -> str | None:
        pose = result.pose
        if pose.frame_id != "map":
            return "localization pose is not in map frame"
        if pose.stamp > result.received_sim_time:
            return "localization pose stamp is in the future"
        if result.received_sim_time - pose.stamp > 0.5:
            return "localization pose is stale"
        if math.hypot(pose.x - intent.goal.x, pose.y - intent.goal.y) > 0.25:
            return "localization pose is outside goal xy tolerance"
        if _angle_error(pose.yaw, intent.goal.yaw) > 0.25:
            return "localization pose is outside goal yaw tolerance"
        return None

    def _reject(self, result: NavigationResult, reason: str) -> None:
        self._record("result_rejected", {"reason": reason, "result": result.to_dict()})
        raise NavigationError(reason)

    def _fail(self, reason: str, result: NavigationResult | None) -> None:
        intent = self.active_intent
        evidence = {
            "reason": reason,
            "sim_time": self.sim_time,
            "intent": None if intent is None else intent.to_dict(),
            "result": None if result is None else result.to_dict(),
            "ledger_seq": self.ledger.ledger_seq,
        }
        self.failed = True
        self.failure = evidence
        self.active_intent = None
        self._record("mission_failed", evidence)

    def _record(self, kind: str, body: dict[str, Any]) -> None:
        self._journal_seq += 1
        self._journal.append({"journal_seq": self._journal_seq, "kind": kind, **deepcopy(body)})


__all__ = [
    "ABORTED",
    "CANCELED",
    "SUCCEEDED",
    "GoalPose",
    "LocalizationPose",
    "NavigationError",
    "NavigationIntent",
    "NavigationReceipt",
    "NavigationResult",
    "SingleOrderMission",
]
