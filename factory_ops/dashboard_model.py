"""CPU operating model for the web client, separate from Isaac/ROS evidence."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from functools import lru_cache
import math
from pathlib import Path
from typing import Any, cast

from .fleet_routing import RouteBook
from .operations import Command, EventId, Ledger, OrderState, Stage, Transition, choose_fifo
from .operations.inventory import InventoryLedger


class DashboardModel:
    """Reuse the operation ledger; route motion is illustrative CPU interpolation."""

    def __init__(self, config: Path) -> None:
        self.book = RouteBook.from_files(config / "layout.json", config / "g3-routing.json")
        self.ledger = Ledger("web-local-model")
        self.inventory = InventoryLedger({(s, "sensor-kit"): 20 for s in ("S1", "S2", "S3")},
                                         [f"T{i:03}" for i in range(1, 17)])
        self.sim_time = 0.0
        self.paused = False
        self.revision = 0
        self._seq = 0
        self._order_seq = 0
        self._trips: dict[str, dict[str, Any]] = {}
        self._poses: dict[str, dict[str, float]] = {}
        self._consumed: set[str] = set()
        self._events: deque[dict[str, Any]] = deque(maxlen=80)
        for i, station in enumerate(("S1", "S2", "S3", "OUT"), 1):
            key = f"amr-{i}"
            self.ledger.add_robot(key, station)
            x, y = self.book.plan(station, station).points[0]
            self._poses[key] = {"x_m": x, "y_m": y, "yaw_rad": 0.0}
        for source in ("S1", "S2", "S3", "S1"):
            self.create_order({"source": source})

    @lru_cache(maxsize=128)
    def _route(self, start: str, goal: str) -> tuple[tuple[float, float], ...]:
        return self.book.plan(start, goal).points

    def create_order(self, parameters: dict[str, Any]) -> None:
        source = parameters.get("source", ("S1", "S2", "S3")[self._order_seq % 3])
        quantity = parameters.get("quantity", 1)
        if source not in ("S1", "S2", "S3"):
            raise ValueError("자재 위치는 S1, S2, S3 중에서 선택하세요")
        if type(quantity) is not int or quantity <= 0:
            raise ValueError("수량은 양의 정수여야 합니다")
        inventory = cast(dict[str, Any], self.inventory.snapshot())
        tote = next((t for t in inventory["totes"] if t["state"] == "available"), None)
        if tote is None:
            raise ValueError("사용 가능한 빈 토트가 없습니다")
        next_id = self._order_seq + 1
        order = f"ORD-{next_id:04}"
        self.inventory.reserve(f"reserve:{order}", order, source, "sensor-kit", quantity, tote["tote_id"])
        self.ledger.add_order(order, tote["tote_id"], source, ready_time=self.sim_time)
        self._order_seq = next_id
        self._event("order_created", f"{order} 자재 공급 요청", None)

    def _event(self, kind: str, message: str, robot: str | None) -> None:
        self._events.append({"id": f"event-{self._seq}-{len(self._events)}-{self.sim_time}",
                             "sim_time_s": self.sim_time, "type": kind, "message": message, "robot_id": robot})

    def _apply(self, task: Any, robot_id: str, transition: Transition) -> None:
        self._seq += 1
        order = self.ledger.orders[task.order_id]
        station = task.pickup_station if transition is Transition.PICKUP else task.drop_station
        self.ledger.apply(Command(EventId(self.ledger.run_id, "web-model", self._seq), self.sim_time,
                                  task.order_id, task.payload_id, task.stage, transition, order.state_version,
                                  task_id=task.task_id, robot_id=robot_id,
                                  station_id=station if transition is not Transition.ASSIGN else None))

    def _trip(self, robot_id: str, task: Any, start: str, goal: str, phase: str) -> None:
        self._trips[robot_id] = {"task": task, "phase": phase, "goal": goal,
                                 "points": deque(self._route(start, goal)[1:])}

    def advance(self, dt: float) -> None:
        if not math.isfinite(dt) or not 0 <= dt <= 1:
            raise ValueError("model step must be between zero and one second")
        if self.paused or dt == 0:
            return
        self.sim_time += dt
        self.ledger.advance(self.sim_time)
        for order in self.ledger.orders.values():
            if order.state is OrderState.PROCESSING and order.order_id not in self._consumed:
                self.inventory.consume(f"consume:{order.order_id}", order.order_id)
                self._consumed.add(order.order_id)
        for robot_id, trip in list(self._trips.items()):
            pose = self._poses[robot_id]
            distance_left = 0.65 * dt  # Authored CPU display speed, not a measured AMR response.
            while trip["points"] and distance_left > 0:
                x, y = trip["points"][0]
                dx, dy = x - pose["x_m"], y - pose["y_m"]
                distance = math.hypot(dx, dy)
                if distance > 0:
                    pose["yaw_rad"] = math.atan2(dy, dx)
                if distance <= distance_left:
                    pose.update(x_m=x, y_m=y)
                    distance_left -= distance
                    trip["points"].popleft()
                else:
                    pose["x_m"] += dx / distance * distance_left
                    pose["y_m"] += dy / distance * distance_left
                    distance_left = 0
            if trip["points"]:
                continue
            task = trip["task"]
            if trip["phase"] == "pickup":
                if task.stage is Stage.INBOUND:
                    self.inventory.pickup(f"pickup:{task.order_id}", task.order_id, robot_id)
                self._apply(task, robot_id, Transition.PICKUP)
                self._trip(robot_id, task, task.pickup_station, task.drop_station, "drop")
                self._event("pickup", f"{robot_id} {task.payload_id} 적재 확인", robot_id)
            else:
                self._apply(task, robot_id, Transition.DROP)
                if task.stage is Stage.INBOUND:
                    self.inventory.deliver(f"deliver:{task.order_id}", task.order_id, "CI")
                else:
                    self._apply(task, robot_id, Transition.ORDER_COMPLETE)
                del self._trips[robot_id]
                self._event("drop", f"{robot_id} {task.drop_station} 인계 확인", robot_id)
        while True:
            assignment = choose_fifo(self.ledger.ready_tasks, list(self.ledger.robots.values()),
                                     lambda r, t: self.book.plan(r.location, t.pickup_station).distance_m,
                                     feasible=self.ledger.can_assign)
            if assignment is None:
                break
            task = self.ledger.tasks[assignment.task_id]
            robot = self.ledger.robots[assignment.robot_id]
            self._apply(task, robot.robot_id, Transition.ASSIGN)
            self._trip(robot.robot_id, task, robot.location, task.pickup_station, "pickup")
            self._event("assigned", f"{robot.robot_id} {task.order_id} 배정", robot.robot_id)

    def snapshot(self) -> dict[str, Any]:
        stock = cast(dict[str, Any], self.inventory.snapshot())
        robots = []
        for robot_id, robot in self.ledger.robots.items():
            trip = self._trips.get(robot_id)
            task = trip["task"] if trip else None
            robots.append({"id": robot_id, "label": f"AMR {robot_id[-1].zfill(2)}",
                           "status": "paused" if self.paused and trip else "moving" if trip else "idle",
                           "pose": dict(self._poses[robot_id]), "speed_mps": 0.65 if trip and not self.paused else 0,
                           "battery_pct": None, "battery_source": "unavailable",
                           "task_id": task.task_id if task else None,
                           "task_type": task.stage.value if task else None,
                           "task_label": ("자재 공급" if task.stage is Stage.INBOUND else "완제품 출하") if task else None,
                           "payload_id": robot.payload_id, "destination": trip["goal"] if trip else None,
                           "waiting_reason": "작업 일시정지" if self.paused and trip else None,
                           "route": [{"x_m": x, "y_m": y} for x, y in trip["points"]] if trip else [],
                           "observed_sim_time_s": self.sim_time})
        tasks = [{"id": t.task_id, "order_id": t.order_id, "type": t.stage.value, "state": t.status.value,
                  "robot_id": t.robot_id, "source": t.pickup_station, "destination": t.drop_station,
                  "payload_id": t.payload_id, "progress": 1 if t.status.value == "complete" else None}
                 for t in self.ledger.tasks.values()]
        return {"schema_version": 1, "run_id": self.ledger.run_id, "revision": self.revision,
                "mode": "local_model", "sim_time_s": self.sim_time,
                "observed_at": datetime.now(timezone.utc).isoformat(), "connection": "connected",
                "paused": self.paused, "capabilities": ["pause", "resume", "create_order"],
                "robots": robots, "tasks": tasks, "inventory": stock,
                "kpis": {"active_robots": len(self._trips), "total_robots": len(robots),
                         "completed_orders": sum(o.state is OrderState.COMPLETE for o in self.ledger.orders.values()),
                         "pending_tasks": sum(t.status.value != "complete" for t in self.ledger.tasks.values()),
                         "stock_units": sum(s["on_hand"] for s in stock["stock"])},
                "events": list(self._events), "commands": [],
                "model_scope": "CPU ledger and authored route interpolation; no physics, traffic clearance, battery or real MES"}
