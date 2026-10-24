"""CPU operating model for the web client, separate from Isaac/ROS evidence."""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from functools import lru_cache
import math
from pathlib import Path
from typing import Any, Mapping, cast
from uuid import uuid4

from .dashboard_charging import DashboardCharging, SCOPE as CHARGING_SCOPE, move_path
from .fleet_routing import RouteBook
from .operations import Command, EventId, Ledger, OrderState, Stage, Transition, choose_fifo
from .operations.inventory import InventoryLedger


class DashboardModel:
    """Reuse the operation ledger; route motion is illustrative CPU interpolation."""

    def __init__(
        self,
        config: Path,
        *,
        charging_profile: Mapping[str, Any] | None = None,
        initial_orders: int = 4,
        run_id: str | None = None,
        robot_stations: tuple[str, ...] = ("S1", "S2", "S3", "OUT"),
    ) -> None:
        if type(initial_orders) is not int or initial_orders < 0:
            raise ValueError("initial_orders must be a nonnegative integer")
        if run_id is not None and (not isinstance(run_id, str) or not run_id.strip()):
            raise ValueError("run_id must be a non-empty string or null")
        self.book = RouteBook.from_files(config / "layout.json", config / "g3-routing.json")
        if (not isinstance(robot_stations, tuple) or not robot_stations
                or any(not isinstance(s, str) for s in robot_stations)
                or len(set(robot_stations)) != len(robot_stations)
                or any(s not in self.book.nodes for s in robot_stations)):
            raise ValueError("nonempty distinct authored robot stations are required")
        self.ledger = Ledger(run_id.strip() if run_id is not None else f"web-local-model-{uuid4()}")
        tote_ids = [f"T{i:03}" for i in range(1, 17)]
        self.inventory = InventoryLedger({(s, "sensor-kit"): 20 for s in ("S1", "S2", "S3")},
                                         tote_ids, tote_locations={
                                             key: f"S{i % 3 + 1}" for i, key in enumerate(tote_ids)})
        self.sim_time = 0.0
        self.paused = False
        self.revision = 0
        self._seq = 0
        self._event_seq = 0
        self._equipment_seq = 0
        self._order_seq = 0
        self._trips: dict[str, dict[str, Any]] = {}
        self._return_tasks: dict[str, dict[str, Any]] = {}
        # Adapter locations include auxiliary tote trips; the production ledger
        # records only its own order handovers and remains replayable.
        self._locations: dict[str, str] = {}
        self._poses: dict[str, dict[str, float]] = {}
        self._consumed: set[str] = set()
        self._events: deque[dict[str, Any]] = deque(maxlen=80)
        for i, station in enumerate(robot_stations, 1):
            key = f"amr-{i}"
            self.ledger.add_robot(key, station)
            self._locations[key] = station
            x, y = self.book.plan(station, station).points[0]
            self._poses[key] = {"x_m": x, "y_m": y, "yaw_rad": 0.0}
        sources = ("S1", "S2", "S3")
        for index in range(initial_orders):
            self.create_order({"source": sources[index % len(sources)]})
        self.charging = DashboardCharging(self, charging_profile) if charging_profile is not None else None

    @lru_cache(maxsize=128)
    def _route(self, start: str, goal: str) -> tuple[tuple[float, float], ...]:
        return self.book.plan(start, goal).points

    def create_order(self, parameters: dict[str, Any]) -> str:
        source = parameters.get("source", ("S1", "S2", "S3")[self._order_seq % 3])
        quantity = parameters.get("quantity", 1)
        if source not in ("S1", "S2", "S3"):
            raise ValueError("자재 위치는 S1, S2, S3 중에서 선택하세요")
        if type(quantity) is not int or quantity <= 0:
            raise ValueError("수량은 양의 정수여야 합니다")
        inventory = cast(dict[str, Any], self.inventory.snapshot())
        tote = next((t for t in inventory["totes"]
                     if t["state"] == "available" and t["location"] == source), None)
        if tote is None:
            raise ValueError(f"{source}에 사용 가능한 빈 토트가 없습니다. 회수·반납을 기다리세요")
        next_id = self._order_seq + 1
        order = f"ORD-{next_id:04}"
        self.inventory.reserve(f"reserve:{order}", order, source, "sensor-kit", quantity, tote["tote_id"])
        # A produced lot keeps its identity after the input tote is emptied.
        # Reusing a tote must never reuse a production payload/business key.
        self.ledger.add_order(order, f"LOT-{next_id:04}", source, ready_time=self.sim_time)
        self._order_seq = next_id
        self._event("order_created", f"{order} 자재 공급 요청", None)
        return order

    def _event(self, kind: str, message: str, robot: str | None, **details: Any) -> None:
        self._event_seq += 1
        self._events.append({"id": f"event-{self._event_seq}",
                             "sim_time_s": self.sim_time, "type": kind, "message": message,
                             "robot_id": robot, **details})

    def set_equipment_availability(self, parameters: Mapping[str, Any]) -> dict[str, Any]:
        """Apply a CELL1 operator override through the replayable ledger."""
        unexpected = set(parameters) - {"available", "reason"}
        if unexpected:
            raise ValueError(f"지원하지 않는 설비 매개변수입니다: {', '.join(sorted(unexpected))}")
        available = parameters.get("available")
        if type(available) is not bool:
            raise ValueError("설비 가동 여부는 available 불리언으로 지정하세요")
        reason = parameters.get("reason")
        if reason is not None and (not isinstance(reason, str) or not reason.strip() or len(reason.strip()) > 160):
            raise ValueError("설비 중단 사유는 1~160자 문자열 또는 null이어야 합니다")
        reason = reason.strip() if isinstance(reason, str) else None

        self._equipment_seq += 1
        result = self.ledger.set_equipment_availability(
            EventId(self.ledger.run_id, "web-equipment", self._equipment_seq),
            sim_time=self.sim_time,
            available=available,
            reason=reason,
        )
        equipment = self._equipment_snapshot()
        event_type = "equipment_recovered" if available else "equipment_down"
        message = "CELL1 설비 복구" if available else f"CELL1 설비 중단{f': {reason}' if reason else ''}"
        self._event(
            event_type,
            message,
            None,
            station_id="CELL1",
            available=available,
            reason=reason,
            active_order_id=equipment["active_order_id"],
            remaining_service_s=equipment["remaining_service_s"],
            no_op=result.duplicate,
        )
        return dict(result.event)

    def _equipment_snapshot(self) -> dict[str, Any]:
        ledger = self.ledger.snapshot()
        equipment = ledger["equipment"]
        processing = next((order for order in ledger["orders"].values()
                           if order["state"] == OrderState.PROCESSING.value), None)
        remaining = equipment["remaining_service"]
        if remaining is None and processing is not None and processing["service_end"] is not None:
            remaining = max(0.0, processing["service_end"] - self.sim_time)
        return {
            "station_id": equipment["station_id"],
            "label": "반도체 후공정 검사 CELL1",
            "available": equipment["available"],
            "status": equipment["status"],
            "reason": equipment["reason"],
            "active_order_id": processing["order_id"] if processing is not None else None,
            "remaining_service_s": remaining,
            "input_waiting_orders": ledger["counts"]["ci_input"],
            "output_waiting_orders": ledger["counts"]["co_output"],
            "wip_orders": ledger["counts"]["wip"],
        }

    def _apply(self, task: Any, robot_id: str, transition: Transition) -> None:
        self._seq += 1
        order = self.ledger.orders[task.order_id]
        station = task.pickup_station if transition is Transition.PICKUP else task.drop_station
        self.ledger.apply(Command(EventId(self.ledger.run_id, "web-model", self._seq), self.sim_time,
                                  task.order_id, task.payload_id, task.stage, transition, order.state_version,
                                  task_id=task.task_id, robot_id=robot_id,
                                  station_id=station if transition is not Transition.ASSIGN else None))

    def _trip(self, robot_id: str, task: Any, start: str, goal: str, phase: str,
              *, kind: str = "order") -> None:
        self._trips[robot_id] = {"task": task, "phase": phase, "start": start, "goal": goal,
                                 "kind": kind,
                                 "points": deque(self._route(start, goal)[1:])}
        if self.charging is not None:
            self.charging.denied.discard(robot_id)

    def _dispatch(self) -> None:
        while True:
            available = [r for r in self.ledger.robots.values() if r.robot_id not in self._trips
                         and r.available and r.task_id is None and r.payload_id is None
                         and (self.charging is None or self.charging.available(r.robot_id))]
            if not available:
                return
            def cost(robot: Any, task: Any) -> float | None:
                if self.charging is not None and not self.charging.can_assign(self, robot.robot_id, task.pickup_station, task.drop_station):
                    return None
                return self.book.plan(self._locations[robot.robot_id], task.pickup_station).distance_m
            assignment = choose_fifo(self.ledger.ready_tasks, available, cost,
                                     feasible=self.ledger.can_assign)
            returns = sorted((t for t in self._return_tasks.values() if t["state"] == "ready"),
                             key=lambda t: (t["ready_time_s"], t["id"]))
            order_task = self.ledger.tasks[assignment.task_id] if assignment else None
            return_choice = None
            for pending in returns:
                candidates = [r for r in available if self.charging is None or self.charging.can_assign(
                    self, r.robot_id, pending["source"], pending["destination"])]
                if candidates:
                    return_choice = (pending, candidates)
                    break
            if return_choice and (order_task is None or return_choice[0]["ready_time_s"] <= order_task.ready_time):
                return_task, candidates = return_choice
                robot = min(candidates, key=lambda r: (
                    self.book.plan(self._locations[r.robot_id], return_task["source"]).distance_m, r.robot_id))
                return_task.update(state="assigned", robot_id=robot.robot_id)
                self._trip(robot.robot_id, return_task, self._locations[robot.robot_id], return_task["source"],
                           "pickup", kind="empty_return")
                self._event("empty_return_assigned", f"{robot.robot_id} {return_task['payload_id']} 빈 토트 회수 배정", robot.robot_id)
            elif assignment is not None:
                task = self.ledger.tasks[assignment.task_id]
                robot_id = assignment.robot_id
                self._apply(task, robot_id, Transition.ASSIGN)
                self._trip(robot_id, task, self._locations[robot_id], task.pickup_station, "pickup")
                self._event("assigned", f"{robot_id} {task.order_id} 배정", robot_id)
            else:
                return

    def _finish_return_leg(self, robot_id: str, trip: dict[str, Any]) -> None:
        task = trip["task"]
        if trip["phase"] == "pickup":
            self.inventory.collect_empty(f"{task['id']}:pickup", task["payload_id"], robot_id)
            task["state"] = "started"
            self._trip(robot_id, task, task["source"], task["destination"], "drop", kind="empty_return")
            self._event("empty_pickup", f"{robot_id} {task['payload_id']} 빈 토트 적재", robot_id)
        else:
            self.inventory.return_empty(f"{task['id']}:drop", task["payload_id"], task["destination"])
            task.update(state="complete", progress=1)
            self._locations[robot_id] = task["destination"]
            del self._trips[robot_id]
            self._event("empty_returned", f"{robot_id} {task['payload_id']} {task['destination']} 반납 완료", robot_id)

    def advance(self, dt: float) -> None:
        if not math.isfinite(dt) or not 0 <= dt <= 1:
            raise ValueError("model step must be between zero and one second")
        if self.paused or dt == 0:
            return
        if self.charging is None:
            self._advance_step(dt)
        else:
            steps = max(1, math.ceil(dt / self.charging.max_step_s))
            for _ in range(steps):
                self._advance_step(dt / steps)

    def _move_trip(self, robot_id: str, trip: dict[str, Any], dt: float) -> None:
        """Advance one authored route leg; specialized operating models may override it."""
        pose = self._poses[robot_id]
        distance_left = 0.65 * dt  # Authored CPU display speed, not a measured AMR response.
        if self.charging is not None:
            move_path(
                pose,
                trip["points"],
                dt,
                self.charging.profile["speed_m_s"],
                self.charging.profile["angular_speed_rad_s"],
            )
            distance_left = 0
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

    def _arrive_trip(self, robot_id: str, trip: dict[str, Any]) -> None:
        """Commit one completed route leg; specialized models may handle extra trip kinds."""
        if trip["kind"] == "empty_return":
            self._finish_return_leg(robot_id, trip)
            return
        task = trip["task"]
        if trip["phase"] == "pickup":
            if task.stage is Stage.INBOUND:
                self.inventory.pickup(f"pickup:{task.order_id}", task.order_id, robot_id)
            self._apply(task, robot_id, Transition.PICKUP)
            self._trip(robot_id, task, task.pickup_station, task.drop_station, "drop")
            self._event("pickup", f"{robot_id} {task.payload_id} 적재 확인", robot_id)
            return
        self._apply(task, robot_id, Transition.DROP)
        if task.stage is Stage.INBOUND:
            self.inventory.deliver(f"deliver:{task.order_id}", task.order_id, "CI")
        else:
            self._apply(task, robot_id, Transition.ORDER_COMPLETE)
        del self._trips[robot_id]
        self._locations[robot_id] = task.drop_station
        self._event("drop", f"{robot_id} {task.drop_station} 인계 확인", robot_id)

    def _advance_auxiliary(self, dt: float, before: Mapping[str, Mapping[str, float]]) -> None:
        """Advance optional preview-only adapters after route arrivals."""
        if self.charging is not None:
            self.charging.move(self, dt)
            self.charging.record(self, dt, before)
            self.charging.prepare(self)

    def _prepare_auxiliary(self) -> None:
        """Give preview adapters a second admission pass after dispatch."""
        if self.charging is not None:
            self.charging.prepare(self)

    def _advance_step(self, dt: float) -> None:
        before = {r: dict(p) for r, p in self._poses.items()} if self.charging is not None else {}
        self.sim_time += dt
        self.ledger.advance(self.sim_time)
        for order in self.ledger.orders.values():
            if order.state is OrderState.PROCESSING and order.order_id not in self._consumed:
                self.inventory.consume(f"consume:{order.order_id}", order.order_id)
                self._consumed.add(order.order_id)
                request = next(r for r in cast(dict[str, Any], self.inventory.snapshot())["requests"]
                               if r["request_id"] == order.order_id)
                task_id = f"{order.order_id}:empty"
                self._return_tasks[task_id] = {
                    "id": task_id, "order_id": order.order_id, "type": "empty_return", "state": "ready",
                    "robot_id": None, "source": "CI", "destination": order.source_station,
                    "payload_id": request["tote_id"], "ready_time_s": self.sim_time, "progress": None}
                self._event("material_consumed", f"{order.order_id} 자재 {request['quantity']}개 소비 · 빈 토트 회수 요청", None)
        for robot_id, trip in list(self._trips.items()):
            if self.charging is not None and robot_id in self.charging.faults:
                continue
            self._move_trip(robot_id, trip, dt)
            if trip["points"]:
                continue
            self._arrive_trip(robot_id, trip)
        self._advance_auxiliary(dt, before)
        self._dispatch()
        self._prepare_auxiliary()

    def snapshot(self) -> dict[str, Any]:
        stock = cast(dict[str, Any], self.inventory.snapshot())
        equipment = self._equipment_snapshot()
        robots = []
        for robot_id, robot in self.ledger.robots.items():
            trip = self._trips.get(robot_id)
            task = trip["task"] if trip else None
            payload_id = stock["robot_loads"].get(robot_id, robot.payload_id)
            if trip and trip["kind"] == "parking":
                task_id, task_type, task_label = f"park:{robot_id}", "parking", "주차 이동"
                payload_id = None
            elif trip and trip["kind"] == "charging":
                task_id, task_type, task_label = f"charge:{robot_id}", "charging", "운영 가상 충전"
                payload_id = None
            elif trip and trip["kind"] == "empty_return":
                return_task = trip["task"]
                task_id, task_type, task_label = return_task["id"], "empty_return", "빈 토트 반납"
                payload_id = return_task["payload_id"] if trip["phase"] == "drop" else None
            else:
                task_id = task.task_id if task else None
                task_type = task.stage.value if task else None
                task_label = ("자재 공급" if task.stage is Stage.INBOUND else "완제품 출하") if task else None
            robots.append({"id": robot_id, "label": f"AMR {robot_id.rsplit('-', 1)[-1].zfill(2)}",
                           "status": "paused" if self.paused and trip else "moving" if trip else "idle",
                           "pose": dict(self._poses[robot_id]), "speed_mps": 0.65 if trip and not self.paused else 0,
                           "battery_pct": None, "battery_source": "unavailable",
                           "task_id": task_id, "task_type": task_type, "task_label": task_label,
                           "payload_id": payload_id,
                           "destination": trip["goal"] if trip else None,
                           "waiting_reason": "작업 일시정지" if self.paused and trip else None,
                           "route": [{"x_m": x, "y_m": y} for x, y in trip["points"]] if trip else [],
                           "observed_sim_time_s": self.sim_time})
            if trip is None and not equipment["available"] and equipment["wip_orders"]:
                if equipment["active_order_id"] is not None:
                    robots[-1]["waiting_reason"] = (
                        f"CELL1 중단 · {equipment['active_order_id']} 공정 잔여 "
                        f"{equipment['remaining_service_s']:.1f}초"
                    )
                else:
                    robots[-1]["waiting_reason"] = (
                        f"CELL1 중단 · 입력 공정 대기 {equipment['input_waiting_orders']}건"
                    )
            if self.charging is not None:
                row = robots[-1]
                row.update(self.charging.robot_snapshot(robot_id))
                row["speed_mps"] = 0. if self.paused else self.charging.speeds[robot_id]
                auxiliary = self.charging.aux.get(robot_id)
                phase = row["charging_phase"]
                if auxiliary or phase == "queued":
                    row.update(task_id=f"charge:{robot_id}", task_type="undocking" if phase == "undocking" else "charging",
                               task_label="CPU 출구 이동" if phase == "undocking" else "CPU 가상 충전",
                               destination=auxiliary["goal"] if auxiliary else None,
                               route=[{"x_m": x, "y_m": y} for x, y in auxiliary["points"]] if auxiliary else [],
                               status="paused" if self.paused else "moving" if row["speed_mps"] else "waiting",
                               waiting_reason="작업 일시정지" if self.paused else f"CPU 충전: {phase}")
                if phase == "fault":
                    row.update(status="waiting", waiting_reason="CPU 가상 에너지 오류")
                elif robot_id in self.charging.denied and trip is None and phase is None:
                    row.update(waiting_reason="CPU 에너지·충전 복귀 예산 부족")
        tasks = [{"id": t.task_id, "order_id": t.order_id, "type": t.stage.value, "state": t.status.value,
                  "robot_id": t.robot_id, "source": t.pickup_station, "destination": t.drop_station,
                  "payload_id": t.payload_id, "progress": 1 if t.status.value == "complete" else None}
                 for t in self.ledger.tasks.values()]
        tasks.extend(dict(t) for t in self._return_tasks.values())
        if self.charging is not None:
            tasks.extend({"id": f"charge:{r}", "order_id": "", "type": "undocking" if t["phase"] == "undocking" else "charging",
                          "state": t["phase"], "robot_id": r, "source": self._locations[r], "destination": t["goal"],
                          "payload_id": None, "progress": None} for r, t in self.charging.aux.items())
        result = {"schema_version": 1, "run_id": self.ledger.run_id, "revision": self.revision,
                "mode": "local_model", "execution_mode": "preview", "sim_time_s": self.sim_time,
                "observed_at": datetime.now(timezone.utc).isoformat(), "connection": "connected",
                "paused": self.paused,
                "capabilities": ["pause", "resume", "create_order", "set_equipment_availability"],
                "robots": robots, "tasks": tasks, "inventory": stock,
                "equipment": equipment,
                "kpis": {"active_robots": len(self._trips), "total_robots": len(robots),
                         "completed_orders": sum(o.state is OrderState.COMPLETE for o in self.ledger.orders.values()),
                         "pending_tasks": sum(t["state"] != "complete" for t in tasks),
                         "stock_units": sum(s["on_hand"] for s in stock["stock"])},
                "events": list(self._events), "commands": [],
                "model_scope": "CPU ledger and authored route interpolation; no physics, traffic clearance, battery or real MES"}
        if self.charging is not None:
            result["charging"] = self.charging.snapshot()
            result["model_scope"] = CHARGING_SCOPE + "; operational ledger load, no real MES or traffic clearance"
            result["kpis"]["active_robots"] += len(self.charging.aux)
        return result
