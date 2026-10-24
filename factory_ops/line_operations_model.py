"""Bind ProcessLine authority to the existing kinematic operations runtime."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Mapping

from .operations import Stage
from .operations_model import OperationsModel
from .process_line import ProcessLine, TransportRequest
from .process_network import ProcessNetwork


@dataclass(frozen=True)
class _TripTask:
    task_id: str
    order_id: str
    payload_id: str
    pickup_station: str
    drop_station: str
    kind: str
    stage: Stage = Stage.INBOUND

    @classmethod
    def from_request(cls, task: TransportRequest) -> "_TripTask":
        return cls(task.task_id, task.lot_id, task.carrier_id, task.origin, task.destination, task.kind)


class _LineInventory:
    """Small finite inventory projection used by charging and the dashboard."""

    def __init__(self, sources: tuple[str, ...] = ("S1", "S2", "S3"), *,
                 stock_per_source: int = 20, carrier_count: int = 16,
                 shipping_stations: frozenset[str] = frozenset({"OUT"})) -> None:
        if not sources or len(set(sources)) != len(sources) or any(
            type(value) is not int or value <= 0 for value in (stock_per_source, carrier_count)
        ):
            raise ValueError("inventory requires distinct sources and positive stock/carrier counts")
        self.sources = sources
        self.shipping_stations = shipping_stations
        self.initial_stock = {source: stock_per_source for source in sources}
        self.stock = dict(self.initial_stock)
        self.consumed = 0
        self.requests: dict[str, dict[str, Any]] = {}
        self.carriers = {
            f"T{index:03}": {
                "tote_id": f"T{index:03}", "state": "available",
                "location": sources[(index - 1) % len(sources)], "owner": None, "request_id": None,
            }
            for index in range(1, carrier_count + 1)
        }
        self.robot_loads: dict[str, str] = {}

    def available_carrier(self, source: str) -> str | None:
        return next((key for key, carrier in sorted(self.carriers.items())
                     if carrier["state"] == "available" and carrier["location"] == source), None)

    def reserve(self, lot_id: str, source: str, quantity: int, carrier_id: str) -> None:
        if self.stock[source] - sum(row["quantity"] for row in self.requests.values()
                                    if row["source"] == source and row["state"] == "reserved") < quantity:
            raise ValueError("insufficient source stock")
        carrier = self.carriers[carrier_id]
        if carrier["state"] != "available" or carrier["location"] != source:
            raise ValueError("carrier is unavailable at source")
        self.requests[lot_id] = {"request_id": lot_id, "source": source, "sku": "packaged-ic-tray",
                                 "quantity": quantity, "tote_id": carrier_id, "state": "reserved",
                                 "robot_id": None, "destination": None}
        carrier.update(state="filled", request_id=lot_id)

    def source_pickup(self, lot_id: str, robot: str) -> None:
        request = self.requests[lot_id]
        if request["state"] != "reserved" or robot in self.robot_loads:
            raise ValueError("source pickup inventory state is invalid")
        self.stock[request["source"]] -= request["quantity"]
        request.update(state="picked", robot_id=robot)
        self._load(request["tote_id"], robot, expected="filled")

    def product_pickup(self, lot_id: str, carrier_id: str, robot: str, origin: str) -> None:
        carrier = self.carriers[carrier_id]
        if carrier["location"] != origin or carrier["request_id"] != lot_id:
            raise ValueError("product carrier is not at the declared origin")
        self._load(carrier_id, robot, expected="filled")

    def process_release(self, lot_id: str, carrier_id: str, output_station: str) -> None:
        carrier = self.carriers[carrier_id]
        if carrier["owner"] is not None or carrier["request_id"] != lot_id or carrier["state"] != "filled":
            raise ValueError("process output carrier state is invalid")
        carrier["location"] = output_station
        self.requests[lot_id].update(state="in_process", destination=output_station)

    def product_drop(self, lot_id: str, carrier_id: str, robot: str, destination: str) -> None:
        self._unload(carrier_id, robot, destination)
        request = self.requests[lot_id]
        request["destination"] = destination
        if destination in self.shipping_stations:
            request["state"] = "consumed"
            self.consumed += request["quantity"]
            self.carriers[carrier_id]["state"] = "empty"
        else:
            request["state"] = "in_process"

    def empty_pickup(self, carrier_id: str, robot: str) -> None:
        self._load(carrier_id, robot, expected="empty")
        self.carriers[carrier_id]["state"] = "returning"

    def empty_return(self, carrier_id: str, robot: str, source: str) -> None:
        self._unload(carrier_id, robot, source)
        self.carriers[carrier_id].update(state="available", request_id=None)

    def _load(self, carrier_id: str, robot: str, *, expected: str) -> None:
        carrier = self.carriers[carrier_id]
        if carrier["state"] != expected or carrier["owner"] is not None or robot in self.robot_loads:
            raise ValueError("robot/carrier load state is invalid")
        carrier.update(owner=robot, location="IN_TRANSIT")
        self.robot_loads[robot] = carrier_id

    def _unload(self, carrier_id: str, robot: str, destination: str) -> None:
        carrier = self.carriers[carrier_id]
        if carrier["owner"] != robot or self.robot_loads.get(robot) != carrier_id:
            raise ValueError("robot does not own the carrier")
        del self.robot_loads[robot]
        carrier.update(owner=None, location=destination)

    def snapshot(self) -> dict[str, Any]:
        stock = []
        for source in self.sources:
            reserved = sum(row["quantity"] for row in self.requests.values()
                           if row["source"] == source and row["state"] == "reserved")
            stock.append({"location": source, "sku": "packaged-ic-tray", "on_hand": self.stock[source],
                          "reserved": reserved, "available": self.stock[source] - reserved})
        in_carriers = sum(row["quantity"] for row in self.requests.values()
                          if row["state"] in {"picked", "in_process"})
        return {
            "model_scope": "finite authored line inventory and carrier ownership; no physical payload sensing",
            "stock": stock,
            "requests": [deepcopy(row) for row in self.requests.values()],
            "totes": [deepcopy(row) for row in self.carriers.values()],
            "robot_loads": dict(self.robot_loads),
            "conservation": [{"sku": "packaged-ic-tray", "initial": sum(self.initial_stock.values()), "replenished": 0,
                              "on_hand": sum(self.stock.values()), "in_totes": in_carriers,
                              "consumed": self.consumed,
                              "balanced": sum(self.initial_stock.values()) == sum(self.stock.values()) + in_carriers + self.consumed}],
        }


class LineOperationsModel(OperationsModel):
    """Kinematic operations adapter with ProcessLine as the production authority."""

    def __init__(self, config: Path, *, policy: str = "fifo", initial_lots: int = 1,
                 run_id: str | None = None) -> None:
        if type(initial_lots) is not int or initial_lots < 0:
            raise ValueError("initial_lots must be a nonnegative integer")
        super().__init__(config, policy=policy, initial_orders=0, run_id=run_id)
        network_path = config / "process-network.json"
        if network_path.is_file():
            network = json.loads(network_path.read_text(encoding="utf-8"))
            self.line = ProcessNetwork(self.ledger.run_id, network)
            dispatch_mode = network.get("dispatch_mode")
            if dispatch_mode != "downstream_wip_drain_least_assigned":
                raise ValueError("process network requires downstream_wip_drain_least_assigned dispatch")
            inventory = network.get("inventory", {})
            self.inventory = _LineInventory(tuple(self.line.source_to_line),
                stock_per_source=inventory.get("stock_per_source", 20),
                carrier_count=inventory.get("carrier_count", 64),
                shipping_stations=self.line.shipping_stations)
            self._line_dispatch_mode = dispatch_mode
        else:
            self.line = ProcessLine(self.ledger.run_id, json.loads((config / "process-line.json").read_text(encoding="utf-8")))
            self.inventory = _LineInventory(shipping_stations=frozenset({self.line.shipping_station}))
            self._line_dispatch_mode = "legacy_fifo_eta"
        self._peak_transport_active = 0
        self._peak_transport_moving = 0
        self._transport_moving_s = {robot: 0.0 for robot in self.ledger.robots}
        self._line_observation_seq = 0
        self._cell_reasons = {cell_id: None for cell_id in self.line.cells}
        for index in range(initial_lots):
            self.create_order({"source": self.inventory.sources[index % len(self.inventory.sources)]})

    def create_order(self, parameters: dict[str, Any]) -> str:
        if not isinstance(parameters, dict) or set(parameters) - {"source", "quantity"}:
            raise ValueError("line lot accepts only source and quantity")
        source = parameters.get("source", self.inventory.sources[self._order_seq % len(self.inventory.sources)])
        quantity = parameters.get("quantity", 1)
        if source not in self.inventory.sources or type(quantity) is not int or quantity <= 0:
            raise ValueError("source must be authored and quantity must be positive")
        carrier = self.inventory.available_carrier(source)
        if carrier is None:
            raise ValueError(f"{source} has no returned carrier available")
        next_id = self._order_seq + 1
        lot_id = f"LOT-{next_id:04}"
        self.inventory.reserve(lot_id, source, quantity, carrier)
        self.line.release_lot(lot_id, carrier, source)
        self._order_seq = next_id
        self._event("line_lot_released", f"{lot_id} TEST→AOI→PACK 투입", None,
                    lot_id=lot_id, carrier_id=carrier, source=source)
        return lot_id

    def _line_can_assign(self, task: TransportRequest) -> bool:
        cell_id = self.line.input_to_cell.get(task.destination)
        if cell_id is None:
            return True
        cell = self.line.cells[cell_id]
        return len(cell.input_lots) + cell.reserved_inputs < cell.spec.input_capacity

    def _line_task_priority(self, task: TransportRequest) -> tuple[Any, ...]:
        if self._line_dispatch_mode == "legacy_fifo_eta":
            return task.created_sim_time_s, task.task_id
        if task.kind == "empty_return":
            stage = 0
        elif task.destination in self.line.shipping_stations:
            stage = 1
        else:
            cell_id = self.line.input_to_cell.get(task.destination)
            suffix = cell_id.rsplit(".", 1)[-1] if cell_id is not None else ""
            stage = {"PACK": 2, "AOI": 3, "TEST": 4}.get(suffix, 5)
        return stage, task.created_sim_time_s, task.task_id

    def _charging_ready_jobs(self) -> list[tuple[str, str, str]]:
        jobs = super()._charging_ready_jobs()
        jobs.extend(
            (task.task_id, task.origin, task.destination)
            for task in self.line.ready_transports
            if self._line_can_assign(task)
        )
        return jobs

    def _dispatch_extra_jobs(self, draining: bool) -> None:
        if draining:
            return
        while True:
            available = [robot for robot, state in self.ledger.robots.items()
                         if (robot not in self._trips or self._can_withdraw_queued_parking(robot))
                         and state.available and state.task_id is None
                         and state.payload_id is None and robot not in self.inventory.robot_loads
                         and (self.operations_charging is None or self.operations_charging.available(robot))]
            candidates = []
            for task in self.line.ready_transports:
                if not self._line_can_assign(task):
                    continue
                for robot in available:
                    blockers = sorted({self._station_owners[station] for station in (task.origin, task.destination)
                                       if station in self._station_owners and self._station_owners[station] != robot})
                    if blockers:
                        continue
                    pickup_route = self.book.plan(
                        self._dispatch_origin(robot), task.origin, closed=self.closed,
                    )
                    loaded_route = self.book.plan(task.origin, task.destination, closed=self.closed)
                    if self._prospective_trip_blockers(
                        robot,
                        task.origin,
                        task.destination,
                        pickup_route=pickup_route,
                        loaded_route=loaded_route,
                    ):
                        continue
                    first = pickup_route.distance_m
                    total = first + loaded_route.distance_m
                    if self._line_dispatch_mode == "legacy_fifo_eta":
                        rank = ((task.created_sim_time_s, task.task_id, first, self._assignments[robot], robot)
                                if self.dispatch_policy == "fifo"
                                else (total / self.limits.max_linear_m_s, task.created_sim_time_s,
                                      self._assignments[robot], task.task_id, robot))
                    else:
                        task_priority = self._line_task_priority(task)
                        rank = ((*task_priority, self._assignments[robot], first, robot)
                                if self.dispatch_policy == "fifo"
                                else (*task_priority, self._assignments[robot],
                                      total / self.limits.max_linear_m_s, robot))
                    candidates.append((rank, robot, task))
            selected = next((candidate for candidate in sorted(candidates, key=lambda row: row[0])
                             if self.operations_charging is None or self.operations_charging.can_assign(
                                 self, candidate[1], candidate[2].origin, candidate[2].destination)), None)
            if selected is None:
                return
            _, robot, task = selected
            if robot in self._trips and not self._withdraw_queued_parking(robot):
                raise RuntimeError("selected queued parking trip is no longer preemptible")
            assigned = self.line.assign(task.task_id, robot)
            self._station_owners.update({assigned.origin: robot, assigned.destination: robot})
            self._jobs[robot] = (assigned.origin, assigned.destination)
            adapter = _TripTask.from_request(assigned)
            self._trip(robot, adapter, self._locations[robot], assigned.origin, "pickup", kind="process_line")
            self._assignments[robot] += 1
            self._event("line_transport_assigned", f"{robot} {assigned.task_id} 배정", robot,
                        task_id=assigned.task_id, lot_id=assigned.lot_id)

    def _observation_id(self, transition: str, task_id: str) -> str:
        self._line_observation_seq += 1
        return f"line:{self._line_observation_seq}:{transition}:{task_id}"

    def _sync_process_outputs(self) -> None:
        for task in self.line.ready_transports:
            if task.kind != "product_move":
                continue
            carrier = self.inventory.carriers[task.carrier_id]
            if carrier["location"] != task.origin:
                self.inventory.process_release(task.lot_id, task.carrier_id, task.origin)

    def _pickup_line_task(self, robot_id: str, task: _TripTask) -> None:
        self.line.pickup(
            self.ledger.run_id,
            task.task_id,
            robot_id,
            self._observation_id("pickup", task.task_id),
            self.sim_time,
        )
        if task.kind == "process_input":
            self.inventory.source_pickup(task.order_id, robot_id)
        elif task.kind == "empty_return":
            self.inventory.empty_pickup(task.payload_id, robot_id)
        else:
            self.inventory.product_pickup(
                task.order_id, task.payload_id, robot_id, task.pickup_station,
            )

    def _chain_ready_shipping_return(self, robot_id: str, delivered: _TripTask) -> bool:
        """Pick an empty carrier at its shipping handover without vacating first."""
        station = delivered.drop_station
        if (self._line_dispatch_mode == "legacy_fifo_eta"
                or delivered.kind != "product_move"
                or station not in self.line.shipping_stations):
            return False
        candidate = next(
            (task for task in self.line.ready_transports
             if task.kind == "empty_return"
             and task.lot_id == delivered.order_id
             and task.carrier_id == delivered.payload_id
             and task.origin == station),
            None,
        )
        if candidate is None or not self._line_can_assign(candidate):
            return False
        blockers = sorted({
            self._station_owners[claimed]
            for claimed in (candidate.origin, candidate.destination)
            if claimed in self._station_owners
            and self._station_owners[claimed] != robot_id
        })
        if blockers:
            return False
        adapter = _TripTask.from_request(candidate)
        loaded_route = self.book.plan(candidate.origin, candidate.destination, closed=self.closed)
        loaded_trip = {
            "kind": "process_line",
            "phase": "drop",
            "goal": candidate.destination,
            "task": adapter,
        }
        window = self._window_for_trip(
            loaded_route.points[0], loaded_route.points[1:], loaded_route.nodes[1:],
            loaded_trip, robot_id=robot_id,
        )
        if self.bridges.blockers(
            robot_id,
            window["nodes"],
            route_tiles=window["tiles"],
            observed_tiles=self._admission_observations(robot_id, loaded_trip),
        ):
            return False
        if (self.operations_charging is not None
                and not self.operations_charging.can_assign(
                    self, robot_id, candidate.origin, candidate.destination,
                )):
            return False
        assigned = self.line.assign(candidate.task_id, robot_id)
        self._station_owners.update({
            assigned.origin: robot_id,
            assigned.destination: robot_id,
        })
        self._jobs[robot_id] = (assigned.origin, assigned.destination)
        adapter = _TripTask.from_request(assigned)
        self._pickup_line_task(robot_id, adapter)
        self._trip(
            robot_id, adapter, assigned.origin, assigned.destination, "drop",
            kind="process_line",
        )
        self._assignments[robot_id] += 1
        self._event(
            "line_transport_assigned",
            f"{robot_id} {assigned.task_id} 연속 인계",
            robot_id,
            task_id=assigned.task_id,
            lot_id=assigned.lot_id,
            handoff="same_station",
            avoided_staging_reapproach=True,
        )
        self._event(
            "line_pickup",
            f"{robot_id} {adapter.payload_id} 적재",
            robot_id,
            task_id=adapter.task_id,
            station=adapter.pickup_station,
            handoff="same_station",
        )
        return True

    def _arrive_trip(self, robot_id: str, trip: dict[str, Any]) -> None:
        if trip["kind"] != "process_line":
            super()._arrive_trip(robot_id, trip)
            return
        if not self.motions[robot_id].stopped:
            raise RuntimeError("line handover attempted before kinematic stop")
        task: _TripTask = trip["task"]
        staging = (self._service_exit_after_drop(robot_id, task.drop_station)
                   if trip["phase"] != "pickup" else None)
        if trip["phase"] != "pickup" and staging is None:
            self.bridges.release(robot_id, stopped=True)
        self.line.advance(self.sim_time)
        self._sync_process_outputs()
        if trip["phase"] == "pickup":
            self._pickup_line_task(robot_id, task)
            self._trip(robot_id, task, task.pickup_station, task.drop_station, "drop", kind="process_line")
            self._event("line_pickup", f"{robot_id} {task.payload_id} 적재", robot_id,
                        task_id=task.task_id, station=task.pickup_station)
        else:
            self.line.drop(self.ledger.run_id, task.task_id, robot_id,
                           self._observation_id("drop", task.task_id), self.sim_time)
            if task.kind == "empty_return":
                self.inventory.empty_return(task.payload_id, robot_id, task.drop_station)
            else:
                self.inventory.product_drop(task.order_id, task.payload_id, robot_id, task.drop_station)
            self._deliveries[robot_id] += 1
            self._jobs.pop(robot_id, None)
            self._locations[robot_id] = task.drop_station
            del self._trips[robot_id]
            self._event("line_drop", f"{robot_id} {task.drop_station} 인계", robot_id,
                        task_id=task.task_id, station=task.drop_station)
            if (staging is not None
                    and not self._chain_ready_shipping_return(robot_id, task)):
                # The business handover is complete, but the vehicle retains
                # bridge protection until its footprint clears the spur mouth.
                self._trip(
                    robot_id, {}, task.drop_station, staging, "parking",
                    kind="parking", purpose=("staging" if task.drop_station in self.staging_after_drop
                                               else "service_exit"),
                )
        self._idle_since[robot_id] = self.sim_time

    def _advance_auxiliary(self, dt: float, before: Mapping[str, Mapping[str, float]]) -> None:
        self.line.advance(self.sim_time)
        self._sync_process_outputs()
        super()._advance_auxiliary(dt, before)
        active = [robot for robot, trip in self._trips.items() if trip["kind"] == "process_line"]
        moving = [robot for robot in active if not self.motions[robot].stopped]
        self._peak_transport_active = max(self._peak_transport_active, len(active))
        self._peak_transport_moving = max(self._peak_transport_moving, len(moving))
        for robot in moving:
            self._transport_moving_s[robot] += dt

    def set_equipment_availability(self, parameters: Mapping[str, Any]) -> dict[str, Any]:
        if set(parameters) - {"cell_id", "available", "reason"}:
            raise ValueError("line equipment accepts cell_id, available and reason")
        cell_id = parameters.get("cell_id", self.line.cell_order[0])
        available = parameters.get("available")
        reason = parameters.get("reason")
        if cell_id not in self.line.cells or type(available) is not bool:
            raise ValueError("known cell_id and boolean available are required")
        if reason is not None and (not isinstance(reason, str) or not reason.strip()):
            raise ValueError("reason must be a non-empty string or null")
        self.line.advance(self.sim_time)
        self.line.set_available(cell_id, available)
        self._cell_reasons[cell_id] = reason.strip() if isinstance(reason, str) else None
        self._event("line_cell_recovered" if available else "line_cell_down",
                    f"{cell_id} {'복구' if available else '중단'}", None,
                    cell_id=cell_id, reason=self._cell_reasons[cell_id])
        return {"cell_id": cell_id, "available": available, "reason": self._cell_reasons[cell_id]}

    def _cell_snapshot(self, cell_id: str) -> dict[str, Any]:
        cell = self.line.cells[cell_id]
        return {
            "station_id": cell_id,
            "label": {"TEST": "반도체 최종 검사", "AOI": "광학 검사·마킹", "PACK": "ESD 포장"}[cell_id.rsplit(".", 1)[-1]],
            "available": cell.available,
            "status": "available" if cell.available else "down",
            "reason": self._cell_reasons[cell_id],
            "active_order_id": cell.active_lot,
            "remaining_service_s": cell.remaining_s,
            "input_waiting_orders": len(cell.input_lots) + cell.reserved_inputs,
            "output_waiting_orders": len(cell.output_lots),
            "wip_orders": len(cell.input_lots) + len(cell.output_lots) + (cell.active_lot is not None),
        }

    def _equipment_snapshot(self) -> dict[str, Any]:
        return self._cell_snapshot(self.line.cell_order[0])

    def snapshot(self) -> dict[str, Any]:
        result = super().snapshot()
        line = self.line.snapshot()
        charging_tasks = [task for task in result["tasks"] if task["type"] == "charging"]
        line_tasks = [{
            "id": task["task_id"], "order_id": task["lot_id"], "type": task["kind"],
            "state": task["state"], "robot_id": task["robot_id"], "source": task["origin"],
            "destination": task["destination"], "payload_id": task["carrier_id"],
            "progress": 1.0 if task["state"] == "complete" else None,
        } for task in line["transports"]]
        result["tasks"] = line_tasks + charging_tasks
        loads = result["inventory"]["robot_loads"]
        by_task = {task["task_id"]: task for task in line["transports"]}
        for robot in result["robots"]:
            robot["payload_id"] = loads.get(robot["id"])
            if robot["task_id"] in by_task:
                task = by_task[robot["task_id"]]
                robot.update(task_type=task["kind"],
                             task_label="빈 캐리어 반납" if task["kind"] == "empty_return" else "공정 간 운송")
        result["equipment"] = self._equipment_snapshot()
        result["equipment_cells"] = [self._cell_snapshot(cell_id) for cell_id in self.line.cell_order]
        result["production_line"] = line
        result["production_line"]["dispatch_mode"] = self._line_dispatch_mode
        result["kpis"].update(
            completed_orders=line["completed_lots"],
            returned_carriers=line["returned"],
            pending_tasks=sum(task["state"] != "complete" for task in line_tasks + charging_tasks),
            stock_units=sum(row["on_hand"] for row in result["inventory"]["stock"]),
        )
        result["traffic"]["quiescent"] = (
            result["traffic"]["quiescent"]
            and line["returned"] == len(line["lots"])
            and all(task["state"] == "complete" for task in line["transports"])
        )
        result["traffic"].update(
            peak_transport_active=self._peak_transport_active,
            peak_transport_moving=self._peak_transport_moving,
            transport_moving_s=dict(self._transport_moving_s),
        )
        result["model_scope"] += (
            "; ProcessLine TEST/AOI/PACK production authority with finite authored inventory; "
            "station coordinates and kinematic reservations are authored CPU evidence, not physical runtime"
        )
        return result


__all__ = ["LineOperationsModel"]
