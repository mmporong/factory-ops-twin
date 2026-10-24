"""Rolling route-window admission for authored inter-bay bridges.

Admission is an additional dispatch gate, never a motion permission. Tile
reservations and the stopped-pose footprint remain authoritative for movement.
Only the next atomic corridor span or bounded local segment is acquired.  The
window is retained until the vehicle stops at its authored safe endpoint.
"""
from __future__ import annotations

from typing import Any, Iterable, Mapping


Tile = tuple[int, int]


class BridgeAdmission:
    def __init__(self, routing: Mapping[str, Any]) -> None:
        raw = routing.get("bridge_admission", {"policy": "atomic_route_leg_resources", "resources": {}})
        if (not isinstance(raw, Mapping) or raw.get("policy") != "atomic_route_leg_resources"
                or not isinstance(raw.get("resources"), Mapping)):
            raise ValueError("bridge admission requires atomic route-leg resources")
        queue_nodes = raw.get("queue_nodes", [])
        if (not isinstance(queue_nodes, list)
                or any(not isinstance(node, str) or node not in routing["nodes"]
                       for node in queue_nodes)
                or len(queue_nodes) != len(set(queue_nodes))):
            raise ValueError("bridge admission queue_nodes must uniquely name existing route nodes")
        self.queue_nodes = frozenset(queue_nodes)
        holdable_nodes = raw.get("holdable_nodes")
        if holdable_nodes is not None and (
            not isinstance(holdable_nodes, list)
            or not holdable_nodes
            or any(not isinstance(node, str) or node not in routing["nodes"]
                   for node in holdable_nodes)
            or len(holdable_nodes) != len(set(holdable_nodes))
        ):
            raise ValueError("bridge admission holdable_nodes must uniquely name existing route nodes")
        self.holdable_nodes = None if holdable_nodes is None else frozenset(holdable_nodes)
        if self.holdable_nodes is not None and not self.queue_nodes.issubset(self.holdable_nodes):
            raise ValueError("bridge admission queue_nodes must be holdable")
        safe_exit_nodes = raw.get("safe_exit_nodes", {})
        if (not isinstance(safe_exit_nodes, Mapping)
                or any(not isinstance(queue, str) or queue not in self.queue_nodes
                       or not isinstance(exit_node, str) or exit_node not in routing["nodes"]
                       for queue, exit_node in safe_exit_nodes.items())
                or safe_exit_nodes and set(safe_exit_nodes) != set(self.queue_nodes)
                or self.holdable_nodes is None
                and safe_exit_nodes
                or self.holdable_nodes is not None
                and any(node not in self.holdable_nodes for node in safe_exit_nodes.values())):
            raise ValueError(
                "bridge admission safe_exit_nodes must map every queue to a holdable route node"
            )
        self.safe_exit_nodes = dict(safe_exit_nodes)
        self.resources: dict[str, frozenset[str]] = {}
        for name, nodes in raw["resources"].items():
            if (not isinstance(name, str) or not name or not isinstance(nodes, list) or not nodes
                    or any(not isinstance(node, str) or node not in routing["nodes"] for node in nodes)):
                raise ValueError("bridge admission resource must name existing route nodes")
            if self.queue_nodes.intersection(nodes):
                raise ValueError("bridge admission queue nodes must remain outside bridge resources")
            if set(self.safe_exit_nodes.values()).intersection(nodes):
                raise ValueError("bridge admission safe exit nodes must remain outside bridge resources")
            self.resources[name] = frozenset(nodes)
        self.owners: dict[str, str] = {}
        self.waiters: dict[str, tuple[str, ...]] = {}
        self._protected_tiles: dict[str, frozenset[Tile]] = {}
        self._handover_protected: set[str] = set()

    def required(self, route_nodes: Iterable[str]) -> frozenset[str]:
        route = frozenset(route_nodes)
        return frozenset(name for name, nodes in self.resources.items() if nodes & route)

    def held(self, robot: str) -> frozenset[str]:
        return frozenset(name for name, owner in self.owners.items() if owner == robot)

    def has_window(self, robot: str) -> bool:
        return robot in self._protected_tiles

    def blockers(
        self,
        robot: str,
        route_nodes: Iterable[str],
        *,
        route_tiles: Iterable[Tile] = (),
        observed_tiles: Mapping[str, Iterable[Tile]] | None = None,
    ) -> tuple[str, ...]:
        required = self.required(route_nodes)
        protected = frozenset(route_tiles)
        blockers = {
            self.owners[name]
            for name in required
            if name in self.owners and self.owners[name] != robot
        }
        if protected:
            blockers.update(
                owner for owner, tiles in self._protected_tiles.items()
                if owner != robot and not protected.isdisjoint(tiles)
            )
        already_holds = self.has_window(robot)
        if protected and not already_holds and observed_tiles is not None:
            blockers.update(
                peer for peer, tiles in observed_tiles.items()
                if peer != robot and not protected.isdisjoint(tiles)
            )
        return tuple(sorted(blockers))

    def admit(
        self,
        robot: str,
        route_nodes: Iterable[str],
        *,
        route_tiles: Iterable[Tile] = (),
        observed_tiles: Mapping[str, Iterable[Tile]] | None = None,
        handover_protected: bool = False,
    ) -> tuple[str, ...]:
        route_nodes = tuple(route_nodes)
        protected = frozenset(route_tiles)
        active = self._protected_tiles.get(robot)
        if active is not None:
            if active != protected or self.held(robot) != self.required(route_nodes):
                raise RuntimeError("admission window cannot change before a stopped release")
            self.waiters.pop(robot, None)
            if handover_protected:
                self._handover_protected.add(robot)
            return ()
        blockers = self.blockers(
            robot,
            route_nodes,
            route_tiles=protected,
            observed_tiles=observed_tiles,
        )
        if blockers:
            self.waiters[robot] = blockers
            return blockers
        required = self.required(route_nodes)
        self.owners.update((name, robot) for name in required)
        if protected:
            self._protected_tiles[robot] = protected
        if handover_protected:
            self._handover_protected.add(robot)
        self.waiters.pop(robot, None)
        return ()

    def release(self, robot: str, *, stopped: bool) -> None:
        if not stopped:
            raise RuntimeError("bridge resources cannot be released before a kinematic stop")
        for name in self.held(robot):
            del self.owners[name]
        self._protected_tiles.pop(robot, None)
        self._handover_protected.discard(robot)
        self.waiters.pop(robot, None)

    def retarget(
        self,
        robot: str,
        route_nodes: Iterable[str],
        *,
        route_tiles: Iterable[Tile],
        stopped: bool,
    ) -> None:
        """Narrow a retained handover window after stopping at its waypoint."""
        if not stopped:
            raise RuntimeError("admission window cannot retarget before a kinematic stop")
        active = self._protected_tiles.get(robot)
        protected = frozenset(route_tiles)
        required = self.required(route_nodes)
        held = self.held(robot)
        if active is None or not protected or not protected.issubset(active):
            raise RuntimeError("retargeted admission must be a nonempty subset of the active window")
        if not required.issubset(held):
            raise RuntimeError("retargeted admission cannot acquire a new bridge resource")
        for name in held - required:
            del self.owners[name]
        self._protected_tiles[robot] = protected
        self.waiters.pop(robot, None)

    def can_retarget(
        self,
        robot: str,
        route_nodes: Iterable[str],
        *,
        route_tiles: Iterable[Tile],
    ) -> bool:
        """Report whether a retained window can narrow to one already-protected successor."""
        active = self._protected_tiles.get(robot)
        protected = frozenset(route_tiles)
        required = self.required(route_nodes)
        return (
            active is not None
            and bool(protected)
            and protected.issubset(active)
            and required.issubset(self.held(robot))
        )

    def withdraw_waiter(self, robot: str) -> bool:
        """Withdraw only an unowned admission wait.

        A queued parking intent may be replaced by business work while the
        vehicle is still stopped at its authored anchor.  An owner is already
        committed to a protected route and cannot be withdrawn through this
        narrow API.
        """
        if self.held(robot) or self.has_window(robot):
            return False
        return self.waiters.pop(robot, None) is not None

    def record_wait(self, robot: str, blockers: Iterable[str]) -> None:
        """Expose a read-only lookahead dependency without changing ownership."""
        normalized = tuple(sorted({peer for peer in blockers if peer != robot}))
        if normalized:
            self.waiters[robot] = normalized
        else:
            self.waiters.pop(robot, None)

    def wait_cycle(self) -> tuple[str, ...] | None:
        """Return one canonical strongly connected wait component, if present."""
        graph = {
            robot: tuple(peer for peer in blockers if peer in self.waiters)
            for robot, blockers in self.waiters.items()
        }
        index = 0
        indices: dict[str, int] = {}
        lowlinks: dict[str, int] = {}
        stack: list[str] = []
        on_stack: set[str] = set()
        cycles: list[tuple[str, ...]] = []

        def visit(robot: str) -> None:
            nonlocal index
            indices[robot] = index
            lowlinks[robot] = index
            index += 1
            stack.append(robot)
            on_stack.add(robot)
            for peer in graph[robot]:
                if peer not in indices:
                    visit(peer)
                    lowlinks[robot] = min(lowlinks[robot], lowlinks[peer])
                elif peer in on_stack:
                    lowlinks[robot] = min(lowlinks[robot], indices[peer])
            if lowlinks[robot] != indices[robot]:
                return
            component: list[str] = []
            while True:
                peer = stack.pop()
                on_stack.remove(peer)
                component.append(peer)
                if peer == robot:
                    break
            members = tuple(sorted(component))
            if len(members) > 1 or robot in graph[robot]:
                cycles.append(members)

        for robot in sorted(graph):
            if robot not in indices:
                visit(robot)
        return min(cycles) if cycles else None

    def snapshot(self) -> dict[str, Any]:
        wait_cycle = self.wait_cycle()
        return {"policy": "atomic_route_leg_resources", "window_policy": "rolling_atomic_next_span",
                "queue_nodes": sorted(self.queue_nodes),
                "holdable_nodes": (None if self.holdable_nodes is None
                                   else sorted(self.holdable_nodes)),
                "safe_exit_nodes": dict(sorted(self.safe_exit_nodes.items())),
                "owners": dict(sorted(self.owners.items())),
                "waiters": {robot: list(peers) for robot, peers in sorted(self.waiters.items())},
                "wait_cycle": list(wait_cycle) if wait_cycle else None,
                "protected_route_tile_counts": {
                    robot: len(tiles) for robot, tiles in sorted(self._protected_tiles.items())
                },
                "handover_protected": sorted(self._handover_protected),
                "scope": "next atomic corridor span or bounded local segment; tile reservations still required"}
