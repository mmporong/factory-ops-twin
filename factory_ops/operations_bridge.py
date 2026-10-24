"""Conservative route-leg admission for authored inter-bay bridges.

Admission is an additional dispatch gate, never a motion permission. Tile
reservations and the stopped-pose footprint remain authoritative for movement.
All bridges on a leg are acquired together and retained until arrival, avoiding
opposing vehicles waiting at the two gates of a single-lane connector.
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
        self.resources: dict[str, frozenset[str]] = {}
        for name, nodes in raw["resources"].items():
            if (not isinstance(name, str) or not name or not isinstance(nodes, list) or not nodes
                    or any(not isinstance(node, str) or node not in routing["nodes"] for node in nodes)):
                raise ValueError("bridge admission resource must name existing route nodes")
            self.resources[name] = frozenset(nodes)
        self.owners: dict[str, str] = {}
        self.waiters: dict[str, tuple[str, ...]] = {}
        self._protected_tiles: dict[str, frozenset[Tile]] = {}

    def required(self, route_nodes: Iterable[str]) -> frozenset[str]:
        route = frozenset(route_nodes)
        return frozenset(name for name, nodes in self.resources.items() if nodes & route)

    def held(self, robot: str) -> frozenset[str]:
        return frozenset(name for name, owner in self.owners.items() if owner == robot)

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
        already_holds = bool(self.held(robot))
        if required and protected and not already_holds and observed_tiles is not None:
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
    ) -> tuple[str, ...]:
        route_nodes = tuple(route_nodes)
        protected = frozenset(route_tiles)
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
        if required and protected:
            self._protected_tiles[robot] = protected
        self.waiters.pop(robot, None)
        return ()

    def release(self, robot: str, *, stopped: bool) -> None:
        if not stopped:
            raise RuntimeError("bridge resources cannot be released before a kinematic stop")
        for name in self.held(robot):
            del self.owners[name]
        self._protected_tiles.pop(robot, None)
        self.waiters.pop(robot, None)

    def withdraw_waiter(self, robot: str) -> bool:
        """Withdraw only an unowned admission wait.

        A queued parking intent may be replaced by business work while the
        vehicle is still stopped at its authored anchor.  An owner is already
        committed to a protected route and cannot be withdrawn through this
        narrow API.
        """
        if self.held(robot):
            return False
        return self.waiters.pop(robot, None) is not None

    def snapshot(self) -> dict[str, Any]:
        return {"policy": "atomic_route_leg_resources", "owners": dict(sorted(self.owners.items())),
                "waiters": {robot: list(peers) for robot, peers in sorted(self.waiters.items())},
                "protected_route_tile_counts": {
                    robot: len(tiles) for robot, tiles in sorted(self._protected_tiles.items())
                },
                "scope": "conservative full-leg bridge and conflict-route admission; tile reservations still required"}
