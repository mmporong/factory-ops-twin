"""Sensor-only, single-tote roller coupon controller; no SDK or inventory state.

All dimensions/drive bounds are authored assumptions. Native observations must
be supplied by the caller; this module neither generates them nor moves bodies.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
import re
from typing import Any, Mapping

from factory_ops.carrier_module import build_manifest, manifest_sha256


def number(value: Any, name: str, low: float, high: float) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(name + " outside finite bounds")
    return float(value)


def integer(value: Any, name: str, low: int, high: int) -> int:
    number(value, name, low, high)
    if not isinstance(value, int):
        raise ValueError(name + " must be an integer")
    return value


def motor_degrees(rad_s: float, maximum_rad_s: float) -> float:
    """USD angular targetVelocity is degrees/s; maxForce remains N m in SI."""
    limit = number(maximum_rad_s, "motor limit", .01, 20.)
    return math.degrees(number(rad_s, "motor target", -limit, limit))


def build_transfer_manifest(profile: Mapping[str, Any]) -> dict:
    p = deepcopy(dict(profile))
    if (p["schema_version"] != 1 or isinstance(p["schema_version"], bool)
            or p["profile_id"] != "roller-transfer-v1"):
        raise ValueError("unsupported roller transfer profile")
    if p["units"] != {"length": "m", "mass": "kg", "speed": "rad/s", "torque": "N*m"}:
        raise ValueError("SI input units required; USD conversion belongs at drive boundary")
    tote_id = p["tote_id"]
    if not isinstance(tote_id, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,47}", tote_id):
        raise ValueError("invalid tote identity")
    carrier = build_manifest(p["carrier_profile"])
    props = carrier["mass_properties"]["tote"]
    if abs(props["mass_kg"]-6.) > 1e-10:
        raise ValueError("fixture admits exactly the authored 6 kg loaded tote")
    if any(not math.isfinite(v) or v <= 0 for v in
           (props["inertia_matrix_kg_m2"][i][i] for i in range(3))):
        raise ValueError("invalid tote inertia")
    r, c = p["rollers"], p["control"]
    radius = number(r["radius_m"], "roller radius", .015, .06)
    length = number(r["length_m"], "roller length", .36, .44)
    pitch = number(r["pitch_m"], "roller pitch", 2*radius+.003, .12)
    count = integer(r["count_per_zone"], "roller count", 4, 12)
    mass = number(r["mass_kg"], "roller mass", .05, 2.)
    speed = number(r["speed_rad_s"], "roller speed", .1, 12.)
    number(r["maximum_rad_s"], "maximum speed", speed, 15.)
    number(r["torque_limit_n_m"], "drive torque limit", .01, 2.)
    number(r["usd_damping_n_m_per_deg_s"], "USD drive damping", .0001, 1.)
    number(c["stopped_rad_s"], "stopped roller threshold", .001, min(.5, speed/4))
    dt = number(c["physics_dt_s"], "physics dt", 1/120, 1/120)
    number(c["sensor_freshness_s"], "sensor freshness", dt, .025)
    number(c["minimum_force_n"], "contact threshold", .01, 2.)
    dwell = number(c["contact_dwell_s"], "contact dwell", .15, 1.)
    timeout = number(c["stage_timeout_s"], "stage timeout", 2., 8.)
    total = number(c["maximum_sim_time_s"], "simulation bound", 2*timeout+2*dwell, 20.)
    number(c["maximum_wall_time_s"], "wall bound", 30., 360.)
    number(c["observation_hz"], "observation rate", 1., 20.)
    number(c["stationary_linear_m_s"], "evaluation linear threshold", .001, .03)
    number(c["stationary_angular_rad_s"], "evaluation angular threshold", .001, .05)
    if c["image_size_px"] != [640, 480]:
        raise ValueError("only sparse 640x480 RGB admitted")
    g = p["geometry"]
    zone_length = number(g["zone_length_m"], "zone length", .5, .8)
    gap = number(g["transfer_gap_m"], "transfer gap", .005, .04)
    top = number(g["roller_top_z_m"], "roller top", .3, 1.)
    initial_gap = number(g["initial_vertical_gap_m"], "initial gap", .001, .01)
    rail_inner = number(g["rail_inner_y_m"], "rail inner", .17, .23)
    stopper_thickness = number(g["stopper_thickness_m"], "stopper thickness", .01, .04)
    stopper_height = number(g["stopper_height_m"], "stopper height", .06, .15)
    tote_size = p["carrier_profile"]["tote"]["outer_size_m"]
    if (zone_length < tote_size[0]+.08 or length < tote_size[1]+.02
            or not tote_size[1]/2+.01 <= rail_inner < length/2
            or (count-1)*pitch+2*radius > zone_length
            or tote_size[0] < 3*pitch+gap or radius*speed > .3):
        raise ValueError("roller/tote/deck/gap fit or surface speed invalid")
    # The two roller banks share a level plane; the original solid carrier deck
    # is NOT instantiated underneath the moving tote.
    centers = {"source": -zone_length/2, "carrier": gap+zone_length/2}
    rollers = []
    for zone, center in centers.items():
        for i in range(count):
            actor = "%s_roller_%02d" % (zone, i)
            rollers.append({"actor_id": actor, "zone": zone,
                "center_m": [center+(i-(count-1)/2)*pitch, 0., top-radius],
                "radius_m": radius, "length_m": length, "mass_kg": mass,
                "diagonal_inertia_kg_m2": [mass*(3*radius**2+length**2)/12,
                    mass*radius**2/2, mass*(3*radius**2+length**2)/12]})
    end_inner = {"source": -zone_length, "carrier": gap+zone_length}
    stoppers = [{"actor_id": zone+"_stop", "zone": zone,
        "center_m": [x+(-1 if zone == "source" else 1)*stopper_thickness/2,
                     0., top+stopper_height/2],
        "size_m": [stopper_thickness, length, stopper_height]} for zone, x in end_inner.items()]
    # No direction controller receives these evaluation targets.
    initial = [-zone_length+tote_size[0]/2+.015, 0., top+initial_gap]
    actor_ids = [r["actor_id"] for r in rollers]+[s["actor_id"] for s in stoppers]+["rail_m", "rail_p", tote_id]
    if len(set(actor_ids)) != len(actor_ids):
        raise ValueError("actor identity collision")
    return {"schema_version": 1, "profile_id": p["profile_id"], "tote_id": tote_id,
        "carrier_manifest_sha256": manifest_sha256(carrier),
        "tote_geometry": [b for b in carrier["geometry"] if b["body"] == "tote"],
        "tote_mass_properties": props, "tote_outer_size_m": tote_size,
        "contact": carrier["contact"], "rollers": rollers, "stoppers": stoppers,
        "actor_ids": actor_ids, "initial_tote_origin_m": initial, "geometry": g,
        "drive": r, "control": c, "units": p["units"],
        "maximum_sim_time_s": total,
        "scope": "fixed roller/stopper coupon; free dynamic loaded tote; no mounted AMR or inventory integration",
        "native_verified": False, "authored_assumptions": p["assumptions"]}


class TransferController:
    """Fail-closed FSM. update takes sensor/motor data ONLY, never body pose.

    Sensors: every roller and both fixed stoppers, each with time_s, frame,
    force_n, in_contact and actor_pairs. Motors: measured rad/s keyed by roller.
    stoppers_fixed is the caller's collider/kinematic configuration readback.
    Events are fixture facts, not a ledger or change of tote ownership.
    """

    def __init__(self, manifest: Mapping[str, Any], run_id: str):
        if not re.fullmatch(r"[0-9a-f]{32}", run_id):
            raise ValueError("32 lowercase hex run id required")
        self.manifest = deepcopy(dict(manifest))
        self.cfg = self.manifest["control"]
        self.run_id = run_id
        self.phase = "source_ready"
        self.ready = False
        self.failure: str | None = None
        self.events: list[dict] = []
        self.command_id: str | None = None
        self._commands: dict[str, str] = {}
        self._time = -1.
        self._frame = -1
        self._phase_since: float | None = None
        self._dwell_since: float | None = None
        self._sensor_clock: dict[str, tuple[float, int]] = {}
        self._rollers = {r["actor_id"]: r["zone"] for r in manifest["rollers"]}
        self._sensor_ids = set(self._rollers) | {"source_stop", "carrier_stop"}

    def fail(self, reason: str) -> dict:
        if self.failure is None:
            self.failure = reason
        self.phase, self.ready = "fault", False
        return self.output()

    def request(self, action: str, command_id: str) -> dict:
        if (action not in ("load", "unload") or not isinstance(command_id, str)
                or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", command_id)):
            raise ValueError("invalid transfer command")
        if command_id in self._commands:
            if self._commands[command_id] != action:
                raise ValueError("command identity reused for different action")
            return self.output()
        expected = "source_ready" if action == "load" else "loaded"
        if self.phase != expected or not self.ready or (action == "load" and self._commands):
            raise ValueError("transfer not sensor-ready for command")
        self._commands[command_id] = action
        self.command_id = command_id
        self.phase = "loading" if action == "load" else "unloading"
        self.ready = False
        self._phase_since = self._time
        self._dwell_since = None
        return self.output()

    def output(self) -> dict:
        direction = 1 if self.phase == "loading" else -1 if self.phase == "unloading" else 0
        target = direction*self.manifest["drive"]["speed_rad_s"]
        return {"phase": self.phase, "ready": self.ready, "failure": self.failure,
            "command_id": self.command_id, "tote_id": self.manifest["tote_id"],
            "motor_targets_rad_s": {actor: target for actor in self._rollers},
            "events": deepcopy(self.events)}

    def update(self, *, now_s: float, frame: int, sensors: Mapping[str, Any],
               motor_rad_s: Mapping[str, Any], stoppers_fixed: Mapping[str, bool]) -> dict:
        if self.phase == "fault":
            return self.output()
        try:
            self._update(now_s, frame, sensors, motor_rad_s, stoppers_fixed)
        except (KeyError, TypeError, ValueError) as error:
            return self.fail(str(error))
        return self.output()

    def _update(self, now_s, frame, sensors, motor_rad_s, stoppers_fixed):
        now = number(now_s, "World time", 0., 1e6)
        f = integer(frame, "World frame", 0, 10000000)
        if now <= self._time or f <= self._frame:
            raise ValueError("World clock/frame did not progress")
        if set(sensors) != self._sensor_ids or set(motor_rad_s) != set(self._rollers):
            raise ValueError("missing or unknown sensor/motor identity")
        if stoppers_fixed != {"source_stop": True, "carrier_stop": True} or any(
                type(v) is not bool for v in stoppers_fixed.values()):
            raise ValueError("stopper fixed/collision state unavailable")
        touching = set()
        tote = self.manifest["tote_id"]
        for key, sensor in sensors.items():
            stamp = number(sensor["time_s"], "sensor time", 0., now)
            sf = integer(sensor["frame"], "sensor frame", 0, f)
            last = self._sensor_clock.get(key, (-1., -1))
            if stamp < last[0] or sf < last[1]:
                raise ValueError("contact sensor clock regressed: "+key)
            if now-stamp > self.cfg["sensor_freshness_s"]:
                raise ValueError("stale contact sensor: "+key)
            force = number(sensor["force_n"], "sensor force", 0., 1e6)
            if type(sensor["in_contact"]) is not bool or not isinstance(sensor["actor_pairs"], list):
                raise ValueError("invalid contact metadata")
            pairs = sensor["actor_pairs"]
            for pair in pairs:
                if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                        or set(pair) != {key, tote}):
                    raise ValueError("unknown or wrongly attributed contact actor: "+key)
            if sensor["in_contact"] and force >= self.cfg["minimum_force_n"] and pairs:
                touching.add(key)
            self._sensor_clock[key] = (stamp, sf)
        stopped = all(abs(number(v, "native roller speed", -2*self.manifest["drive"]["maximum_rad_s"],
            2*self.manifest["drive"]["maximum_rad_s"])) <= self.cfg["stopped_rad_s"] for v in motor_rad_s.values())
        # all() short-circuits, so validate every value even when one is moving.
        for v in motor_rad_s.values():
            number(v, "native roller speed", -2*self.manifest["drive"]["maximum_rad_s"],
                   2*self.manifest["drive"]["maximum_rad_s"])
        self._time, self._frame = now, f
        if self._phase_since is None:
            self._phase_since = now
        if self.phase != "returned" and now-self._phase_since > self.cfg["stage_timeout_s"]:
            raise ValueError("transfer stage timeout: "+self.phase)
        zone = "source" if self.phase in ("source_ready", "unloading", "return_settling", "returned") else "carrier"
        other = "carrier" if zone == "source" else "source"
        support = sum(a in touching for a, z in self._rollers.items() if z == zone)
        wrong_support = any(a in touching for a, z in self._rollers.items() if z == other)
        if self.phase in ("loading", "unloading") and zone+"_stop" in touching:
            self.phase = "load_settling" if self.phase == "loading" else "return_settling"
            self._dwell_since = None
        qualified = support >= 2 and not wrong_support and stopped
        if self.phase in ("source_ready", "load_settling", "return_settling"):
            self._dwell_since = (now if self._dwell_since is None else self._dwell_since) if qualified else None
            if self._dwell_since is not None and now-self._dwell_since >= self.cfg["contact_dwell_s"]:
                if self.phase != "source_ready":
                    self.phase = "loaded" if self.phase == "load_settling" else "returned"
                    self.events.append({"event": "transfer_"+self.phase, "run_id": self.run_id,
                        "tote_id": tote, "command_id": self.command_id,
                        "world_time_s": now, "world_physics_frame": f,
                        "source": "native_contact_and_stopped_rollers",
                        "scope": "fixture confirmation; no inventory/ownership mutation"})
                self.ready = True
                self._phase_since = now
        elif self.phase in ("loaded", "returned"):
            if not qualified:
                raise ValueError("confirmed tote support or roller stop lost")
