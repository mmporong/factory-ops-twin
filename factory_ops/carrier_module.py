"""Authored carrier geometry in metres/kg; no SDK imports or robot mutation.

Clearance checks use declared candidate-frame keepouts, not measured Nova Carter
geometry. A native robot admission must supply its actual geometry in this frame.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


def _number(value: Any, name: str, minimum: float = 0., *, positive: bool = True) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < minimum or (positive and value == minimum)):
        raise ValueError(name + " must be finite and within its positive bounds")
    return float(value)


def _xyz(value: Any, name: str, *, size: bool = False) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(name + " must have three coordinates")
    return [_number(v, name) if size else _number(v, name, -1e6, positive=False) for v in value]


def _box(name: str, body: str, size: Sequence[float], center: Sequence[float], mass: float) -> dict:
    return {"name": name, "body": body, "size_m": list(size), "center_m": list(center), "mass_kg": mass}


def _bounds(box: Mapping[str, Any]) -> tuple[list[float], list[float]]:
    return ([c-s/2 for c, s in zip(box["center_m"], box["size_m"])],
            [c+s/2 for c, s in zip(box["center_m"], box["size_m"])])


def mass_properties(boxes: Sequence[Mapping[str, Any]]) -> dict:
    """Uniform cuboid components, combined about their centre of mass (SI)."""
    total = math.fsum(b["mass_kg"] for b in boxes)
    _number(total, "assembly mass")
    com = [math.fsum(b["mass_kg"]*b["center_m"][i] for b in boxes)/total for i in range(3)]
    inertia = [[0.]*3 for _ in range(3)]
    for b in boxes:
        m, size = b["mass_kg"], b["size_m"]
        delta = [b["center_m"][i]-com[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                inertia[i][j] += (m*(sum(size[k]**2 for k in range(3) if k != i)/12
                                      + sum(delta[k]**2 for k in range(3) if k != i)) if i == j
                                  else -m*delta[i]*delta[j])
    return {"mass_kg": total, "center_of_mass_m": com, "inertia_matrix_kg_m2": inertia,
            "method": "uniform cuboids plus parallel-axis theorem; authored, not measured"}


def validate_clearance(boxes: Sequence[Mapping[str, Any]], keepouts: Sequence[Mapping[str, Any]],
                       radius_m: float, clearance_m: float) -> None:
    """Reject declared intersections/overhang; all inputs must share one frame."""
    radius = _number(radius_m, "existing footprint radius")
    gap = _number(clearance_m, "clearance")
    for b in boxes:
        _xyz(b["center_m"], "box center")
        _xyz(b["size_m"], "box size", size=True)
        lo, hi = _bounds(b)
        if max(math.hypot(x, y) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])) > radius:
            raise ValueError("geometry exceeds unchanged footprint: " + b["name"])
        for forbidden in keepouts:
            _xyz(forbidden["center_m"], "keepout center")
            _xyz(forbidden["size_m"], "keepout size", size=True)
            low, high = _bounds(forbidden)
            if all(hi[i] > low[i]-gap+1e-12 and lo[i] < high[i]+gap-1e-12 for i in range(3)):
                raise ValueError(b["name"] + " overlaps keepout " + forbidden["name"])


def build_manifest(profile: Mapping[str, Any]) -> dict:
    """Validate parameters and export separate compound carrier/tote bodies."""
    p = deepcopy(dict(profile))
    if p["schema_version"] != 1 or isinstance(p["schema_version"], bool) or p["profile_id"] != "tote-carrier-v1":
        raise ValueError("unsupported carrier profile")
    deck, mount, tote, payload = (p[k] for k in ("deck", "mount", "tote", "payload"))
    d = _xyz(deck["size_m"], "deck size", size=True)
    c = _xyz(deck["center_m"], "deck center")
    post = _xyz(mount["post_size_m"], "post size", size=True)
    span = _xyz(mount["support_span_m"], "support span", size=True)
    if span[2] != post[2]:
        raise ValueError("support span z must equal post height")
    if any(span[i]+post[i] > d[i] for i in range(2)):
        raise ValueError("support feet must fit below deck")
    module = [_box("deck", "carrier", d, c, _number(deck["mass_kg"], "deck mass"))]
    for x in (-1, 1):
        for y in (-1, 1):
            module.append(_box("post_%s_%s" % ("m" if x < 0 else "p", "m" if y < 0 else "p"), "carrier", post,
                [c[0]+x*span[0]/2, c[1]+y*span[1]/2, c[2]-d[2]/2-post[2]/2],
                _number(mount["post_mass_kg"], "post mass")))
    outer = _xyz(tote["outer_size_m"], "tote size", size=True)
    wall = _number(tote["wall_m"], "tote wall")
    margin = _number(mount["fit_margin_m"], "fit margin")
    if any(2*wall >= v for v in outer) or any(outer[i]+2*margin > d[i] for i in range(2)):
        raise ValueError("tote wall or tote/deck fit is invalid")
    if any(outer[i]/2+margin > span[i]/2 for i in range(2)):
        raise ValueError("support polygon must enclose tote and centred payload")
    parts = [([outer[0], outer[1], wall], [0., 0., wall/2])]
    for sign in (-1, 1):
        parts.extend([([wall, outer[1], outer[2]-wall],
                       [sign*(outer[0]-wall)/2, 0., (outer[2]+wall)/2]),
                      ([outer[0]-2*wall, wall, outer[2]-wall],
                       [0., sign*(outer[1]-wall)/2, (outer[2]+wall)/2])])
    empty_mass = _number(tote["empty_mass_kg"], "empty tote mass")
    volume = math.fsum(math.prod(s) for s, _ in parts)
    cargo = [_box("tote_%d" % i, "tote", s, v, empty_mass*math.prod(s)/volume)
             for i, (s, v) in enumerate(parts)]
    payload_size = _xyz(payload["size_m"], "payload size", size=True)
    load = _number(payload["mass_kg"], "payload mass", positive=False)
    maximum = _number(payload["maximum_mass_kg"], "authored payload bound")
    if load > maximum or any(payload_size[i]+2*margin > outer[i]-2*wall for i in range(2)) or payload_size[2] > outer[2]-wall:
        raise ValueError("payload exceeds authored mass/inner tote bounds")
    if load:
        cargo.append(_box("rigid_ballast", "tote", payload_size, [0., 0., wall+payload_size[2]/2], load))
    contact = p["contact"]
    for key in ("static_friction", "dynamic_friction", "contact_offset_m"):
        _number(contact[key], key)
    if contact["dynamic_friction"] > contact["static_friction"]:
        raise ValueError("dynamic friction exceeds static friction")
    restitution = _number(contact["restitution"], "restitution", positive=False)
    rest = _number(contact["rest_offset_m"], "rest offset", positive=False)
    if restitution > 1 or rest >= contact["contact_offset_m"] or 2*contact["contact_offset_m"] >= wall:
        raise ValueError("contact offsets/restitution exceed authored bounds")
    admission = p["compatibility"]
    top = c[2]+d[2]/2
    placed_cargo = [dict(b, center_m=[c[0]+b["center_m"][0], c[1]+b["center_m"][1], top+b["center_m"][2]]) for b in cargo]
    validate_clearance(module+placed_cargo, admission["declared_keepouts"],
                       admission["unchanged_footprint_radius_m"], mount["clearance_m"])
    probe = p["probe"]
    for key in ("physics_dt_s", "maximum_sim_time_s", "maximum_wall_time_s", "initial_gap_m",
                "contact_dwell_s", "separation_dwell_s", "release_velocity_z_m_s",
                "minimum_force_n", "stationary_linear_m_s", "stationary_angular_rad_s",
                "sensor_freshness_s", "maximum_settle_error_m"):
        _number(probe[key], key)
    if (not .001 <= probe["physics_dt_s"] <= 1/30 or not 2 <= probe["maximum_sim_time_s"] <= 20
            or not 10 <= probe["maximum_wall_time_s"] <= 420
            or probe["contact_dwell_s"]+probe["separation_dwell_s"] >= probe["maximum_sim_time_s"]
            or probe["image_size_px"] != [640, 480]):
        raise ValueError("probe must retain bounded time and 640x480 capture")
    properties = {"carrier": mass_properties(module), "tote": mass_properties(cargo)}
    return {"schema_version": 1, "profile_id": p["profile_id"], "units": {"length": "m", "mass": "kg"},
        "frame": "authored candidate mount frame: x forward, y left, z up; robot transform unqualified",
        "geometry": module+cargo, "mass_properties": properties, "contact": contact,
        "tote_rest_origin_m": [c[0], c[1], top], "mount_plane_z_m": c[2]-d[2]/2-post[2],
        "declared_clearance_passed": True, "robot_integration_verified": False,
        "canonical_map_or_footprint_changed": False, "presentation_cargo_replaced": False,
        "payload_bounds_kg": [0., maximum], "probe": probe,
        "assumptions": p["assumptions"], "compatibility": admission}


def manifest_json(manifest: Mapping[str, Any]) -> str:
    return json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n"


def load_profile(path: str | Path) -> dict:
    profile = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    build_manifest(profile)
    return profile


def manifest_sha256(manifest: Mapping[str, Any]) -> str:
    return hashlib.sha256(manifest_json(manifest).encode("utf-8")).hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    text = manifest_json(build_manifest(load_profile(args.config)))
    if args.output is None:
        print(text, end="")
    else:
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
