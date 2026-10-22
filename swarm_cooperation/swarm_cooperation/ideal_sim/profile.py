"""Single source of truth for the A/B/C mission profile.

The canonical file is ``mission_profile.json`` at the repository root.  Every
current module (A, B ``ideal_sim`` and C) obtains grid length, route, speeds,
dwell times, radio parameters, thresholds and station candidates through this
loader.  Set ``MISSION_PROFILE`` to use an explicit file in tests or packaging.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

FILENAME = "mission_profile.json"


def _candidate_paths() -> tuple[Path, ...]:
    here = Path(__file__).resolve()
    paths: list[Path] = []
    override = os.environ.get("MISSION_PROFILE")
    if override:
        paths.append(Path(override))
    # repo/swarm_cooperation/swarm_cooperation/ideal_sim/profile.py -> repo root
    paths.append(here.parents[3] / FILENAME)
    # Optional copy shipped beside the inner module for a standalone package.
    paths.append(here.parents[1] / FILENAME)
    paths.append(Path.cwd() / FILENAME)
    return tuple(paths)


def _finite(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"mission profile {name} must be a finite number")
    return float(value)


def _validate(profile: dict[str, Any], path: Path) -> None:
    def need(condition: bool, message: str) -> None:
        if not condition:
            raise ValueError(f"{path}: {message}")

    need(profile.get("schema_version") == "mission-profile-v1", "unsupported schema_version")
    need(isinstance(profile.get("contract_version"), str) and profile["contract_version"],
         "contract_version is required")
    for section in ("vehicle", "map", "timing", "radio", "topology", "thresholds", "mission",
                    "stations", "scheduling", "baselines", "legacy"):
        need(isinstance(profile.get(section), dict), f"missing object section: {section}")
    need(profile["vehicle"].get("kind") in ("relay_drone", "ground_car"),
         "vehicle.kind must be relay_drone or ground_car")

    mapping = profile["map"]
    need((mapping["columns"], mapping["rows"]) == (3, 4), "this route requires a 3 by 4 map")
    _finite(mapping["cell_m"], "map.cell_m")
    route = tuple(tuple(float(v) for v in point) for point in mapping["route"])
    need(len(route) >= 2, "map.route needs at least two points")
    need(route[0] == (0.0, 0.0) and route[-1] == (3.0, 4.0), "route must start at (0,0) and end at (3,4)")
    for a, b in zip(route, route[1:]):
        need(a[0] == b[0] or a[1] == b[1], f"route segment {a}->{b} is not axis aligned")
    need(sum(math.dist(a, b) for a, b in zip(route, route[1:])) == 23.0,
         "route must be the standard 23-cell outbound path")
    occupied = {tuple(int(v) for v in cell) for cell in mapping["occupied_cells"]}
    need(len(occupied) == 12, "occupied_cells must contain 12 cells")
    need(all(1 <= x <= 3 and 1 <= y <= 4 for x, y in occupied), "occupied_cells outside 1..3 x 1..4")

    timing = profile["timing"]
    for key in ("dt_s", "main_speed_mps", "relay_speed_mps", "terminal_dwell_s",
                "home_dwell_s", "takeoff_s", "settle_s", "probe_s", "route_ack_s",
                "route_timeout_s", "measurement_max_age_s", "failsafe_hold_s",
                "deployment_timeout_s"):
        _finite(timing[key], f"timing.{key}")
        need(timing[key] >= 0, f"timing.{key} must be non-negative")
    for key in ("dt_s", "main_speed_mps", "relay_speed_mps", "route_timeout_s",
                "measurement_max_age_s", "deployment_timeout_s"):
        need(timing[key] > 0, f"timing.{key} must be positive")
    need(timing["dt_s"] == 0.1, "the shared clock uses 0.1 s slots")
    need(timing["probe_s"] + timing["route_ack_s"] < timing["route_timeout_s"],
         "route_timeout_s must exceed probe + ACK")

    radio = profile["radio"]
    _finite(radio["p0_dbm"], "radio.p0_dbm")
    _finite(radio["path_loss_n"], "radio.path_loss_n")
    _finite(radio["shelf_loss_db"], "radio.shelf_loss_db")
    need(radio["shelf_loss_db"] >= 0, "radio.shelf_loss_db must be non-negative")
    need(_finite(radio["gain_db"], "radio.gain_db") == 0.0,
         "radio.gain_db must be 0: the +6 dB assumption is retired")
    links = radio["links"]
    need(len(links) == 6, "radio.links must contain exactly six physical hops")
    ids = [item["id"] for item in links]
    need(len(set(ids)) == 6, "radio link ids must be unique")
    need(all(0 <= item["a"] <= 3 and 0 <= item["b"] <= 3 for item in links),
         "radio link endpoints must be node indexes 0..3")

    paths = profile["topology"]["paths"]
    need(len(paths) == 5, "topology.paths must contain five simple routes")
    seen = set()
    for item in paths:
        route_key = tuple(int(v) for v in item["route"])
        need(route_key not in seen, f"duplicate topology route {route_key}")
        seen.add(route_key)
        need(len(item["links"]) == len(route_key) + 1,
             f"route {route_key} has the wrong number of links")

    thresholds = profile["thresholds"]
    _finite(thresholds["service_dbm"], "thresholds.service_dbm")
    _finite(thresholds["disconnect_dbm"], "thresholds.disconnect_dbm")
    need(thresholds["disconnect_dbm"] < thresholds["service_dbm"], "disconnect must be below service")
    need(_finite(thresholds["hysteresis_db"], "thresholds.hysteresis_db") >= 0,
         "hysteresis_db must be non-negative")

    stations = profile["stations"]
    need(len(stations["initial"]) == 2 and stations["initial"][0] != stations["initial"][1],
         "two distinct initial stations are required")
    need(len(stations["candidates"]) == 20, "stations.candidates must contain the 20 grid nodes")

    baseline = profile["baselines"]
    need(int(baseline["rounds"]) >= 1, "baselines.rounds must be positive")
    need(len(baseline["seeds"]) >= 1, "baselines.seeds cannot be empty")
    need(set(baseline["strategies"]) == {"reactive", "predeploy"},
         "baselines.strategies must be reactive and predeploy")

    legacy = profile["legacy"]
    historical = legacy.get("historical_b", {})
    need(historical.get("frozen") is True, "legacy.historical_b must be marked frozen")
    need(_finite(historical.get("cell_m"), "legacy.historical_b.cell_m") == 2.0,
         "historical B legacy must stay at 2 m/cell")
    need(_finite(historical.get("pick_s"), "legacy.historical_b.pick_s") == 3.0,
         "historical B legacy must stay at a 3 s dwell")
    compatibility = legacy.get("compatibility_b", {})
    need(compatibility.get("param_source") is False,
         "legacy.compatibility_b is a compatibility entry, not a current parameter source")
    need(_finite(compatibility.get("cell_m"), "legacy.compatibility_b.cell_m") == 1.0,
         "README compatibility B entry uses 1 m/cell")
    need(_finite(compatibility.get("pick_s"), "legacy.compatibility_b.pick_s") == 1.0,
         "README compatibility B entry uses a 1 s dwell")


def _load() -> tuple[dict[str, Any], Path, str]:
    checked = []
    for path in _candidate_paths():
        path = path.expanduser().resolve()
        if path in checked:
            continue
        checked.append(path)
        if not path.is_file():
            continue
        raw = path.read_bytes()
        data = json.loads(raw.decode("utf-8"))
        _validate(data, path)
        return data, path, hashlib.sha256(raw).hexdigest()
    locations = ", ".join(str(path) for path in checked)
    raise FileNotFoundError(f"mission_profile.json not found; checked: {locations}")


PROFILE, PROFILE_PATH, PROFILE_SHA256 = _load()
