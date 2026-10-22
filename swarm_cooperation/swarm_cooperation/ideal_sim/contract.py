"""A/B/C simulation contract. Grid coordinates; SI distances and monotonic seconds.

All geometry and radio constants come from ``mission_profile.json`` through
``ideal_sim.profile``.  Do not duplicate these values in A/B/C.
"""
import math

from .profile import PROFILE, PROFILE_SHA256

VERSION = PROFILE["contract_version"]
PROFILE_VERSION = PROFILE["schema_version"]

_MAP = PROFILE["map"]
_TIMING = PROFILE["timing"]
_RADIO = PROFILE["radio"]

CELL_M = float(_MAP["cell_m"])
DT_S = float(_TIMING["dt_s"])
REFERENCE_DISTANCE_M = float(_RADIO["reference_distance_m"])
TERMINAL_DWELL_S = float(_TIMING["terminal_dwell_s"])
HOME_DWELL_S = float(_TIMING["home_dwell_s"])
OUTWARD = tuple(tuple(float(v) for v in point) for point in _MAP["route"])
OCCUPIED = frozenset(tuple(int(v) for v in cell) for cell in _MAP["occupied_cells"])
LINKS = tuple(item["id"] for item in _RADIO["links"])
ENDPOINTS = tuple((int(item["a"]), int(item["b"])) for item in _RADIO["links"])
ROUTE_LINKS = {tuple(int(v) for v in item["route"]): tuple(item["links"])
               for item in PROFILE["topology"]["paths"]}

if len(LINKS) != 6 or len(ENDPOINTS) != 6:
    raise ValueError("mission profile must define six physical links")
if OUTWARD[0] != (0.0, 0.0) or OUTWARD[-1] != (3.0, 4.0):
    raise ValueError("mission profile route endpoints are inconsistent")


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_observation(measurement, now, max_age_s):
    """One fail-closed gate used by online inputs and offline label ingestion."""
    stamp = measurement.get("sample_time_s")
    return (measurement.get("valid") is True and finite_number(now)
            and finite_number(measurement.get("rssi_dbm")) and finite_number(stamp)
            and 0 <= now - stamp <= max_age_s + 1e-9)


def route_for(direction):
    if direction not in ("outbound", "return"):
        raise ValueError("direction must be outbound or return")
    return OUTWARD if direction == "outbound" else tuple(reversed(OUTWARD))


def route_position(direction, progress_cells):
    if not finite_number(progress_cells):
        raise ValueError("route progress must be finite")
    remaining = max(0., progress_cells)
    route = route_for(direction)
    for index, (a, b) in enumerate(zip(route, route[1:])):
        distance = math.dist(a, b)
        if remaining <= distance + 1e-9:
            fraction = min(1., remaining / distance)
            return tuple(x + (y - x) * fraction for x, y in zip(a, b)), index
        remaining -= distance
    return route[-1], len(route) - 2


def manifest(cfg):
    single_leg_m = sum(math.dist(a, b) for a, b in zip(OUTWARD, OUTWARD[1:])) * cfg.cell_m
    return dict(version=VERSION, profile_version=PROFILE_VERSION, profile_sha256=PROFILE_SHA256,
                cell_m=cfg.cell_m, dt_s=cfg.dt_s,
                terminal_dwell_s=cfg.terminal_dwell_s, home_dwell_s=cfg.home_dwell_s,
                outward_route=OUTWARD, single_leg_m=single_leg_m,
                coordinates="grid; multiply by cell_m for metres",
                progress="distance from current leg start; reset only at leg change; frozen during waits",
                segment="zero-based segment in direction-specific route; vertex belongs to incoming segment",
                time="seconds since run start, monotonic; post-step samples; sample_time_s is acquisition time",
                dwell="at terminal after outbound and home after return, including final round",
                radio=dict(p0_dbm=cfg.p0_dbm, path_loss_n=cfg.path_loss_n,
                           shelf_loss_db=cfg.shelf_loss_db,
                           reference_distance_m=REFERENCE_DISTANCE_M,
                           gain_db=0., occlusion="each occupied cell open interior once; boundaries/corners excluded",
                           topology="six symmetric physical links; five simple paths; weakest hop; no gain"))
