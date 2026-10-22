"""Causal feature encoder. Standard library only; identical in training and runtime.

Coordinates in frames are grid coordinates, cell_m converts them to metres.
No map truth, future measurements or training labels are read by this module.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math
from ideal_sim.contract import LINKS, ENDPOINTS, route_for, route_position, valid_observation


SCHEMA = "relay-link-v2"


@dataclass(frozen=True)
class FeatureSpec:
    schema_version: str = SCHEMA
    history_steps: int = 20
    dt_s: float = 0.1
    horizons_s: tuple = (1.5, 6.0)
    weak_dbm: float = -75.0
    max_age_s: float = 0.5
    coordinate_scale_m: float = 10.0

    def __post_init__(self):
        object.__setattr__(self, "horizons_s", tuple(self.horizons_s))
        if self.schema_version != SCHEMA or self.history_steps != 20:
            raise ValueError("Expected relay-link-v2 with 20 history slots")
        values = (self.dt_s, self.max_age_s, self.coordinate_scale_m, *self.horizons_s)
        if len(self.horizons_s) != 2 or not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Invalid feature timing/scales")
        if self.horizons_s[0] >= self.horizons_s[1] or not math.isfinite(self.weak_dbm):
            raise ValueError("Expected short < deployment horizon and a finite weak threshold")
        if any(abs(h / self.dt_s - round(h / self.dt_s)) > 1e-7 for h in self.horizons_s):
            raise ValueError("Horizons must be integer multiples of dt_s")

    @property
    def names(self):
        names = [f"history_{i}_{field}" for i in range(self.history_steps)
                 for field in ("rssi", "valid", "age")]
        names += [f"link_{link}" for link in LINKS]
        names += [f"{when}_{endpoint}_{axis}" for when in ("now", "short", "deploy")
                  for endpoint in ("a", "b") for axis in ("x", "y")]
        names += ["route_segment", "route_progress", "direction", "main_speed"]
        names += [f"r{i}_{field}" for i in (1, 2)
                  for field in ("x", "y", "target_x", "target_y", "ready", "eta", "battery")]
        return names

    @property
    def input_size(self):
        return len(self.names)

    def manifest(self):
        return {**asdict(self), "input_size": self.input_size, "feature_names": self.names,
                "link_order": LINKS, "output_names": ["weak_short_logit", "weak_deploy_logit"],
                "normalization": "fixed-v2: RSSI=(dBm+75)/25; invalid=0; xy metres/10; "
                                 "age/max_age clipped; progress/23; segment/8; speed/2; eta/6",
                "link_symmetry": "explicit symmetric simulator assumption; validate on hardware"}


def move_on_grid(point, target, distance):
    """Horizontal then vertical, matching the simulation's legal grid movement."""
    coords = list(point)
    for axis in (0, 1):
        amount = min(abs(target[axis] - coords[axis]), max(0.0, distance))
        coords[axis] += math.copysign(amount, target[axis] - coords[axis])
        distance -= amount
    return tuple(coords)


def planned_nodes(state, horizon):
    cell = state["cell_m"]
    main = tuple(state["main_position"])
    if state["phase"] == "FLY":
        main, _ = route_position(state["direction"], state["progress_cells"]
                                 + state["main_speed_mps"] * horizon / cell)
    relays = []
    for i, point in enumerate(state["relay_positions"]):
        # The advertised ETA is an estimate, not a future true arrival.
        eta = max(state["relay_eta_s"][i], 1e-9)
        target = state["relay_targets"][i]
        length = sum(abs(a - b) for a, b in zip(point, target))
        relays.append(move_on_grid(point, target, length * min(horizon / eta, 1.)))
    return ((0., 0.), *relays, main)


class HistoryBuffer:
    def __init__(self, spec=None):
        self.spec = spec or FeatureSpec()
        self.rows = deque(maxlen=self.spec.history_steps)
        self.last_time = None

    def push(self, now, measurements):
        if not math.isfinite(now) or (self.last_time is not None and now <= self.last_time):
            raise ValueError("Frame timestamps must increase")
        by_link = {m["link_id"]: m for m in measurements}
        if len(by_link) != len(measurements) or not set(by_link) <= set(LINKS):
            raise ValueError("Duplicate or unknown link identifiers")
        # Preserve missing slots; do not compress a gap into contiguous history.
        if self.last_time is not None:
            gap = (now - self.last_time) / self.spec.dt_s
            if abs(gap - round(gap)) > 1e-5:
                raise ValueError("Resample telemetry onto the configured dt_s grid first")
            for _ in range(min(self.spec.history_steps, max(0, round(gap) - 1))):
                self.rows.append({})
        row = {}
        for link in LINKS:
            m = by_link.get(link, {})
            value, stamp = m.get("rssi_dbm"), m.get("sample_time_s")
            valid = valid_observation(m, now, self.spec.max_age_s)
            row[link] = (float(value), float(stamp)) if valid else None
        self.rows.append(row)
        self.last_time = now

    def encode(self, state, link):
        if link not in LINKS or self.last_time is None:
            raise ValueError("Push observations before encoding a valid link")
        spec, now = self.spec, self.last_time
        x = []
        padding = spec.history_steps - len(self.rows)
        for row in [None] * padding + list(self.rows):
            item = row.get(link) if row else None
            # Old valid samples remain history; age expresses their distance from now.
            x.extend(((item[0] + 75.) / 25., 1., min((now - item[1]) / spec.max_age_s, 1.))
                     if item else (0., 0., 1.))
        x.extend(float(link == name) for name in LINKS)
        a, b = ENDPOINTS[LINKS.index(link)]
        current = ((0., 0.), *state["relay_positions"], state["main_position"])
        nodes_by_time = (current, *(planned_nodes(state, h) for h in spec.horizons_s))
        scale = state["cell_m"] / spec.coordinate_scale_m
        for nodes in nodes_by_time:
            x.extend(float(v) * scale for node in (nodes[a], nodes[b]) for v in node)
        _, segment = route_position(state["direction"], state["progress_cells"])
        x.extend((segment / 8., state["progress_cells"] / 23.,
                  1. if state["direction"] == "outbound" else -1., state["main_speed_mps"] / 2.))
        for i in range(2):
            x.extend(float(v) * scale for p in (state["relay_positions"][i], state["relay_targets"][i]) for v in p)
            x.extend((float(state["relay_ready"][i]), state["relay_eta_s"][i] / 6.,
                      state["relay_battery"][i]))
        if len(x) != spec.input_size or not all(math.isfinite(v) for v in x):
            raise ValueError("Invalid/nonfinite feature state")
        return x

    def latest_valid(self, link):
        return bool(self.rows and self.rows[-1].get(link))
