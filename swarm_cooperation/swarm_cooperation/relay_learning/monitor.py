"""Synthetic radio adapter and shared predictor loop; truth is only for scoring/labels."""
from collections import deque
import math
import random
import time

from .features import FeatureSpec, HistoryBuffer, LINKS
from ideal_sim.contract import finite_number


class LinkMonitor:
    def __init__(self, cfg, predictor=None):
        self.cfg, self.predictor = cfg, predictor
        self.spec = FeatureSpec(dt_s=cfg.dt_s, horizons_s=(cfg.short_horizon_s, cfg.deploy_horizon_s),
                                weak_dbm=cfg.service_dbm, max_age_s=cfg.measurement_max_age_s)
        if predictor is not None and predictor.spec != self.spec:
            raise ValueError("Simulation and model feature contracts differ")
        self.buffer = HistoryBuffer(self.spec)
        self.delayed = deque()
        self.measured = {}
        self.observations = []
        self.risks = None
        self.thresholds = predictor.thresholds if predictor is not None else [.5, .5]
        self.last_prediction = -math.inf
        self.fallback_count = 0
        self.inference_calls = 0
        self.last_error = None
        self.frames = []
        self.inference_ms = []
        # Test/CI fault injection: link -> [start_s, end_s).  A dropped hop is
        # invalid, never replaced by the last good value.
        self.injected_outages = {}

    def inject_outage(self, link, start_s, end_s):
        """Drop one physical link's observations for a bounded interval."""
        if link not in LINKS:
            raise ValueError(f"unknown link {link}")
        if not (math.isfinite(start_s) and math.isfinite(end_s) and 0 <= start_s < end_s):
            raise ValueError("outage interval must be finite and increasing")
        self.injected_outages[link] = (float(start_s), float(end_s))

    def clear_outages(self):
        self.injected_outages.clear()

    def _outage_active(self, link, now):
        interval = self.injected_outages.get(link)
        return interval is not None and interval[0] <= now < interval[1]

    def update(self, now, physical, state, plan_epoch):
        cfg = self.cfg
        noisy = []
        for i, value in enumerate(physical):
            # Seeded by time/link, not PRNG consumption order: same exogenous disturbance
            # across policies; geometry is still recomputed at each policy's positions.
            rng = random.Random(f"{cfg.seed}:noise:{round(now / cfg.dt_s)}:{i}")
            phase = random.Random(f"{cfg.seed}:phase:{i}").uniform(0., 6.28)
            noise = cfg.noise_sigma_db * (.7 * math.sin(now * .8 + phase) + .3 * rng.gauss(0, 1))
            noisy.append(value + noise)
        self.delayed.append((now, noisy))
        selected = None
        while self.delayed and self.delayed[0][0] <= now - cfg.measurement_delay_s + 1e-9:
            selected = self.delayed.popleft()
        measurements = []
        for i, link in enumerate(LINKS):
            valid = (selected is not None and
                     random.Random(f"{cfg.seed}:drop:{round(now / cfg.dt_s)}:{i}").random()
                     >= cfg.measurement_drop_rate)
            value = selected[1][i] if valid else None
            if self._outage_active(link, now):
                valid, value = False, None
            measurements.append(dict(link_id=link, rssi_dbm=value,
                                     sample_time_s=selected[0] if selected else now, valid=valid))
        self.buffer.push(now, measurements)
        self.observations = [dict(m, valid=self.buffer.latest_valid(m["link_id"]),
                                  rssi_dbm=m["rssi_dbm"] if finite_number(m["rssi_dbm"]) else None)
                             for m in measurements]
        self.measured = {m["link_id"]: m["rssi_dbm"] for m in measurements
                         if self.buffer.latest_valid(m["link_id"])}
        if cfg.record_learning:
            self.frames.append(dict(schema_version=self.spec.schema_version, source="simulation",
                                    time_s=now, plan_epoch=plan_epoch, state=state, links=self.observations,
                                    label_rssi_dbm={link: v if finite_number(v) else None
                                                   for link, v in zip(LINKS, noisy)}))
        if self.predictor is not None and now - self.last_prediction + 1e-9 >= cfg.inference_period_s:
            self.last_prediction = now
            started = time.perf_counter()
            try:
                if not all(self.buffer.latest_valid(link) for link in LINKS):
                    raise ValueError("Missing/stale current observations; use CPU rules")
                rows = [self.buffer.encode(state, link) for link in LINKS]
                values = self.predictor.predict(rows)
                if (len(values) != 6 or any(len(row) != 2 for row in values)
                        or any(not math.isfinite(v) or not 0 <= v <= 1 for row in values for v in row)):
                    raise ValueError("Invalid prediction output")
                self.risks = dict(zip(LINKS, values))
                self.last_error = None
                self.inference_calls += 1
            except Exception as exc:
                self.risks = None
                self.fallback_count += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
            finally:
                self.inference_ms.append((time.perf_counter() - started) * 1000.)
        # Missing current telemetry invalidates cached model decisions immediately.
        if not all(self.buffer.latest_valid(link) for link in LINKS):
            self.risks = None
        return noisy
