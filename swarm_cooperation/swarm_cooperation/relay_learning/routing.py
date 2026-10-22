"""CPU route transactions; measurements/ACKs are explicit inputs."""
from dataclasses import dataclass
import math

from ideal_sim.contract import ROUTE_LINKS, finite_number


def path_quality(route, measured, ready):
    if route not in ROUTE_LINKS or any(not ready[i - 1] for i in route):
        return -math.inf
    values = [measured.get(link) for link in ROUTE_LINKS[route]]
    return min(values) if all(finite_number(v) for v in values) else -math.inf


@dataclass
class PendingRoute:
    epoch: int
    route: tuple
    prepared_at: float
    commit_at: float | None = None


class RouteController:
    def __init__(self, service=-75., disconnect=-88., hysteresis=3., dwell=1.,
                 probe_s=.2, ack_s=.3, timeout_s=2., on_slots=3):
        self.service, self.disconnect, self.hysteresis = service, disconnect, hysteresis
        self.dwell, self.probe_s, self.ack_s, self.timeout_s = dwell, probe_s, ack_s, timeout_s
        self.on_slots = max(1, int(on_slots))
        self.active = ()
        self.pending = None
        self.epoch = 0
        self.last_switch = -math.inf
        self.events = []
        self.weak_run = 0
        self.strong_run = 0

    def propose(self, now, measured, ready, risks=None, threshold=.5, forbidden=()):
        values = {r: path_quality(r, measured, ready) for r in ROUTE_LINKS
                  if not set(r).intersection(forbidden)}
        feasible = [r for r, q in values.items() if q >= self.disconnect]
        if not feasible:
            return None  # Explicit degraded result; retain old route, never claim feasibility.

        def risk(route):
            # Conservative score, NOT a calibrated joint probability.
            return min(1., sum(risks.get(link, 1.) for link in ROUTE_LINKS[route])) if risks else 0.

        best = min(feasible, key=lambda r: (values[r] < self.service, risk(r) >= threshold,
                                           len(r), risk(r), -values[r], r))
        current = values.get(self.active, -math.inf)
        if current < self.disconnect:
            # Safety overrides dwell and the passive N-slot counter.
            self.weak_run = self.on_slots
            self.strong_run = 0
            return best
        if current < self.service:
            self.weak_run += 1
            self.strong_run = 0
        else:
            self.weak_run = 0
        if now - self.last_switch < self.dwell:
            return self.active
        if best == self.active:
            return best
        proactive = risks and risk(self.active) >= threshold and risk(best) < threshold
        if self.active and best == ():
            # Passive recovery needs N consecutive slots above -72 dBm (service+3).
            self.strong_run = self.strong_run + 1 if values[()] >= self.service + self.hysteresis else 0
            if self.strong_run < self.on_slots:
                return self.active
            return best if values[best] >= self.service + self.hysteresis and not (
                risks and risk(best) >= threshold) else self.active
        self.strong_run = 0
        if proactive:
            return best
        if current < self.service:
            # Passive trigger: N consecutive weak slots plus the 3 dB improvement guard.
            if self.weak_run < self.on_slots:
                return self.active
            return best if values[best] >= current + self.hysteresis else self.active
        if len(best) <= len(self.active) and values[best] >= current + self.hysteresis:
            return best
        return self.active

    def request(self, route, now):
        if route is None or self.pending or tuple(route) == self.active:
            return
        if tuple(route) not in ROUTE_LINKS:
            raise ValueError("Unknown route")
        self.epoch += 1
        self.pending = PendingRoute(self.epoch, tuple(route), now)
        self.events.append(dict(kind="route_prepare", time_s=now, epoch=self.epoch, route=list(route)))

    def advance(self, now, measured, ready):
        pending = self.pending
        if pending is None:
            return
        if now - pending.prepared_at >= self.timeout_s:
            self.events.append(dict(kind="route_timeout", time_s=now, epoch=pending.epoch))
            self.pending = None
            return
        if path_quality(pending.route, measured, ready) < self.disconnect:
            # A failed probe invalidates this attempt; old route remains in service.
            self.events.append(dict(kind="route_probe_failed", time_s=now, epoch=pending.epoch))
            self.pending = None
            return
        if pending.commit_at is None and now - pending.prepared_at + 1e-9 >= self.probe_s:
            pending.commit_at = now
            self.events.append(dict(kind="route_commit", time_s=now, epoch=pending.epoch))

    def acknowledge(self, epoch, now, measured, ready):
        p = self.pending
        if (p is None or epoch != p.epoch or p.commit_at is None
                or now - p.prepared_at >= self.timeout_s
                or now - p.commit_at + 1e-9 < self.ack_s
                or path_quality(p.route, measured, ready) < self.disconnect):
            return False
        old = self.active
        self.active, self.last_switch, self.pending = p.route, now, None
        self.events.append(dict(kind="route_confirmed", time_s=now, epoch=epoch,
                                old_route=list(old), route=list(self.active)))
        return True
