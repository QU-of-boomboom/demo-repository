"""理想运动学实验：五轮固定航线、可观测地图更新、有限候选中继规划。"""
from dataclasses import asdict, replace
from collections import deque
import math
import random
import time

from .config import ExperimentConfig
from .contract import manifest, route_position
from .scheduler import (CandidateScheduler, SchedulingState, evaluate, load_model,
                        route_qualities, select_route)
from .world import WarehouseMap, at_distance, hop_rssi, length, outward_route, sample_route
from relay_learning.features import LINKS, ENDPOINTS
from relay_learning.monitor import LinkMonitor
from relay_learning.routing import RouteController, path_quality, ROUTE_LINKS


def timing_summary(values):
    if not values:
        return dict(count=0, mean_ms=None, p95_ms=None, p99_ms=None)
    ordered = sorted(values)
    return dict(count=len(values), mean_ms=sum(values) / len(values),
                p95_ms=ordered[math.ceil(.95 * len(values)) - 1],
                p99_ms=ordered[math.ceil(.99 * len(values)) - 1])


class IdealSimulation:
    def __init__(self, cfg=None, output=None, model=None, predictor=None):
        self.cfg = cfg or ExperimentConfig()
        if self.cfg.predictor_path and predictor is None:
            from relay_learning.backend import OnnxPredictor
            predictor = OnnxPredictor(self.cfg.predictor_path, self.cfg.inference_timeout_s)
        if self.cfg.strategy == "predictive" and predictor is None:
            raise ValueError("predictive requires a trained --predictor model")
        self.predictor = predictor
        self.monitor = LinkMonitor(self.cfg, predictor)
        self.controller = RouteController(self.cfg.service_dbm, self.cfg.disconnect_dbm,
                                          self.cfg.hysteresis_db, self.cfg.min_dwell_s,
                                          self.cfg.probe_s, self.cfg.route_ack_s, self.cfg.route_timeout_s,
                                          self.cfg.reactive_on_slots)
        self.run_context = {"entrypoint": "python_api"}
        self.output = output
        self.world = WarehouseMap(self.cfg)
        self.rng = random.Random(self.cfg.seed)
        self.scheduler = CandidateScheduler(self.cfg, self.world.nodes,
                                            model if model is not None else load_model(self.cfg.model_plugin))
        self.now, self.ticks = 0., 0
        self.round_index, self.direction = 1, "outbound"
        self.phase, self.progress = "DEPLOY", 0.
        self.route = outward_route()
        self.position = self.route[0]
        self.relay_positions = [(0., 0.), (0., 0.)]
        self.relay_status = ["STANDBY", "STANDBY"]
        self.targets = [(0., 0.), (0., 0.)]
        self.relay_timers = [0., 0.]
        self.relay_battery = [1., 1.]
        self.plan_epoch = 0
        self.last_plan_at = 0.
        self.deploy_started = 0.
        self.failure_reason = None
        self.degraded = False
        self.route_event_cursor = 0
        self.total_switches = 0
        self.outage_events = 0
        self.deployment_count = 0
        self.deployment_successes = 0
        self.pending_detach = None
        self.last_contact = 0.
        self.hold_started = None
        self.emergency_route = None
        self.emergency_progress = 0.
        self.planning_ms = []
        self.link_history = deque(maxlen=20)
        self.events, self.samples, self.legs, self.decisions = [], [], [], []
        self.removal = None
        self.commands = {}
        self.window_announced, self.window_closed = False, False
        self.return_elapsed = 0.
        self.random_at = self.rng.uniform(self.cfg.window_start_s, self.cfg.window_end_s)
        self.optimization = None
        self.finished = False
        self.current_route = ()
        self.dwell_elapsed = 0.
        self.main_distance_m, self.relay_distance_m = 0., 0.
        self.leg_distance, self.leg_elapsed = 0., 0.
        self.leg_usage, self.leg_outage, self.leg_weak, self.leg_switches = 0., 0., 0., 0
        self.all_outage_s, self.max_outage_s, self.outage_run_s = 0., 0., 0.
        self.emit("map_initialized", nodes=len(self.world.nodes), edges=len(self.world.edges),
                  boxes=len(self.world.truth), before_first_flight=True)
        if self.cfg.map_observation == "ideal":
            self.world.observe(self.position, self.now)
        self._prepare_leg()

    def emit(self, kind, **details):
        event = dict(time_s=round(self.now, 6), round=self.round_index,
                     direction=self.direction, kind=kind, **details)
        self.events.append(event)
        if self.output:
            self.output(event)

    def scheduling_state(self, occupied=None, relay_positions=None):
        remaining = length(self.route) - self.progress
        positions = tuple(at_distance(self.route, self.progress + min(i * self.cfg.planning_step_cells, remaining))
                          for i in range(math.ceil(remaining / self.cfg.planning_step_cells) + 1))
        return SchedulingState("relay-scheduling-v1", self.now, self.round_index, self.direction,
                               self.world.version, tuple(sorted(self.world.known if occupied is None else occupied)),
                               self.position, tuple(self.relay_positions if relay_positions is None else relay_positions),
                               positions, tuple(self.link_history), tuple(self.relay_battery),
                               tuple(s == "READY" for s in self.relay_status), self.progress,
                               self.controller.active, tuple(sorted(self.monitor.measured.items())),
                               tuple(self.relay_status), tuple(self.feature_state()["relay_eta_s"]))

    def _plan_targets(self):
        """Translate a plan into relay targets without aborting an in-flight relay.

        ``None`` means "not taskable".  A READY relay that is no longer needed
        returns home; a relay already moving keeps its announced target until it
        becomes READY, so a replan cannot silently redirect or fake a station.
        """
        targets = []
        for i, station in enumerate(self.plan.stations):
            if station is not None:
                targets.append(tuple(station))
            elif self.relay_status[i] == "READY":
                targets.append((0., 0.))
            else:
                targets.append(tuple(self.targets[i]))
        return targets

    def _prepare_leg(self):
        self.progress = 0.
        self.leg_distance = self.leg_elapsed = self.leg_usage = self.leg_outage = self.leg_weak = 0.
        self.leg_switches = 0
        self.phase = "DEPLOY"
        self.route = outward_route() if self.direction == "outbound" else tuple(reversed(outward_route()))
        state = self.scheduling_state()
        if self.cfg.strategy == "reactive":
            state = replace(state, planned_main_positions=(self.position,))
        try:
            self.plan = self._plan(state)
        except RuntimeError as exc:
            self.targets = list(self.relay_positions)
            self._emergency_return(str(exc))
            return
        if self.cfg.strategy == "fixed":
            self.plan = next(c for c in self.scheduler.last_candidates
                             if c.stations == self.cfg.initial_stations)
        elif self.cfg.strategy == "reactive":
            # No anticipation at turnarounds: keep current stations until a live threshold trigger.
            stations = tuple(p if s == "READY" else None
                             for p, s in zip(self.relay_positions, self.relay_status))
            self.plan = next(c for c in self.scheduler.last_candidates if c.stations == stations)
        self.decisions.append(self.scheduler.record(state, self.plan, self.scheduler.last_fallback))
        self.decisions[-1]["candidates"] = [asdict(c) for c in self.scheduler.last_candidates]
        self.decisions[-1]["candidate_cost_source"] = "known-map model, not future ground-truth labels"
        if self.scheduler.last_fallback:
            self.emit("model_fallback", reason=self.scheduler.last_fallback)
        self.emit("plan_selected", map_version=self.world.version,
                  candidate_id=self.plan.candidate_id, stations=self.plan.stations,
                  predicted_cost=self.plan.cost,
                  strategy=self.cfg.strategy)
        if self.round_index == 2 and self.direction == "outbound":
            self._compare_second_outbound(state)
        self.targets = self._plan_targets()
        self.plan_epoch += 1
        self.deploy_started = self.last_plan_at = self.now
        if self.cfg.strategy == "reactive":
            self.phase = "FLY"
            self.emit("reactive_wait_for_weak_link")
        else:
            self.deployment_count += 1

    def _plan(self, state):
        started = time.perf_counter()
        try:
            return self.scheduler.plan(state)
        finally:
            self.planning_ms.append((time.perf_counter() - started) * 1000.)

    def _compare_second_outbound(self, state):
        # 反事实对照：相同当前中继位置、相同未来固定路线、相同变更真值。
        # 真值仅用于离线评估，未送入任何 plan()/模型调用。
        baseline = CandidateScheduler(self.cfg, self.world.nodes)
        stale = baseline.plan(replace(state, known_occupied=tuple(sorted(self.world.initial)), map_version=0))
        initial = next(c for c in baseline.last_candidates if c.stations == self.cfg.initial_stations)
        updated = baseline.plan(state)
        records = {}
        for name, plan in (("first_pass_fixed", initial), ("frozen_map", stale),
                           ("updated_map", updated), ("selected", self.plan)):
            records[name] = {"stations": plan.stations, "movement_m": plan.movement_m,
                             **evaluate(plan.stations, state.planned_main_positions, self.world.truth, self.cfg)}
        keys = ("outage_samples", "weak_samples", "relay_hop_samples", "movement_m", "switches")
        old_cost = tuple(records["frozen_map"][k] for k in keys)
        new_cost = tuple(records["updated_map"][k] for k in keys)
        initial_cost = tuple(records["first_pass_fixed"][k] for k in keys)
        self.optimization = {
            "evaluation_scope": "same-scene counterfactual; relays at planned stations; no deployment time",
            "map_version": self.world.version, "sample_spacing_cells": self.cfg.planning_step_cells,
            "sample_count": len(state.planned_main_positions),
            "records": records, "lexicographic_improvement": new_cost < initial_cost,
            "map_only_improvement": new_cost < old_cost,
            "delta_updated_minus_initial": {k: records["updated_map"][k] - records["first_pass_fixed"][k] for k in keys},
            "delta_updated_minus_frozen": {k: records["updated_map"][k] - records["frozen_map"][k] for k in keys}}
        self.emit("second_outbound_comparison", **self.optimization)

    @property
    def window_open(self):
        return (not self.finished and self.cfg.mode == "window" and self.round_index == 1
                and self.direction == "return" and self.phase == "FLY"
                and self.cfg.window_start_s - 1e-9 <= self.return_elapsed <= self.cfg.window_end_s + 1e-9
                and self.removal is None)

    def submit_removal(self, cells, command_id="manual-1"):
        """一次原子移除2–4块。窗口外、重复格、非法格、来不及观测时拒绝。

        同ID同内容重发返回原结果；同ID异内容拒绝。命令本身不更新在线地图。
        """
        try:
            cells = tuple(tuple(c) for c in cells)
        except TypeError:
            return {"accepted": False, "reason": "cells必须为二维坐标列表"}
        if not isinstance(command_id, str) or not command_id:
            return {"accepted": False, "reason": "command_id必须为非空字符串"}
        if command_id in self.commands:
            previous, result = self.commands[command_id]
            return dict(result) if previous == cells else {"accepted": False, "reason": "相同ID的内容不同"}
        reason = None
        if not self.window_open:
            reason = "不在首轮返程可操作窗口，或本轮已经执行移除"
        elif not 2 <= len(cells) <= 4:
            reason = "一次任务必须移除2–4块"
        elif any(len(c) != 2 or any(type(v) is not int for v in c) for c in cells):
            reason = "货箱坐标须为整数(列,行)，范围(1..3,1..4)"
        elif len(set(cells)) != len(cells):
            reason = "不允许重复货箱"
        elif not set(cells) <= self.world.truth:
            reason = "货箱不存在或已经移除"
        elif not set(cells) <= self.world.observable_ahead(self.route, self.progress):
            reason = "部分区块已超出剩余返程观测范围，请选择仍可观测的区块或提前提交"
        result = {"accepted": reason is None, "reason": reason or "已移除，等待巡线观测更新地图"}
        # 拒绝不占用ID，允许窗口开启后纠正重试。
        if reason is None:
            self._remove(cells, "manual", command_id)
            self.commands[command_id] = cells, dict(result)
        self.emit("removal_command", command_id=command_id, cells=cells, **result)
        return result

    def _remove(self, cells, source, command_id):
        self.world.truth.difference_update(cells)
        self.removal = dict(cells=cells, source=source, command_id=command_id,
                            time_s=self.now, return_elapsed_s=self.return_elapsed)
        self.emit("boxes_removed", **{k: v for k, v in self.removal.items() if k != "time_s"})

    def _change_events(self):
        if self.round_index != 1 or self.direction != "return":
            return
        if self.cfg.mode == "random" and self.removal is None and self.return_elapsed + 1e-9 >= self.random_at:
            eligible = sorted(self.world.observable_ahead(self.route, self.progress))
            if len(eligible) < 2:
                raise RuntimeError("随机变更时刻太晚，剩余可观测货箱不足2块")
            count = self.rng.randint(2, min(4, len(eligible)))
            self._remove(tuple(self.rng.sample(eligible, count)), "random", "random-1")
        if self.window_open and not self.window_announced:
            self.window_announced = True
            self.emit("window_open", start_s=self.cfg.window_start_s, end_s=self.cfg.window_end_s,
                      eligible_cells=sorted(self.world.observable_ahead(self.route, self.progress)))
        if self.cfg.mode == "window" and self.return_elapsed > self.cfg.window_end_s + 1e-9 and not self.window_closed:
            self.window_closed = True
            self.emit("window_closed", removal_submitted=self.removal is not None)

    def _observe(self):
        if self.cfg.map_observation == "static":
            return
        changed = self.world.observe(self.position, self.now)
        if changed:
            self.emit("map_updated", cells=changed, map_version=self.world.version,
                      known_occupied=sorted(self.world.known))

    def _move_relays(self):
        cfg = self.cfg
        # One relay moves at a time; an upstream relay cannot leave an active route.
        self.pending_detach = None
        for i, (point, target) in enumerate(zip(self.relay_positions, self.targets)):
            status = self.relay_status[i]
            if point == target and status in ("READY", "STANDBY"):
                continue
            if i + 1 in self.controller.active and point != target:
                self.pending_detach = i + 1
                continue
            if status == "STANDBY" and point != target:
                if cfg.vehicle_kind == "ground_car" or cfg.takeoff_s <= 1e-9:
                    # Ground cars start rolling immediately; no takeoff phase.
                    self.relay_status[i] = "MOVING"
                else:
                    self.relay_status[i], self.relay_timers[i] = "TAKEOFF", cfg.takeoff_s
                    return False
            if status in ("TAKEOFF", "SETTLING"):
                self.relay_timers[i] = max(0., self.relay_timers[i] - cfg.dt_s)
                if self.relay_timers[i] <= 1e-9:
                    self.relay_status[i] = ("STANDBY" if target == (0., 0.) else "READY") if status == "SETTLING" else "MOVING"
                return False
            self.relay_status[i] = "MOVING"
            budget = cfg.relay_speed_mps * cfg.dt_s / cfg.cell_m
            coords = list(point)
            # 沿格线，先水平后竖直；每次部署端点都是合法格点。
            for axis in (0, 1):
                amount = min(abs(target[axis] - coords[axis]), budget)
                coords[axis] += math.copysign(amount, target[axis] - coords[axis])
                budget -= amount
                self.relay_distance_m += amount * self.cfg.cell_m
                self.relay_battery[i] -= amount * cfg.cell_m * cfg.battery_per_m
                if abs(coords[axis] - target[axis]) < 1e-9:
                    coords[axis] = target[axis]
            self.relay_positions[i] = tuple(coords)
            if self.relay_positions[i] == target:
                if cfg.vehicle_kind == "ground_car" or cfg.settle_s <= 1e-9:
                    self.relay_status[i] = "STANDBY" if target == (0., 0.) else "READY"
                    self.relay_timers[i] = 0.
                else:
                    self.relay_status[i], self.relay_timers[i] = "SETTLING", cfg.settle_s
            return False
        return all(p == t and s in ("READY", "STANDBY")
                   for p, t, s in zip(self.relay_positions, self.targets, self.relay_status))

    def feature_state(self):
        """Snapshot of estimates made NOW, never backfilled from actual future arrivals.

        ETA is remaining nominal takeoff/travel/settling time. Serialized motion,
        active-route dependencies and route ACK delays are not arrival guarantees.
        Metadata is outside the frozen 96-dimensional encoder.
        """
        cfg = self.cfg
        eta = []
        for p, target, status, timer in zip(self.relay_positions, self.targets,
                                           self.relay_status, self.relay_timers):
            travel = sum(abs(a - b) for a, b in zip(p, target)) * cfg.cell_m / cfg.relay_speed_mps
            eta.append(travel + (cfg.takeoff_s if status == "STANDBY" and travel else 0.)
                       + (timer if status in ("TAKEOFF", "SETTLING") else 0.)
                       + (cfg.settle_s if travel else 0.))
        return dict(main_position=self.position, relay_positions=tuple(self.relay_positions),
                    relay_targets=tuple(self.targets), relay_ready=tuple(s == "READY" for s in self.relay_status),
                    relay_eta_s=eta, relay_battery=tuple(self.relay_battery),
                    relay_eta_estimated_at_s=self.now,
                    relay_eta_basis="nominal_takeoff_travel_settle_excludes_queue_handover_ack",
                    relay_eta_blocked=tuple(i + 1 in self.controller.active and p != target
                                            for i, (p, target) in enumerate(zip(self.relay_positions, self.targets))),
                    direction=self.direction, progress_cells=self.progress, phase=self.phase,
                    progress_m=self.progress * cfg.cell_m,
                    segment_id=route_position(self.direction, self.progress)[1],
                    mission_distance_m=self.main_distance_m,
                    main_speed_mps=cfg.main_speed_mps, cell_m=cfg.cell_m)

    def _stop(self, reason):
        self.failure_reason = reason
        self.finished, self.phase = True, "ABORTED"
        self.emit("mission_aborted", reason=reason)

    def _emergency_return(self, reason):
        # This is the simulated onboard executor. Position is assumed valid.
        # Do not require a new ground-station command after loss of communication.
        self.failure_reason = reason
        remaining, segment = self.progress, 0
        for segment, (a, b) in enumerate(zip(self.route, self.route[1:])):
            size = math.dist(a, b)
            if remaining <= size + 1e-9:
                break
            remaining -= size
        points = ([self.position] + list(reversed(self.route[:segment + 1]))
                  if self.direction == "outbound"
                  else [self.position] + list(self.route[segment + 1:]))
        compact = [points[0]]
        for point in points[1:]:
            if math.dist(point, compact[-1]) > 1e-9:
                compact.append(point)
        self.emergency_route, self.emergency_progress = tuple(compact), 0.
        self.phase = "FAILSAFE_RETURN"
        self.plan_epoch += 1
        self.emit("onboard_emergency_return", reason=reason, route=self.emergency_route,
                  position_assumed_valid=True)

    def _rolling_plan(self):
        cfg = self.cfg
        if cfg.strategy == "fixed":
            return
        reactive = cfg.strategy == "reactive"
        # The passive baseline checks every observation. A periodic planning gate
        # must not delay its first threshold reaction until after failsafe entry.
        if not reactive and self.now - self.last_plan_at < cfg.rolling_replan_s:
            return
        current_quality = path_quality(self.controller.active, self.monitor.measured,
                                       tuple(s == "READY" for s in self.relay_status))
        risks = self.monitor.risks
        forecast_weak = (risks is not None and any(risks[link][1] >= self.monitor.thresholds[1]
                         for link in ROUTE_LINKS[self.controller.active]))
        if reactive and current_quality >= cfg.service_dbm:
            return
        # Predictive trigger can bring replanning forward; periodic known-map planning
        # remains the stronger CPU baseline when predictions are unavailable.
        if cfg.strategy == "predictive" and risks is not None and not forecast_weak and current_quality >= cfg.service_dbm:
            return
        state = self.scheduling_state()
        if reactive:
            state = replace(state, planned_main_positions=(self.position,))
        try:
            plan = self._plan(state)
        except RuntimeError as exc:
            self._emergency_return(str(exc))
            return
        self.last_plan_at = self.now
        self.plan = plan
        targets = self._plan_targets()
        if targets == self.targets:
            return
        self.targets = targets
        self.plan_epoch += 1
        self.phase, self.deploy_started = "DEPLOY", self.now
        self.deployment_count += 1
        self.decisions.append(self.scheduler.record(state, plan, self.scheduler.last_fallback))
        self.decisions[-1]["reason"] = "deployment-risk" if forecast_weak else "CPU rolling plan"
        self.emit("rolling_plan", stations=plan.stations, estimated_deployment_s=plan.deployment_s,
                  predicted_within_horizon=plan.deployment_s <= cfg.deploy_horizon_s,
                  main_waits_for_ready=True)

    def _measure(self, flying):
        cfg = self.cfg
        g, m, r1, r2 = (0., 0.), self.position, *self.relay_positions
        physical = tuple(hop_rssi(a, b, self.world.truth, cfg)
                         for a, b in ((g, m), (g, r1), (g, r2), (r1, m), (r2, m), (r1, r2)))
        actual = self.monitor.update(self.now, physical, self.feature_state(), self.plan_epoch)
        ready = tuple(s == "READY" for s in self.relay_status)
        risks = {link: row[0] for link, row in self.monitor.risks.items()} if self.monitor.risks else None
        forbidden = (self.pending_detach,) if self.pending_detach else ()
        desired = self.controller.propose(self.now, self.monitor.measured, ready, risks,
                                          self.monitor.thresholds[0], forbidden)
        degraded = not any(path_quality(r, self.monitor.measured, ready) >= cfg.disconnect_dbm
                           for r in ROUTE_LINKS)
        if degraded != self.degraded:
            self.emit("link_degraded" if degraded else "link_available",
                      reason="no feasible observed path" if degraded else "fresh feasible path")
        self.degraded = degraded
        self.controller.request(desired, self.now)
        self.controller.advance(self.now, self.monitor.measured, ready)
        pending = self.controller.pending
        if pending is not None and random.Random(f"{cfg.seed}:ack:{pending.epoch}").random() >= cfg.ack_drop_rate:
            self.controller.acknowledge(pending.epoch, self.now, self.monitor.measured, ready)
        for event in self.controller.events[self.route_event_cursor:]:
            self.emit(event["kind"], **{k: v for k, v in event.items() if k not in ("kind", "time_s")})
        self.route_event_cursor = len(self.controller.events)
        route = self.controller.active
        if path_quality(route, self.monitor.measured, ready) >= cfg.disconnect_dbm:
            self.last_contact = self.now
        effective = path_quality(route, dict(zip(LINKS, actual)), ready)
        effective = max(effective, -150.)  # finite log sentinel, never considered a good link
        dt = self.cfg.dt_s
        if effective < self.cfg.disconnect_dbm:
            self.outage_events += self.outage_run_s == 0
            self.all_outage_s += dt
            self.outage_run_s += dt
            self.max_outage_s = max(self.max_outage_s, self.outage_run_s)
        else:
            self.outage_run_s = 0.
        if flying:
            self.leg_elapsed += dt
            self.leg_usage += len(route) * dt
            self.leg_outage += (effective < self.cfg.disconnect_dbm) * dt
            self.leg_weak += (effective < self.cfg.service_dbm) * dt
            self.leg_switches += route != self.current_route
        self.total_switches += route != self.current_route
        self.current_route = route
        # Legacy scoring plug-in history receives measurements only, never hidden truth.
        self.link_history.append((self.now, *(self.monitor.measured.get(link, -150.) for link in LINKS)))
        for i, status in enumerate(self.relay_status):
            if status != "STANDBY":
                self.relay_battery[i] = max(0., self.relay_battery[i] - cfg.battery_per_s * dt)
        self.samples.append(dict(time_s=round(self.now, 6), round=self.round_index,
                                 direction=self.direction, phase=self.phase,
                                 x=self.position[0], y=self.position[1],
                                 progress_cells=round(self.progress, 9),
                                 progress_m=round(self.progress * cfg.cell_m, 9),
                                 segment_id=route_position(self.direction, self.progress)[1],
                                 mission_distance_m=round(self.main_distance_m, 9),
                                 r1_x=r1[0], r1_y=r1[1], r2_x=r2[0], r2_y=r2[1],
                                 map_version=self.world.version, route=list(route),
                                 effective_rssi_dbm=effective, window_open=self.window_open,
                                 relay_status=list(self.relay_status), relay_battery=list(self.relay_battery),
                                 relay_eta_s=self.feature_state()["relay_eta_s"],
                                 relay_eta_estimated_at_s=self.now,
                                 route_epoch=self.controller.epoch,
                                 link_degraded=self.degraded,
                                 prediction=self.monitor.risks, model_fallback=self.monitor.last_error,
                                 measured_links=self.monitor.measured,
                                 observations=self.monitor.observations))

    def _finish_dwell(self):
        self.emit("dwell_complete", dwell_s=round(self.dwell_elapsed, 9))
        if self.direction == "outbound":
            self.direction = "return"
        elif self.round_index == self.cfg.rounds:
            self.phase, self.targets = "RECOVER", [(0., 0.), (0., 0.)]
            self.deploy_started = self.now
            # The mission is complete and main is home; no downstream service remains.
            self.controller.active, self.controller.pending = (), None
            self.plan_epoch += 1
            return
        else:
            self.round_index += 1
            self.direction = "outbound"
        self._prepare_leg()

    def step(self):
        if self.finished:
            return
        self.ticks += 1
        self.now = round(self.ticks * self.cfg.dt_s, 9)
        if self.phase == "FAILSAFE_RETURN":
            previous = self.emergency_progress
            total = length(self.emergency_route)
            self.emergency_progress = min(total, previous + self.cfg.main_speed_mps * self.cfg.dt_s / self.cfg.cell_m)
            self.position = at_distance(self.emergency_route, self.emergency_progress)
            self.main_distance_m += (self.emergency_progress - previous) * self.cfg.cell_m
            self._measure(False)
            if self.emergency_progress >= total - 1e-9:
                self.phase, self.targets = "RECOVER", [(0., 0.), (0., 0.)]
                self.deploy_started = self.now
                self.controller.active, self.controller.pending = (), None
            return
        if self.phase == "LINK_HOLD":
            self._measure(False)
            if self.now - self.last_contact <= self.cfg.dt_s + 1e-8:
                self.phase, self.hold_started = "FLY", None
                self.emit("onboard_link_recovered")
            elif self.now - self.hold_started >= self.cfg.failsafe_hold_s:
                self._emergency_return("link observations unavailable for failsafe hold interval")
            return
        if self.phase in ("DEPLOY", "RECOVER"):
            ready = self._move_relays()
            self._measure(False)
            if self.now - self.deploy_started > self.cfg.deployment_timeout_s:
                self._stop("deployment timeout: cannot release active relay or reach target")
                return
            if ready:
                if self.phase == "RECOVER":
                    self.finished, self.phase = True, "ABORTED" if self.failure_reason else "DONE"
                    self.emit("mission_aborted" if self.failure_reason else "mission_complete",
                              reason=self.failure_reason, completed_rounds=len(self.legs) // 2)
                else:
                    self.deployment_successes += 1
                    self.phase = "FLY"
                    if self.round_index == 1 and self.direction == "return":
                        self.return_elapsed = 0.
                    self.emit("leg_started", stations=self.plan.stations)
            return
        if self.phase == "DWELL":
            self.dwell_elapsed += self.cfg.dt_s
            self._measure(False)
            dwell = self.cfg.terminal_dwell_s if self.direction == "outbound" else self.cfg.home_dwell_s
            if self.dwell_elapsed + 1e-9 >= dwell:
                self._finish_dwell()
            return
        if self.now - self.last_contact > self.cfg.measurement_max_age_s:
            self.phase, self.hold_started = "LINK_HOLD", self.now
            self.emit("onboard_link_hold", hold_s=self.cfg.failsafe_hold_s)
            self._measure(False)
            return
        if min(self.relay_battery) < self.cfg.battery_reserve:
            self._emergency_return("relay battery reserve")
            return
        old = self.progress
        self.progress = min(length(self.route), old + self.cfg.main_speed_mps * self.cfg.dt_s / self.cfg.cell_m)
        self.position = at_distance(self.route, self.progress)
        movement = (self.progress - old) * self.cfg.cell_m
        self.main_distance_m += movement
        self.leg_distance += movement
        if self.round_index == 1 and self.direction == "return":
            self.return_elapsed = self.progress * self.cfg.cell_m / self.cfg.main_speed_mps
        self._change_events()
        self._observe()
        self._measure(True)
        if self.progress >= length(self.route) - 1e-9:
            self.position = self.route[-1]
            record = dict(round=self.round_index, direction=self.direction,
                          distance_m=round(self.leg_distance, 6), flight_s=round(self.leg_elapsed, 6),
                          outage_s=round(self.leg_outage, 6), weak_s=round(self.leg_weak, 6),
                          relay_hop_s=round(self.leg_usage, 6), switches=self.leg_switches,
                          map_version=self.world.version, stations=self.plan.stations)
            self.legs.append(record)
            self.emit("leg_complete", metrics=record, surveyed_nodes=len(self.world.node_samples))
            if self.round_index == 1 and self.direction == "return":
                self.emit("first_return_map", removed_observed=self.world.known == self.world.truth,
                          map_version=self.world.version, removed_count=len(self.world.initial - self.world.truth))
            self.phase, self.dwell_elapsed = "DWELL", 0.
            dwell = self.cfg.terminal_dwell_s if self.direction == "outbound" else self.cfg.home_dwell_s
            if dwell == 0.:
                self._finish_dwell()
        else:
            self._rolling_plan()

    def run(self, max_steps=100000):
        for _ in range(max_steps):
            if self.finished:
                if self.predictor is not None and callable(getattr(self.predictor, "close", None)):
                    self.predictor.close()
                return self.report()
            self.step()
        raise RuntimeError("超过最大步数，任务未完成")

    def snapshot(self):
        return dict(time_s=self.now, round=self.round_index, direction=self.direction, phase=self.phase,
                    main=self.position, relays=self.relay_positions, active_route=self.current_route,
                    window_open=self.window_open, return_elapsed_s=self.return_elapsed,
                    map=self.world.snapshot(), removed=sorted(self.world.initial - self.world.truth))

    def report(self):
        return dict(config=asdict(self.cfg), contract=manifest(self.cfg), run_context=self.run_context,
                    finished=self.finished, success=self.finished and self.failure_reason is None,
                    failure_reason=self.failure_reason, completed_rounds=len(self.legs) // 2,
                    single_legs=len(self.legs), total_time_s=self.now,
                    phase_time_s={phase: round(sum(s["phase"] == phase for s in self.samples) * self.cfg.dt_s, 9)
                                  for phase in sorted({s["phase"] for s in self.samples})},
                    main_distance_m=round(self.main_distance_m, 6),
                    relay_distance_m=round(self.relay_distance_m, 6),
                    all_phase_outage_s=round(self.all_outage_s, 6), longest_outage_s=round(self.max_outage_s, 6),
                    outage_events=self.outage_events, total_switches=self.total_switches,
                    deployment_count=self.deployment_count, deployment_successes=self.deployment_successes,
                    predictor_source=self.predictor.metadata.get("data_source") if self.predictor else "none",
                    inference_calls=self.monitor.inference_calls, inference_fallbacks=self.monitor.fallback_count,
                    inference_cycle_timing=timing_summary(self.monitor.inference_ms),
                    planning_timing=timing_summary(self.planning_ms),
                    relay_hop_s=round(sum(len(s["route"]) * self.cfg.dt_s for s in self.samples), 6),
                    pingpong_count=sum(b["route"] == a["old_route"] and b["old_route"] == a["route"]
                                       and b["time_s"] - a["time_s"] <= 2.
                                       for a, b in zip(
                                           [e for e in self.controller.events if e["kind"] == "route_confirmed"],
                                           [e for e in self.controller.events if e["kind"] == "route_confirmed"][1:])),
                    feature_schema=self.monitor.spec.manifest(),
                    change_executed=self.removal is not None, removal=self.removal,
                    final_map_matches_truth=self.world.known == self.world.truth,
                    final_map=self.world.snapshot(), legs=self.legs, second_outbound=self.optimization,
                    model_plugin=self.cfg.model_plugin or "CPU enumeration (no trained AI)",
                    assumptions=["perfect position; occupancy observation=" + self.cfg.map_observation,
                                 "finite-speed kinematics; serialized relay motion; main waits during deployment",
                                 "simulated takeoff/settling/probe/commit/ACK, not a real radio implementation",
                                 "six symmetric physical links; configurable noise/drop/delay",
                                 "onboard failsafe uses link-observation availability as a heartbeat proxy",
                                 "one occupied cell contributes one shelf loss; no +6dB gain",
                                 "battery is a proxy; no aerodynamic or full collision avoidance model"])
