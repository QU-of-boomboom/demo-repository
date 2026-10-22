from dataclasses import dataclass
import math

from .contract import CELL_M, DT_S, TERMINAL_DWELL_S, HOME_DWELL_S, OUTWARD
from .profile import PROFILE

_MISSION = PROFILE["mission"]
_TIMING = PROFILE["timing"]
_RADIO = PROFILE["radio"]
_THRESHOLDS = PROFILE["thresholds"]
_STATIONS = PROFILE["stations"]
_SCHEDULING = PROFILE["scheduling"]


@dataclass(frozen=True)
class ExperimentConfig:
    """Current A/B/C experiment configuration, defaulted from mission_profile.json.

    Values are loaded once at import.  Tests and the CLI may still override
    individual fields explicitly; the profile is the shared source of defaults,
    not a second hidden parameter set.
    """
    rounds: int = int(_MISSION["rounds"])
    columns: int = int(PROFILE["map"]["columns"])
    rows: int = int(PROFILE["map"]["rows"])
    vehicle_kind: str = str(PROFILE["vehicle"]["kind"])
    cell_m: float = CELL_M
    dt_s: float = DT_S
    main_speed_mps: float = float(_TIMING["main_speed_mps"])
    relay_speed_mps: float = float(_TIMING["relay_speed_mps"])
    terminal_dwell_s: float = TERMINAL_DWELL_S
    home_dwell_s: float = HOME_DWELL_S
    mode: str = str(_MISSION["mode"])
    seed: int = int(_MISSION["seed"])
    window_start_s: float = float(_MISSION["window_start_s"])
    window_end_s: float = float(_MISSION["window_end_s"])
    sensor_range_cells: float = float(_MISSION["sensor_range_cells"])
    service_dbm: float = float(_THRESHOLDS["service_dbm"])
    disconnect_dbm: float = float(_THRESHOLDS["disconnect_dbm"])
    shelf_loss_db: float = float(_RADIO["shelf_loss_db"])
    p0_dbm: float = float(_RADIO["p0_dbm"])
    path_loss_n: float = float(_RADIO["path_loss_n"])
    planning_step_cells: float = float(_SCHEDULING["planning_step_cells"])
    initial_stations: tuple = tuple(tuple(float(v) for v in p) for p in _STATIONS["initial"])
    model_plugin: str = ""
    predictor_path: str = ""
    inference_timeout_s: float = float(_SCHEDULING["inference_timeout_s"])
    inference_period_s: float = float(_SCHEDULING["inference_period_s"])
    short_horizon_s: float = float(_SCHEDULING["short_horizon_s"])
    deploy_horizon_s: float = float(_SCHEDULING["deploy_horizon_s"])
    takeoff_s: float = float(_TIMING["takeoff_s"])
    settle_s: float = float(_TIMING["settle_s"])
    probe_s: float = float(_TIMING["probe_s"])
    route_ack_s: float = float(_TIMING["route_ack_s"])
    route_timeout_s: float = float(_TIMING["route_timeout_s"])
    min_dwell_s: float = float(_THRESHOLDS["min_dwell_s"])
    hysteresis_db: float = float(_THRESHOLDS["hysteresis_db"])
    reactive_on_slots: int = int(_THRESHOLDS["reactive_on_slots"])
    deployment_timeout_s: float = float(_TIMING["deployment_timeout_s"])
    measurement_max_age_s: float = float(_TIMING["measurement_max_age_s"])
    measurement_delay_s: float = 0.0
    measurement_drop_rate: float = 0.0
    noise_sigma_db: float = 0.0
    ack_drop_rate: float = 0.0
    map_observation: str = str(_MISSION["map_observation"])
    strategy: str = str(_MISSION["strategy"])
    rolling_replan_s: float = float(_SCHEDULING["rolling_replan_s"])
    battery_reserve: float = float(_SCHEDULING["battery_reserve"])
    battery_per_m: float = float(_SCHEDULING["battery_per_m"])
    battery_per_s: float = float(_SCHEDULING["battery_per_s"])
    record_learning: bool = False
    failsafe_hold_s: float = float(_TIMING["failsafe_hold_s"])

    def __post_init__(self):
        if type(self.rounds) is not int or self.rounds < 1:
            raise ValueError("往返轮数必须为正整数，默认5轮")
        if (self.columns, self.rows) != (3, 4):
            raise ValueError("本实验采用 README 的3×4地图")
        if self.mode not in ("random", "window", "none"):
            raise ValueError("mode须为random/window/none")
        positive = (self.cell_m, self.dt_s, self.main_speed_mps,
                    self.relay_speed_mps, self.sensor_range_cells, self.planning_step_cells)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("格长、步长、速度、观测半径必须为有限正数")
        values = (self.window_start_s, self.window_end_s, self.terminal_dwell_s,
                  self.home_dwell_s, self.service_dbm, self.disconnect_dbm,
                  self.shelf_loss_db, self.p0_dbm, self.path_loss_n)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("实验参数不能包含NaN或无穷值")
        single_leg_m = sum(math.dist(a, b) for a, b in zip(OUTWARD, OUTWARD[1:])) * self.cell_m
        if not 0 <= self.window_start_s < self.window_end_s < single_leg_m / self.main_speed_mps:
            raise ValueError("窗口必须位于首轮返程归位之前（按返程运动时间，单位秒）")
        if min(self.terminal_dwell_s, self.home_dwell_s, self.shelf_loss_db) < 0:
            raise ValueError("停留与货箱损耗不能为负")
        if self.disconnect_dbm >= self.service_dbm:
            raise ValueError("断链门限应低于服务门限")
        if (len(self.initial_stations) != 2 or self.initial_stations[0] == self.initial_stations[1]
                or any(len(p) != 2 or any(not math.isfinite(v) or int(v) != v for v in p)
                       or not 0 <= p[0] <= 3 or not 0 <= p[1] <= 4 for p in self.initial_stations)):
            raise ValueError("初始中继站位必须为两个不同合法格点")
        if self.map_observation not in ("ideal", "static"):
            raise ValueError("map_observation must be ideal or static")
        if self.strategy not in ("fixed", "reactive", "predeploy", "predictive"):
            raise ValueError("Unknown scheduling strategy")
        if self.vehicle_kind not in ("relay_drone", "ground_car"):
            raise ValueError("vehicle_kind must be relay_drone or ground_car")
        timing = (self.inference_timeout_s, self.inference_period_s, self.short_horizon_s,
                  self.deploy_horizon_s, self.route_timeout_s, self.deployment_timeout_s,
                  self.measurement_max_age_s, self.rolling_replan_s, self.failsafe_hold_s)
        nonnegative = (self.takeoff_s, self.settle_s, self.probe_s, self.route_ack_s,
                       self.min_dwell_s, self.hysteresis_db, self.measurement_delay_s,
                       self.noise_sigma_db, self.battery_per_m, self.battery_per_s)
        if any(not math.isfinite(v) or v <= 0 for v in timing) or any(
                not math.isfinite(v) or v < 0 for v in nonnegative):
            raise ValueError("Invalid execution timing/noise/battery parameter")
        if not all(0 <= p < 1 for p in (self.measurement_drop_rate, self.ack_drop_rate, self.battery_reserve)):
            raise ValueError("Drop rates and reserve must be in [0,1)")
        if type(self.reactive_on_slots) is not int or self.reactive_on_slots < 1:
            raise ValueError("reactive_on_slots must be a positive integer")
        if self.probe_s + self.route_ack_s >= self.route_timeout_s:
            raise ValueError("Route timeout must exceed probe and ACK delays")
