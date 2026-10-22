"""三部分共用参数；坐标采用格点，物理距离统一换算为米。"""

from dataclasses import dataclass
from typing import Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class Config:
    columns: int = 3
    rows: int = 4
    cell_m: float = 2.0  # 未给定格子尺寸，演示假设每格 2 米
    home: Point = (0.0, 0.0)
    relay_positions: Tuple[Point, Point] = ((1.0, 2.0), (2.0, 4.0))
    dt_s: float = 0.1
    main_speed_mps: float = 1.0
    relay_speed_mps: float = 2.0
    takeoff_s: float = 2.0
    land_s: float = 2.0
    pick_s: float = 3.0
    failsafe_hold_s: float = 5.0
    handover_s: float = 0.5
    max_sim_s: float = 600.0
    p0_dbm: float = -40.0
    path_loss_n: float = 2.2
    shelf_loss_db: float = 12.0
    reference_distance_m: float = 1.0
    relay_threshold_dbm: float = -75.0
    disconnect_dbm: float = -88.0
    hysteresis_db: float = 3.0


@dataclass(frozen=True)
class NetworkConfig:
    ground_system: int = 250
    ground_component: int = 190
    main_system: int = 4
    relay_systems: Tuple[int, int] = (5, 6)
    autopilot_component: int = 1
    companion_component: int = 191
    heartbeat_s: float = 1.0
    heartbeat_timeout_s: float = 3.0
    telemetry_s: float = 0.1
    telemetry_timeout_s: float = 1.0
    ack_timeout_s: float = 0.5
    max_retries: int = 3
    duplicate_cache_s: float = 30.0
    udp_base_port: int = 16000
    mission_timeout_s: float = 2.0
    mission_total_timeout_s: float = 120.0
    connect_timeout_s: float = 30.0
    sitl_instances: Tuple[int, int, int] = (3, 4, 5)
    sitl_tcp_base: int = 5760
    sitl_port_stride: int = 10
    flight_altitude_m: float = 3.0
    gps_rate_hz: float = 5.0
    attitude_rate_hz: float = 10.0


CONFIG = Config()
NETWORK = NetworkConfig()
