"""Training and B use the same simulation and feature definitions."""
import pipeline_v2
from ideal_sim.simulation import IdealSimulation
from ideal_sim.config import ExperimentConfig
from pipeline_v2.dataset import simulation_config, samples_from_episode
from ideal_sim.contract import manifest, route_position
from ideal_sim.world import outward_route, hop_rssi, intersects_cell
from ideal_sim.scheduler import route_qualities


def create_simulation(cfg=None, **kwargs):
    return IdealSimulation(cfg, **kwargs)


if __name__ == "__main__":
    from ideal_sim.cli import main
    raise SystemExit(main())
