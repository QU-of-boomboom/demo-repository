"""A display/API entry: same geometry, clock and executor as B and C.

Use --mode none --rounds 1 for one round, or --strategy reactive/predeploy.
Historical plotting and +6 dB experiments are isolated in legacy_v1.
"""
from pathlib import Path
import sys

SWARM_ROOT = Path(__file__).resolve().parents[2] / "swarm_cooperation" / "swarm_cooperation"
if str(SWARM_ROOT) not in sys.path:
    sys.path.insert(0, str(SWARM_ROOT))

from ideal_sim.config import ExperimentConfig as Config
from ideal_sim.contract import manifest, route_position
from ideal_sim.world import outward_route, hop_rssi, intersects_cell
from ideal_sim.scheduler import route_qualities
from ideal_sim.simulation import IdealSimulation


def create_simulation(cfg=None, **kwargs):
    return IdealSimulation(cfg, **kwargs)


def simulate(cfg=None):
    sim = create_simulation(cfg)
    report = sim.run()
    return {**report, "samples": sim.samples, "events": sim.events}


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if "--legacy" in args:
        import runpy
        print("Historical A V1: column loss, +6dB, instant readiness; incompatible with current A/B/C.")
        sys.argv = [str(Path(__file__).parent / "legacy_v1" / "warehouse_relay_sim.py"), *args]
        runpy.run_path(sys.argv[0], run_name="__main__")
        return 0
    from ideal_sim.cli import main as shared_main
    return shared_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
