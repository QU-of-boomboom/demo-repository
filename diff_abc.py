"""Diff A/B/C positions, occlusion and all six physical RSSI hops; exit nonzero on any mismatch."""
import argparse
import csv
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT / "swarm_cooperation/swarm_cooperation"),
               str(ROOT / "ai_relay_scheduler/ai_relay_scheduler")]
from ideal_sim import world as B
from ideal_sim.config import ExperimentConfig
from ideal_sim.contract import (LINKS, ENDPOINTS, OCCUPIED, PROFILE_SHA256, PROFILE_VERSION,
                                route_position, VERSION)
from ideal_sim.simulation import IdealSimulation


def entry(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def geometry_rows(module, sample, occupied, cfg):
    nodes = ((0., 0.), (sample["r1_x"], sample["r1_y"]),
             (sample["r2_x"], sample["r2_y"]), (sample["x"], sample["y"]))
    rows = []
    for link, (i, j) in zip(LINKS, ENDPOINTS):
        a, b = nodes[i], nodes[j]
        cells = sorted(c for c in occupied if module.intersects_cell(a, b, c))
        columns = sorted({c[0] for c in cells})
        rows.append(dict(time_s=sample["time_s"], phase=sample["phase"], direction=sample["direction"],
                         progress_m=sample["progress_m"], link=link,
                         a_x_m=a[0] * cfg.cell_m, a_y_m=a[1] * cfg.cell_m,
                         b_x_m=b[0] * cfg.cell_m, b_y_m=b[1] * cfg.cell_m,
                         distance_m=math.dist(a, b) * cfg.cell_m,
                         crossed_columns=columns, crossed_column_count=len(columns),
                         crossed_cells=cells, crossed_cell_count=len(cells),
                         physical_rssi_dbm=module.hop_rssi(a, b, occupied, cfg)))
    return rows


def differences(reference, candidate, tolerance=1e-9):
    diff = []
    if len(reference) != len(candidate):
        diff.append(dict(field="row_count", expected=len(reference), actual=len(candidate)))
    for index, (a, b) in enumerate(zip(reference, candidate)):
        for key in a.keys() | b.keys():
            expected, actual = a.get(key), b.get(key)
            equal = (math.isfinite(expected) and math.isfinite(actual)
                     and math.isclose(expected, actual, rel_tol=0., abs_tol=tolerance)
                     if isinstance(expected, (int, float)) and isinstance(actual, (int, float))
                     else expected == actual)
            if not equal:
                diff.append(dict(row=index, time_s=a.get("time_s"), link=a.get("link"),
                                 field=key, expected=expected, actual=actual))
    return diff


def run(output, cfg=None):
    cfg = cfg or ExperimentConfig(rounds=1, mode="random", noise_sigma_db=.5, measurement_delay_s=.1)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    A = entry("diff_module_a", "warehouse_relay_sim/warehouse_relay_sim/warehouse_relay_sim.py")
    C = entry("diff_module_c", "ai_relay_scheduler/ai_relay_scheduler/simulator.py")
    tables, summaries = {}, {}
    for name, module, factory in (("A", A, A.create_simulation), ("B", B, IdealSimulation),
                                  ("C", C, C.create_simulation)):
        sim = factory(cfg)
        report = sim.run()
        rows, occupied = [], set(OCCUPIED)
        removals = iter(e for e in sim.events if e["kind"] == "boxes_removed")
        removal = next(removals, None)
        for sample in sim.samples:
            while removal is not None and removal["time_s"] <= sample["time_s"]:
                occupied.difference_update(tuple(c) for c in removal["cells"])
                removal = next(removals, None)
            rows.extend(geometry_rows(module, sample, occupied, cfg))
        tables[name] = rows
        summaries[name] = dict(success=report["success"], samples=len(sim.samples), six_hop_rows=len(rows),
                               main_distance_m=report["main_distance_m"], removed_cells=report["removal"]["cells"]
                               if report["removal"] else [])
        with (output / f"{name}_six_hops.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows({k: json.dumps(v) if isinstance(v, list) else v for k, v in row.items()} for row in rows)
    diffs = {name: differences(tables["A"], tables[name]) for name in ("B", "C")}
    report = dict(contract_version=VERSION, profile_version=PROFILE_VERSION,
                  profile_sha256=PROFILE_SHA256, config=asdict(cfg), modules=summaries,
                  passed=all(s["success"] for s in summaries.values()) and not any(diffs.values()),
                  mismatch_count={k: len(v) for k, v in diffs.items()}, first_mismatches={k: v[:20] for k, v in diffs.items()},
                  tolerance=1e-9, links=list(LINKS),
                  note="RSSI is deterministic pre-noise physical RSSI at actual moving positions. "
                       "Column count is diagnostic; occupied cell count determines attenuation. "
                       "Map removals applied at their event timestamps.")
    (output / "diff_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "outputs_v2/p0/diff")
    args = p.parse_args()
    report = run(args.output)
    print(json.dumps({"passed": report["passed"], "mismatch_count": report["mismatch_count"]}))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
