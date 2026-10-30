#!/usr/bin/env python3
"""Headless scaling probe with nonintersecting parallel lanes; not a crowd benchmark."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
from batch import ROOT

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--counts", default="1,2,5,10,20")
p.add_argument("--duration", type=float, default=2)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--binary", type=Path, default=ROOT / "build/bin/avoidance_sim")
args = p.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
base = json.loads((ROOT / "scenarios/empty_corridor.json").read_text())
rows = []
for count in map(int, args.counts.split(",")):
    if count <= 0 or count > 200:
        raise ValueError("counts must be in [1,200]")
    s = copy.deepcopy(base)
    s["name"] = f"Scaling: {count} parallel robots"
    s["robots"] = []
    s["simulation"]["duration"] = args.duration
    s["obstacles"] = []
    for i in range(count):
        robot = copy.deepcopy(base["robots"][0])
        robot["id"] = f"R{i:03}"
        robot["start"][1] = robot["goal"][1] = i * 2.
        robot["path"] = [robot["start"][:2], robot["goal"][:2]]
        s["robots"].append(robot)
    path = args.output / f"scenario_{count}.json"
    path.write_text(json.dumps(s))
    run = subprocess.run([str(args.binary), "--scenario", str(path), "--no-record", "--output", str(args.output / f"run_{count}")], capture_output=True, text=True)
    if run.returncode not in (0, 2):
        raise RuntimeError(run.stderr or run.stdout)
    m = json.loads(run.stdout)
    row = dict(robots=count, simulation_seconds=m["simulation_time"], execution_seconds=m["execution_wall_seconds"],
               planner_average_ms=m["planner"]["average_wall_seconds"] * 1000, planner_max_ms=m["planner"]["maximum_wall_seconds"] * 1000, collisions=m["collisions"])
    rows.append(row)
    print(json.dumps(row), flush=True)
(args.output / "scaling.json").write_text(json.dumps(rows, indent=2) + "\n")
