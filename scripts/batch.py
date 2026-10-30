#!/usr/bin/env python3
"""Run primary scenarios, optionally experimental cases, and retain honest outcomes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PRIMARY = ["empty_corridor", "head_on_wide", "head_on_narrow", "static_obstacle_left", "static_obstacle_right", "crossing", "l_shaped_corridor"]
EXPERIMENTAL = ["three_robots", "overtaking", "unexpected_obstacle", "abrupt_stop", "communication_loss", "l_shaped_head_on"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--binary", type=Path, default=ROOT / "build/bin/avoidance_sim")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--experimental", action="store_true")
    p.add_argument("--record", action="store_true")
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()
    results = []
    print(f"{'Scenario':26} {'Outcome':20} {'Goals':7} {'Hits':5} {'Clear':6} {'Time [s]':9}", flush=True)
    for name in PRIMARY + (EXPERIMENTAL if args.experimental else []):
        command = [str(args.binary), "--scenario", str(ROOT / "scenarios" / f"{name}.json"), "--output", str(args.output / name), "--seed", str(args.seed), "--headless"]
        if not args.record:
            command.append("--no-record")
        run = subprocess.run(command, capture_output=True, text=True)
        if run.returncode not in (0, 2, 3):
            print(run.stderr, file=sys.stderr)
            return 1
        m = json.loads(run.stdout)
        row = dict(scenario=name, outcome=m["outcome"], goals=m["goal_completion"], robots=m["robot_count"], collisions=m["collisions"], clearance_violations=m["clearance_violations"], time=m["simulation_time"])
        results.append(row)
        print(f"{name:26} {row['outcome']:20} {row['goals']}/{row['robots']:<5} {row['collisions']:<5} {row['clearance_violations']:<6} {row['time']:.2f}", flush=True)
    (args.output / "aggregate.json").write_text(json.dumps(results, indent=2) + "\n")
    return int(any(r["collisions"] or r["clearance_violations"] or
                   (r["scenario"] in PRIMARY and r["scenario"] != "head_on_narrow" and r["outcome"] != "completed")
                   for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
