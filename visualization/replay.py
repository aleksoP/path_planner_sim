#!/usr/bin/env python3
"""Recording-only viewer. Contains no simulation or planning decisions."""
import argparse
import bisect
import json
from pathlib import Path


def load_run(directory):
    directory = Path(directory)
    config = json.loads((directory / "run_config.json").read_text())
    metrics = json.loads((directory / "metrics.json").read_text())
    frames = [json.loads(line) for line in (directory / "trace.jsonl").read_text().splitlines() if line]
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines() if line]
    if not frames:
        raise ValueError("Run has no playback frames; rerun without --no-record")
    if any(b["time"] <= a["time"] for a, b in zip(frames, frames[1:])):
        raise ValueError("Trace timestamps must strictly increase")
    ids = {r["id"] for r in config["robots"]}
    if any({r["id"] for r in f["robots"]} != ids for f in frames):
        raise ValueError("Trace robot identifiers do not match configuration")
    return config, metrics, frames, events


class Replay:
    def __init__(self, data):
        import matplotlib.pyplot as plt
        from matplotlib import animation
        from matplotlib.widgets import Button, CheckButtons, Slider
        self.plt = plt
        self.config, self.metrics, self.frames, self.events = data
        self.times = [f["time"] for f in self.frames]
        self.index = 0
        self.playing = False
        self.changing = False
        self.fig, self.ax = plt.subplots(figsize=(13, 8))
        self.fig.subplots_adjust(left=.07, right=.76, bottom=.21, top=.93)
        self.definitions = {r["id"]: r for r in self.config["robots"]}
        colors = ["#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]
        self.colors = {name: colors[i % len(colors)] for i, name in enumerate(self.definitions)}
        self.plan_records = {name: [] for name in self.definitions}
        for event in self.events:
            if event.get("type") == "plan":
                self.plan_records[event["robot"]].append(event)
        self.plan_times = {name: [p["time"] for p in plans] for name, plans in self.plan_records.items()}
        self.info = self.fig.text(.78, .91, "", va="top", fontsize=8, family="monospace")
        self.timeline = Slider(self.fig.add_axes([.10, .12, .61, .025]), "Time", 0, max(.001, self.times[-1]), valinit=0)
        self.speed = Slider(self.fig.add_axes([.10, .06, .25, .025]), "Speed", .25, 4, valinit=1, valstep=.25)
        self.buttons = []
        for label, x, callback in [("Play/Pause", .40, self.toggle), ("Step", .52, self.step), ("Reset view", .62, self.reset_view)]:
            button = Button(self.fig.add_axes([x, .045, .09, .045]), label)
            button.on_clicked(callback)
            self.buttons.append(button)
        self.overlays = CheckButtons(self.fig.add_axes([.78, .08, .18, .11]), ["Plans", "Predictions", "Rejected"], [True, True, False])
        self.overlays.on_clicked(lambda _: self.draw())
        self.timeline.on_changed(self.seek)
        self.fig.canvas.mpl_connect("key_press_event", self.key)
        self.fig.suptitle(self.config["name"])
        self.view = None
        self.draw()
        self.animation = animation.FuncAnimation(self.fig, self.tick, interval=50, cache_frame_data=False)

    def toggle(self, _=None):
        self.playing = not self.playing

    def step(self, _=None):
        self.playing = False
        self.set_index(min(self.index + 1, len(self.frames) - 1))

    def key(self, event):
        if event.key == " ":
            self.toggle()
        elif event.key == "right":
            self.step()
        elif event.key == "left":
            self.playing = False
            self.set_index(max(0, self.index - 1))
        elif event.key == "home":
            self.set_index(0)
        elif event.key == "r":
            self.reset_view()

    def reset_view(self, _=None):
        self.view = None
        self.draw(reset=True)

    def seek(self, value):
        if not self.changing:
            self.set_index(max(0, bisect.bisect_right(self.times, value + 1e-9) - 1))

    def set_index(self, index, playback=False):
        self.index = index
        if not playback:
            self.play_time = self.times[index]
        self.changing = True
        self.timeline.set_val(self.times[index])
        self.changing = False
        self.draw()

    def tick(self, _):
        if self.playing:
            advance = .05 * self.speed.val
            self.play_time = max(getattr(self, "play_time", self.times[self.index]), self.times[self.index]) + advance
            self.set_index(min(len(self.frames) - 1, bisect.bisect_right(self.times, self.play_time + 1e-9) - 1), playback=True)
            if self.index == len(self.frames) - 1:
                self.playing = False
        else:
            self.play_time = self.times[self.index]

    def draw(self, reset=False):
        from matplotlib.patches import Polygon
        import math
        if self.view is not None and not reset:
            self.view = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.clear()
        frame = self.frames[self.index]
        now = frame["time"]
        walls = list(self.config["world_polygons"])
        for event in self.events:
            if event.get("type") == "add_obstacle" and event["applied_time"] <= now:
                obstacle = event["obstacle"]
                if "polygon" in obstacle:
                    walls.append(obstacle["polygon"])
                else:
                    walls.append(self.corners(obstacle["pose"], *obstacle["size"]))
        for vertices in walls:
            self.ax.add_patch(Polygon(vertices, facecolor=".75", edgecolor=".3"))
        plans_on, predictions_on, rejected_on = self.overlays.get_status()
        separation = frame["minimum_separation_so_far"]
        separation_text = f"{separation:.3f} m" if separation is not None else "n/a"
        text = [f"t = {now:.2f} s", f"Final: {self.metrics['outcome']}", f"Collisions: {frame['collisions']}",
                f"Min ever: {separation_text}", ""]
        for robot in frame["robots"]:
            name = robot["id"]
            definition, color = self.definitions[name], self.colors[name]
            path = self.config["effective_paths"][name]
            self.ax.plot(*zip(*path), color="royalblue", lw=1, alpha=.5, label="Reference" if name == next(iter(self.definitions)) else None)
            history = [next(r for r in f["robots"] if r["id"] == name)["pose"][:2] for f in self.frames[:self.index + 1]]
            self.ax.plot(*zip(*history), color=color, lw=1, label=f"{name} driven")
            f = definition["footprint"]
            self.ax.add_patch(Polygon(self.corners(robot["pose"], f["length"], f["width"]), facecolor=color, alpha=.8))
            x, y, yaw = robot["pose"]
            self.ax.arrow(x, y, .5 * math.cos(yaw), .5 * math.sin(yaw), width=.025, color="black", length_includes_head=True)
            self.ax.text(x, y + .55, name, ha="center", fontsize=9)
            pindex = bisect.bisect_right(self.plan_times[name], now) - 1
            plan = self.plan_records[name][pindex] if pindex >= 0 else None
            if plan:
                if plans_on:
                    self.ax.plot([p[1] for p in plan["trajectory"]], [p[2] for p in plan["trajectory"]], color=color, lw=2, alpha=.65)
                    intent = plan.get("maneuver_path", [])
                    if intent:
                        self.ax.plot(*zip(*intent), "-.", color=color, lw=.9, alpha=.65)
                        self.ax.plot(*intent[-1], "o", color=color, ms=5, mfc="white")
                if predictions_on:
                    for prediction in plan["predictions"]:
                        self.ax.plot([p[1] for p in prediction["trajectory"]], [p[2] for p in prediction["trajectory"]], "--", color=self.colors.get(prediction["id"], "gray"), alpha=.4)
                if rejected_on:
                    for candidate in plan["candidates"]:
                        if not candidate["feasible"] and candidate.get("trajectory"):
                            self.ax.plot([p[1] for p in candidate["trajectory"]], [p[2] for p in candidate["trajectory"]], ":", color="gray", alpha=.35)
            text += [f"{name}: {robot['state']}", f"  {robot['side']} / {robot['maneuver']}",
                     f"  v={robot['velocity'][0]:.2f} m/s", f"  cmd={robot['command'][0]:.2f} m/s", f"  distance={robot['distance']:.2f} m"]
            if plan:
                rejected = [c for c in plan["candidates"] if not c["feasible"]]
                text += [f"  conflict={plan['predicted_conflict']}", f"  rejected={len(rejected)}/{len(plan['candidates'])}"]
                if "observations" in plan:
                    seen = ",".join(o["id"]+("" if o["visible"] else "*") for o in plan["observations"]) or "none"
                    text += [f"  observed={seen}"]
                if plan.get("maneuver_path"):
                    text += ["  dash-dot: merge intent"]
                reason = plan["reason"]
                import textwrap
                text += textwrap.wrap(reason, 31)
            text.append("")
        for event in self.events:
            if event.get("type") == "collision" and event["time"] <= now:
                self.ax.plot(*event["location"], marker="x", color="magenta", ms=12, mew=3)
        self.info.set_text("\n".join(text))
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(alpha=.2)
        self.ax.legend(loc="upper left", fontsize=8)
        if self.view is None or reset:
            self.ax.autoscale_view()
            self.view = self.ax.get_xlim(), self.ax.get_ylim()
        else:
            self.ax.set_xlim(self.view[0]); self.ax.set_ylim(self.view[1])
        self.fig.canvas.draw_idle()

    @staticmethod
    def corners(pose, length, width):
        import math
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * u - s * v, y + s * u + c * v) for u, v in
                [(-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2), (-length / 2, width / 2)]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--snapshot", type=Path, help="Render a PNG without a window")
    parser.add_argument("--time", type=float, default=0)
    parser.add_argument("--validate", action="store_true", help="Validate recording without importing Matplotlib")
    args = parser.parse_args()
    data = load_run(args.run)
    if args.validate:
        print(f"Loaded {len(data[2])} frames, {len(data[3])} events; outcome={data[1]['outcome']}")
        return
    if args.snapshot:
        import matplotlib
        matplotlib.use("Agg")
    replay = Replay(data)
    replay.set_index(max(0, bisect.bisect_right(replay.times, args.time + 1e-9) - 1))
    if args.snapshot:
        replay.animation._draw_was_started = True  # Static rendering intentionally never starts a GUI timer.
        args.snapshot.parent.mkdir(parents=True, exist_ok=True)
        replay.fig.savefig(args.snapshot, dpi=150)
        replay.plt.close(replay.fig)
    else:
        replay.plt.show()


if __name__ == "__main__":
    main()
