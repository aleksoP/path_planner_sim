#!/usr/bin/env python3
"""Interactive box-map / multi-robot scenario editor using the existing Matplotlib stack."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import sys
import textwrap

from scenario_document import ROOT, DEFAULT_LIMITS, ScenarioDocument, align_headings, corners, numbers
from editor_runner import EditorRun

TOOLS = ["Select / move", "Wall box", "Obstacle box", "Place robot", "Draw path"]
COLORS = ["#cc3e38", "#20965c", "#8150b5", "#da851d", "#168b9d", "#a75482"]


class ScenarioEditor:
    def __init__(self, document, filename, binary=None, output_root=None, seed=42):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons, RadioButtons, TextBox
        self.plt, self.document = plt, document
        self.binary = Path(binary or ROOT/"build/bin/avoidance_sim")
        self.output_root = Path(output_root or ROOT/"runs")
        self.seed = seed
        self.selected = None  # (robot|obstacle, array index, optional waypoint index)
        self.drag = None
        self.preview = None
        self.draft = None
        self.last_run = None
        self.job = None
        self.grid_size = .25
        self.mode = TOOLS[0]
        self.buttons = []
        self.fields = []
        self.field_labels = []
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.canvas.manager.set_window_title("AMR Scenario Editor")
        # Ctrl+S must save a scenario, rather than invoke Matplotlib's Save Figure dialog.
        handler = getattr(self.fig.canvas.manager, "key_press_handler_id", None)
        if handler is not None:
            self.fig.canvas.mpl_disconnect(handler)
        self.ax = self.fig.add_axes([.055, .20, .665, .675])
        self.heading = self.fig.text(.055, .967, "AMR Scenario Editor", fontsize=17, weight="bold")
        self.fig.text(.055, .943, "Box geometry · robot footprints · reference paths     |     metres / degrees", fontsize=9, color=".35")
        for label, x, action in [
            ("Blank", .055, lambda: self.preset("blank")),
            ("Wide tunnel", .139, lambda: self.preset("wide")),
            ("Narrow tunnel", .223, lambda: self.preset("narrow")),
            ("Crossing", .307, lambda: self.preset("crossing")),
            ("L tunnel", .391, lambda: self.preset("l_shaped")),
            ("Undo", .475, lambda: self.history(False)),
            ("Redo", .559, lambda: self.history(True)),
            ("Fit map", .643, self.fit)]:
            self._button(label, [x, .895, .078, .032], action)
        self.radio = RadioButtons(self.fig.add_axes([.755, .727, .225, .17]), TOOLS, active=0)
        for label in self.radio.labels:
            label.set_fontsize(9)
        self.radio.on_clicked(self.change_tool)
        self.inspector_title = self.fig.text(.755, .703, "Select a robot or box", fontsize=10, weight="bold")
        for i in range(9):
            y = .652-i*.0425
            label = self.fig.text(.755, y+.027, "", fontsize=8, color=".3")
            field = TextBox(self.fig.add_axes([.755, y, .225, .024]), "", initial="")
            field.text_disp.set_fontsize(9)
            self.field_labels.append(label)
            self.fields.append(field)
        self._button("Apply properties", [.755, .25, .137, .035], self.apply_properties)
        self._button("Delete", [.901, .25, .079, .035], self.delete_selection)
        self._button("Finish path", [.755, .20, .105, .035], self.finish_path)
        self._button("Cancel path", [.875, .20, .105, .035], self.cancel_interaction)
        self.filename = TextBox(self.fig.add_axes([.095, .145, .625, .031]), "JSON  ", initial=str(filename))
        self.filename.text_disp.set_fontsize(9)
        for label, x, width, action in [
            ("Open", .055, .075, self.open_file), ("Save JSON", .14, .09, self.save_file),
            ("Run planner", .24, .11, self.run), ("Replay run", .36, .10, self.replay),
            ("Stop run", .47, .085, self.stop_run)]:
            self._button(label, [x, .093, width, .035], action)
        self.snap = CheckButtons(self.fig.add_axes([.57, .09, .15, .04]), ["Snap (Shift: off)"], [True])
        self.snap.labels[0].set_fontsize(8)
        self.name = TextBox(self.fig.add_axes([.095, .047, .335, .027]), "Name  ", initial=document.data["name"])
        self.name.text_disp.set_fontsize(9)
        self.duration = TextBox(self.fig.add_axes([.505, .047, .065, .027]), "Time [s]  ", initial=str(document.data.get("simulation", {}).get("duration", 35)))
        self.grid = TextBox(self.fig.add_axes([.655, .047, .065, .027]), "Grid [m]  ", initial="0.25")
        self.grid.on_submit(self.update_grid)
        self.status = self.fig.text(.055, .014, "", fontsize=9)
        self.help = self.fig.text(.755, .16, "", va="top", fontsize=8, linespacing=1.45)
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.fig.canvas.mpl_connect("close_event", self.on_close)
        self.timer = self.fig.canvas.new_timer(interval=200)
        self.timer.add_callback(self.poll_run)
        self.timer.start()
        self.ax.set_xlim(-12, 12)
        self.ax.set_ylim(-6, 6)
        self.fit()
        self.set_status("Drag boxes; place robots; select a robot and draw its path. Ctrl+S saves.")

    def _button(self, label, rectangle, action):
        from matplotlib.widgets import Button
        button = Button(self.fig.add_axes(rectangle), label)
        button.label.set_fontsize(9)
        button.on_clicked(lambda _: self.guard(action))
        self.buttons.append(button)

    def guard(self, action):
        try:
            return action()
        except (ValueError, OSError, KeyError, TypeError, IndexError) as exc:
            self.document.cancel()
            self.drag = self.preview = None
            self.set_status(str(exc), error=True)
            self.draw()
            return None

    def set_status(self, message, error=False):
        # The full error remains readable in the terminal; the footer stays in its allotted width.
        self.status.set_text(textwrap.shorten(message.replace("\n", " "), width=175, placeholder="…"))
        self.status.set_color("#b32d25" if error else "#264c60")
        if error:
            print(message, file=sys.stderr)
        self.fig.canvas.draw_idle()

    def _sync_metadata(self):
        name = self.name.text.strip()
        duration = numbers(self.duration.text, 1, "Duration")[0]
        if not name or duration <= 0:
            raise ValueError("Enter a nonempty name and positive simulation duration")
        def update():
            self.document.data["name"] = name
            self.document.data.setdefault("simulation", {})["duration"] = duration
        self.document.edit(update)

    def _sync_metadata_fields(self):
        self.name.set_val(self.document.data["name"])
        self.duration.set_val(str(self.document.data.get("simulation", {}).get("duration", 35)))

    def update_grid(self, value):
        def update():
            size = numbers(value, 1, "Grid spacing")[0]
            if not .01 <= size <= 10:
                raise ValueError("Grid spacing must be between 0.01 and 10 m")
            self.grid_size = size
            self.draw()
        self.guard(update)

    def point(self, event):
        if event.xdata is None or event.ydata is None:
            return None
        p = [float(event.xdata), float(event.ydata)]
        if self.snap.get_status()[0] and "shift" not in (event.key or "").lower():
            p = [round(v/self.grid_size)*self.grid_size for v in p]
        return p

    def _selection_valid(self):
        if self.selected is None:
            return False
        kind, index, waypoint = self.selected
        items = self.document.data["robots" if kind == "robot" else "obstacles"]
        if index >= len(items) or (kind == "robot" and waypoint is not None and waypoint >= len(items[index]["path"])):
            self.selected = None
            return False
        return True

    def _rotation_handle(self, box):
        x, y, yaw = box["pose"]
        extra = abs(self.ax.get_xlim()[1]-self.ax.get_xlim()[0])/max(1, self.ax.bbox.width)*32
        distance = box["size"][0]/2+extra
        return [x+math.cos(yaw)*distance, y+math.sin(yaw)*distance]

    def draw(self, inspector=False):
        import matplotlib.patches as patches
        from matplotlib.ticker import MultipleLocator
        view = self.ax.get_xlim(), self.ax.get_ylim()
        self.ax.clear()
        self._selection_valid()
        for i, box in enumerate(self.document.data["obstacles"]):
            selected = self.selected is not None and self.selected[:2] == ("obstacle", i)
            vertices = box["polygon"] if "polygon" in box else corners(box["pose"], box["size"])
            kind = box.get("editor", {}).get("kind", "wall")
            self.ax.add_patch(patches.Polygon(vertices, facecolor="#d7bb79" if kind == "obstacle" else "#aeb9c1",
                edgecolor="#237baf" if selected else "#52606b", linewidth=2 if selected else 1, zorder=2))
            if selected and "polygon" not in box:
                self.ax.plot(*zip(*vertices), "s", ms=6, mfc="white", mec="#237baf", zorder=10)
                knob = self._rotation_handle(box)
                self.ax.plot([box["pose"][0], knob[0]], [box["pose"][1], knob[1]], color="#237baf", lw=.8, zorder=5)
                self.ax.plot(*knob, "o", ms=7, mfc="white", mec="#237baf", zorder=10)
        for i, robot in enumerate(self.document.data["robots"]):
            color = COLORS[i % len(COLORS)]
            selected = self.selected is not None and self.selected[:2] == ("robot", i)
            path = robot["path"]
            self.ax.plot(*zip(*path), "--", lw=1.7 if selected else 1., color=color, alpha=.8, zorder=3)
            f = robot["footprint"]
            self.ax.add_patch(patches.Polygon(corners(robot["start"], [f["length"], f["width"]]),
                facecolor=color, edgecolor="#111" if selected else color, linewidth=2 if selected else 1, alpha=.85, zorder=6))
            x, y, yaw = robot["start"]
            self.ax.arrow(x, y, .65*math.cos(yaw), .65*math.sin(yaw), width=.025,
                          color="black", length_includes_head=True, zorder=8)
            self.ax.text(x, y+.55, robot["id"], fontsize=9, ha="center", color=color, weight="bold", zorder=8)
            goal = robot["goal"]
            self.ax.add_patch(patches.Polygon(corners(goal, [f["length"], f["width"]]),
                fill=False, edgecolor=color, linestyle=":", alpha=.7, zorder=4))
            self.ax.plot(*goal[:2], "*", color=color, ms=12, zorder=7)
            if selected:
                self.ax.plot(*zip(*path), "o", ms=6, mfc="white", mec=color, zorder=10)
                if self.selected[2] is not None:
                    self.ax.plot(*path[self.selected[2]], "o", ms=9, mfc=color, mec="white", zorder=11)
        if self.preview is not None:
            a, b = self.preview
            self.ax.add_patch(patches.Polygon(corners([(a[0]+b[0])/2, (a[1]+b[1])/2, 0],
                [abs(b[0]-a[0]), abs(b[1]-a[1])]), facecolor="#68aaca", edgecolor="#237baf", alpha=.4, zorder=9))
        if self.draft is not None:
            self.ax.plot(*zip(*self.draft), "o-", color="#12689d", lw=2, ms=5, zorder=12)
        self.ax.set_xlim(*view[0]); self.ax.set_ylim(*view[1])
        self.ax.set_aspect("equal", adjustable="box")
        step = self.grid_size
        while max(view[0][1]-view[0][0], view[1][1]-view[1][0])/step > 80:
            step *= 2
        self.ax.xaxis.set_major_locator(MultipleLocator(step))
        self.ax.yaxis.set_major_locator(MultipleLocator(step))
        # Sparse labels; the snap grid remains visible without filling the map with numbers.
        from matplotlib.ticker import FuncFormatter
        self.ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g}" if abs(x-round(x)) < 1e-6 else ""))
        self.ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x:g}" if abs(x-round(x)) < 1e-6 else ""))
        self.ax.grid(alpha=.18)
        self.ax.set_xlabel("x [m]"); self.ax.set_ylabel("y [m]")
        count = f"{len(self.document.data['robots'])} robots · {len(self.document.data['obstacles'])} boxes / polygons"
        self.ax.set_title(count+"     "+("● Unsaved" if self.document.dirty else "Saved"), fontsize=9, loc="left")
        if inspector:
            self.refresh_inspector()
        self.help.set_text({
            "Select / move": "Drag bodies or waypoint handles.\nBox corners resize; circle rotates.\nDouble-click a path to add a point.\nDelete removes the selected item.\nMouse wheel zooms; toolbar pans.",
            "Wall box": "Drag opposite corners to build a wall.\nWalls are physical collision geometry.\nUse properties for exact sizes / angle.",
            "Obstacle box": "Drag opposite corners to add a box.\nObstacle and wall boxes both collide.\nUse properties for exact sizes / angle.",
            "Place robot": "Click to place a new robot.\nSelect Draw path to choose its route.\nProperties set size, speed and limits.",
            "Draw path": "Select a robot, then click waypoints.\nEnter / right-click: finish.\nBackspace: remove last draft point.\nEscape: discard this path draft.\nLast point becomes the goal."}[self.mode])
        self.fig.canvas.draw_idle()

    def refresh_inspector(self):
        entries = []
        if self._selection_valid():
            kind, index, waypoint = self.selected
            if kind == "obstacle":
                box = self.document.data["obstacles"][index]
                if "polygon" in box:
                    self.inspector_title.set_text(f"Polygon {index+1} (move / delete)")
                else:
                    self.inspector_title.set_text(f"Box {index+1}")
                    x, y, yaw = box["pose"]
                    entries = [("Center x, y, angle [deg]", f"{x:g}, {y:g}, {math.degrees(yaw):g}"),
                               ("Length, width [m]", ", ".join(f"{v:g}" for v in box["size"])),
                               ("Kind: wall / obstacle", box.get("editor", {}).get("kind", "wall"))]
            else:
                r = self.document.data["robots"][index]
                self.inspector_title.set_text(f"Robot {r['id']}"+(f" / point {waypoint+1}" if waypoint is not None else ""))
                def pose_string(p):
                    return f"{p[0]:g}, {p[1]:g}, {math.degrees(p[2]):g}"
                l = dict(DEFAULT_LIMITS, **r.get("limits", {}))
                entries = [("Robot ID", r["id"]), ("Start x, y, heading [deg]", pose_string(r["start"])),
                    ("Goal x, y, heading [deg]", pose_string(r["goal"])),
                    ("Footprint length, width [m]", f"{r['footprint']['length']:g}, {r['footprint']['width']:g}"),
                    ("Initial v [m/s], omega [rad/s]", ", ".join(f"{v:g}" for v in r.get("velocity", [0, 0]))),
                    ("Max forward, reverse [m/s]", f"{l['max_forward']:g}, {l['max_reverse']:g}"),
                    ("Max omega [rad/s], curvature [1/m]", f"{l['max_angular']:g}, {l['max_curvature']:g}"),
                    ("Accel, brake [m/s²], angular accel", f"{l['acceleration']:g}, {l['deceleration']:g}, {l['angular_acceleration']:g}"),
                    ("Cooperative (true/false), smoothing passes", f"{str(r.get('cooperative', True)).lower()}, {r.get('smoothing_passes', 3)}")]
        else:
            self.inspector_title.set_text("Select a robot or box")
        for i, field in enumerate(self.fields):
            visible = i < len(entries)
            field.ax.set_visible(visible)
            self.field_labels[i].set_visible(visible)
            if visible:
                self.field_labels[i].set_text(entries[i][0])
                field.set_val(entries[i][1])

    def apply_properties(self):
        if not self._selection_valid():
            raise ValueError("Select a robot or box first")
        self.cancel_interaction(quiet=True)
        kind, index, _ = self.selected
        def apply():
            if kind == "obstacle":
                box = self.document.data["obstacles"][index]
                if "polygon" in box:
                    raise ValueError("Imported polygons can be moved and deleted; use boxes for resizing")
                x, y, degrees = numbers(self.fields[0].text, 3, "Box pose")
                size = numbers(self.fields[1].text, 2, "Box size")
                label = self.fields[2].text.strip().lower()
                if min(size) <= 0 or label not in ("wall", "obstacle"):
                    raise ValueError("Use positive dimensions and kind wall or obstacle")
                box.update(pose=[x, y, math.radians(degrees)], size=size)
                box.setdefault("editor", {})["kind"] = label
            else:
                r = self.document.data["robots"][index]
                self.document.rename_robot(index, self.fields[0].text.strip())
                start = numbers(self.fields[1].text, 3, "Start pose")
                goal = numbers(self.fields[2].text, 3, "Goal pose")
                start[2], goal[2] = math.radians(start[2]), math.radians(goal[2])
                r["start"], r["goal"] = start, goal
                r["path"][0], r["path"][-1] = start[:2], goal[:2]
                length, width = numbers(self.fields[3].text, 2, "Footprint")
                r["footprint"] = dict(length=length, width=width)
                r["velocity"] = numbers(self.fields[4].text, 2, "Initial velocity")
                l = dict(DEFAULT_LIMITS, **r.get("limits", {}))
                l["max_forward"], l["max_reverse"] = numbers(self.fields[5].text, 2, "Speed limits")
                l["max_angular"], l["max_curvature"] = numbers(self.fields[6].text, 2, "Steering limits")
                l["acceleration"], l["deceleration"], l["angular_acceleration"] = numbers(self.fields[7].text, 3, "Acceleration limits")
                r["limits"] = l
                flags = self.fields[8].text.replace(",", " ").split()
                if len(flags) != 2 or flags[0].lower() not in ("true", "false"):
                    raise ValueError("Enter cooperative true/false and an integer smoothing pass count")
                passes = int(flags[1])
                if not 0 <= passes <= 8:
                    raise ValueError("Smoothing passes must be between 0 and 8")
                r["cooperative"], r["smoothing_passes"] = flags[0].lower() == "true", passes
            self.document.validate(self.document.data, allow_empty=True)
        self.document.edit(apply)
        self.draw(inspector=True)
        self.set_status("Properties applied. Dragging path endpoints realigns headings to the path.")

    def fit(self):
        points = []
        for box in self.document.data["obstacles"]:
            points += box["polygon"] if "polygon" in box else corners(box["pose"], box["size"])
        for r in self.document.data["robots"]:
            points += r["path"]+[r["start"][:2], r["goal"][:2]]
        if points:
            xs, ys = zip(*points)
            self.ax.set_xlim(min(xs)-1.2, max(xs)+1.2)
            self.ax.set_ylim(min(ys)-1.2, max(ys)+1.2)
        else:
            self.ax.set_xlim(-12, 12); self.ax.set_ylim(-6, 6)
        self.draw(inspector=True)

    def cancel_interaction(self, quiet=False):
        self.document.cancel()
        self.drag = self.preview = self.draft = None
        if not quiet:
            self.set_status("Draft cancelled; existing geometry/path retained.")
        self.draw()

    def change_tool(self, label):
        self.cancel_interaction(quiet=True)
        self.mode = label
        if label == "Draw path" and self._selection_valid() and self.selected[0] == "robot":
            self.draft = [self.document.data["robots"][self.selected[1]]["start"][:2]]
        self.draw(inspector=True)
        self.set_status(f"Tool: {label}")

    def preset(self, name):
        self.cancel_interaction(quiet=True)
        self.document.preset(name)
        self.selected = None
        self._sync_metadata_fields()
        self.radio.set_active(0)
        self.fit()
        self.set_status(f"Loaded {name} template. Undo restores the previous map; save to your own JSON file.")

    def history(self, redo=False):
        self.cancel_interaction(quiet=True)
        changed = self.document.redo() if redo else self.document.undo()
        self.selected = None
        self._sync_metadata_fields()
        self.draw(inspector=True)
        self.set_status(("Redo" if redo else "Undo")+(" applied" if changed else ": no history"))

    def delete_selection(self):
        self.cancel_interaction(quiet=True)
        if not self._selection_valid():
            raise ValueError("Select an item first")
        kind, index, point = self.selected
        self.document.edit(lambda: self.document.delete(kind, index, point))
        self.selected = None
        self.draw(inspector=True)
        self.set_status("Selection deleted (Undo restores it). Deleting a robot also removes its linked events.")

    def finish_path(self):
        if self.draft is None or not self._selection_valid() or self.selected[0] != "robot":
            raise ValueError("Select a robot, choose Draw path, and click destinations first")
        points = deepcopy(self.draft)
        self.document.edit(lambda: self.document.replace_path(self.selected[1], points))
        self.draft = None
        self.radio.set_active(0)
        self.draw(inspector=True)
        self.set_status("Path saved: first point is start; last point is goal. Drag handles to refine it.")

    def _pixel_distance(self, point, event):
        px, py = self.ax.transData.transform(point)
        return math.hypot(px-event.x, py-event.y)

    def hit(self, event):
        from matplotlib.path import Path as MplPath
        # Opposing missions exchange start/goal positions. Pick solid start bodies
        # before another robot's coincident goal handle or overlapping path.
        for i in reversed(range(len(self.document.data["robots"]))):
            robot = self.document.data["robots"][i]
            footprint = robot["footprint"]
            if MplPath(corners(robot["start"], [footprint["length"], footprint["width"]])).contains_point((event.xdata, event.ydata)):
                return ("robot", i, None), "waypoint", 0
        if self._selection_valid():
            kind, index, _ = self.selected
            if kind == "robot":
                r = self.document.data["robots"][index]
                for i, point in reversed(list(enumerate(r["path"]))):
                    if self._pixel_distance(point, event) < 10:
                        return (kind, index, i), "waypoint", i
            else:
                box = self.document.data["obstacles"][index]
                if "polygon" not in box:
                    if self._pixel_distance(self._rotation_handle(box), event) < 11:
                        return (kind, index, None), "rotate", None
                    for i, point in enumerate(corners(box["pose"], box["size"])):
                        if self._pixel_distance(point, event) < 10:
                            return (kind, index, None), "resize", i
        for i in reversed(range(len(self.document.data["robots"]))):
            r = self.document.data["robots"][i]
            if self._pixel_distance(r["goal"][:2], event) < 10:
                return ("robot", i, len(r["path"])-1), "waypoint", len(r["path"])-1
            for j, (a, b) in enumerate(zip(r["path"], r["path"][1:])):
                a, b = self.ax.transData.transform(a), self.ax.transData.transform(b)
                dx, dy = b-a
                u = max(0, min(1, ((event.x-a[0])*dx+(event.y-a[1])*dy)/max(1e-10, dx*dx+dy*dy)))
                if math.hypot(event.x-a[0]-u*dx, event.y-a[1]-u*dy) < 7:
                    return ("robot", i, None), "segment", j
        for i in reversed(range(len(self.document.data["obstacles"]))):
            box = self.document.data["obstacles"][i]
            vertices = box["polygon"] if "polygon" in box else corners(box["pose"], box["size"])
            if MplPath(vertices).contains_point((event.xdata, event.ydata)):
                return ("obstacle", i, None), "translate", None
        return None

    def _navigation_active(self):
        toolbar = self.fig.canvas.toolbar
        return toolbar is not None and bool(toolbar.mode)

    def on_press(self, event):
        if event.inaxes != self.ax or self._navigation_active():
            return
        self.guard(lambda: self._press(event))

    def _press(self, event):
        if event.button == 3 and self.mode == "Draw path":
            self.finish_path()
            return
        if event.button != 1:
            return
        point = self.point(event)
        if self.mode in ("Wall box", "Obstacle box"):
            self.preview = (point, point)
            self.drag = dict(type="create", origin=point)
        elif self.mode == "Place robot":
            index = self.document.edit(lambda: self.document.add_robot(point))
            self.selected = ("robot", index, None)
            self.draw(inspector=True)
            self.set_status("Robot placed. Choose Draw path, then click its destinations.")
        elif self.mode == "Draw path":
            if self.draft is None:
                hit = self.hit(event)
                if hit is None or hit[0][0] != "robot":
                    raise ValueError("Click a robot first to start drawing its path")
                self.selected = ("robot", hit[0][1], None)
                self.draft = [self.document.data["robots"][hit[0][1]]["start"][:2]]
            elif math.dist(point, self.draft[-1]) > 1e-8:
                self.draft.append(point)
            self.draw(inspector=True)
        else:
            hit = self.hit(event)
            if hit is None:
                self.selected = None
                self.draw(inspector=True)
                return
            selection, action, detail = hit
            self.selected = selection
            if event.dblclick and action == "segment":
                r = self.document.data["robots"][selection[1]]
                self.document.edit(lambda: r["path"].insert(detail+1, point))
                self.selected = ("robot", selection[1], detail+1)
            elif action != "segment":
                self.document.begin()
                items = self.document.data["robots" if selection[0] == "robot" else "obstacles"]
                self.drag = dict(type=action, origin=point, original=deepcopy(items[selection[1]]), detail=detail)
            self.draw(inspector=True)

    def on_motion(self, event):
        if self.drag is None or event.inaxes != self.ax:
            return
        self.guard(lambda: self._motion(event))

    def _motion(self, event):
        p = self.point(event)
        d = self.drag
        if d["type"] == "create":
            self.preview = (d["origin"], p)
        else:
            kind, index, _ = self.selected
            if d["type"] == "waypoint":
                original = d["original"]["path"][d["detail"]]
                moved = [original[i]+p[i]-d["origin"][i] for i in (0, 1)]
                self.document.move_waypoint(index, d["detail"], moved)
            else:
                box = self.document.data["obstacles"][index]
                original = d["original"]
                if d["type"] == "translate":
                    delta = [p[i]-d["origin"][i] for i in (0, 1)]
                    if "polygon" in box:
                        box["polygon"] = [[v[i]+delta[i] for i in (0, 1)] for v in original["polygon"]]
                    else:
                        box["pose"][:2] = [original["pose"][i]+delta[i] for i in (0, 1)]
                elif d["type"] == "rotate":
                    angle = math.atan2(p[1]-original["pose"][1], p[0]-original["pose"][0])
                    if self.snap.get_status()[0] and "shift" not in (event.key or ""):
                        angle = round(angle/(math.pi/12))*(math.pi/12)
                    box["pose"][2] = angle
                else:
                    anchor = corners(original["pose"], original["size"])[(d["detail"]+2) % 4]
                    yaw = original["pose"][2]
                    dx, dy = p[0]-anchor[0], p[1]-anchor[1]
                    size = [abs(math.cos(yaw)*dx+math.sin(yaw)*dy), abs(-math.sin(yaw)*dx+math.cos(yaw)*dy)]
                    if min(size) >= .05:
                        box["pose"] = [(p[0]+anchor[0])/2, (p[1]+anchor[1])/2, yaw]
                        box["size"] = size
        self.draw()

    def on_release(self, event):
        if self.drag is None:
            return
        def release():
            if event.inaxes == self.ax:
                self._motion(event)
            if self.drag["type"] == "create":
                a, b = self.preview
                index = self.document.edit(lambda: self.document.add_box(a, b, "wall" if self.mode == "Wall box" else "obstacle"))
                self.selected = ("obstacle", index, None)
            else:
                self.document.validate(self.document.data, allow_empty=True)
                self.document.commit()
            self.drag = self.preview = None
            self.draw(inspector=True)
        self.guard(release)

    def on_scroll(self, event):
        if event.inaxes != self.ax or self.drag is not None:
            return
        factor = .8 if event.button == "up" else 1.25
        x, y = event.xdata, event.ydata
        xlim, ylim = self.ax.get_xlim(), self.ax.get_ylim()
        if not .5 <= (xlim[1]-xlim[0])*factor <= 10000:
            return
        self.ax.set_xlim(x+(xlim[0]-x)*factor, x+(xlim[1]-x)*factor)
        self.ax.set_ylim(y+(ylim[0]-y)*factor, y+(ylim[1]-y)*factor)
        self.draw()

    def on_key(self, event):
        if event.key == "ctrl+s":
            self.guard(self.save_file)
            return
        text_axes = [f.ax for f in self.fields]+[self.filename.ax, self.name.ax, self.duration.ax, self.grid.ax]
        if event.inaxes in text_axes:
            return
        key = event.key or ""
        actions = {"ctrl+s": self.save_file, "ctrl+z": lambda: self.history(False),
                   "ctrl+y": lambda: self.history(True), "ctrl+shift+z": lambda: self.history(True),
                   "delete": self.delete_selection, "escape": self.cancel_interaction,
                   "enter": self.finish_path, "f": self.fit}
        if key in actions:
            self.guard(actions[key])
        elif key in ("1", "2", "3", "4", "5"):
            self.radio.set_active(int(key)-1)
        elif key == "backspace" and self.draft is not None and len(self.draft) > 1:
            self.draft.pop()
            self.draw()

    def open_file(self):
        self.cancel_interaction(quiet=True)
        self.document.load(self.filename.text)
        self.selected = None
        self._sync_metadata_fields()
        self.radio.set_active(0)
        self.fit()
        self.set_status("Loaded JSON, including planner parameters and timed events. Undo restores the prior map.")

    def save_file(self):
        if self.draft is not None or self.drag is not None:
            raise ValueError("Finish or cancel the active drawing before saving")
        self._sync_metadata()
        path = self.document.save(self.filename.text, allow_empty=True)
        self.draw()
        suffix = " (map draft; add robots before running)" if not self.document.data["robots"] else ""
        self.set_status(f"Saved {path}{suffix}")

    def run(self):
        if self.job is not None:
            raise ValueError("A run is in progress; stop it or wait for completion")
        if self.draft is not None or self.drag is not None:
            raise ValueError("Finish or cancel the active drawing before running")
        self._sync_metadata()
        self.job = EditorRun(self.document.data, self.binary, self.output_root, self.seed)
        self.set_status(f"Running C++ planner on a snapshot. You can keep editing. Output: {self.job.directory.name}")

    def poll_run(self):
        if self.job is None:
            return
        outcome = self.job.poll()
        if outcome is None:
            return
        directory = self.job.directory
        cancelled = self.job.cancelled
        self.job = None
        if outcome.metrics is None:
            self.set_status("Run cancelled" if cancelled else outcome.log, error=not cancelled)
            return
        self.last_run = directory
        m = outcome.metrics
        self.set_status(f"{m['outcome']} · goals {m['goal_completion']}/{m['robot_count']} · collisions {m['collisions']} · "
                        f"clearance violations {m['clearance_violations']} · {m['simulation_time']:.2f} s. Replay run opens this snapshot.",
                        error=m["collisions"] > 0 or m["clearance_violations"] > 0)

    def stop_run(self):
        if self.job is not None:
            self.job.stop()
            self.set_status("Stopping the current run; the editor document is retained.")

    def replay(self):
        if self.last_run is None:
            raise ValueError("Run a scenario first; Replay opens its recorded result in a separate window")
        subprocess.Popen([sys.executable, str(ROOT/"visualization/replay.py"), "--run", str(self.last_run)])

    def on_close(self, _):
        self.timer.stop()
        if self.job is not None:
            self.job.stop()
            try:
                self.job.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.job.process.kill()
                self.job.process.wait()
        if self.document.dirty:
            print("Editor closed with unsaved edits. Use Save JSON before closing to retain a scenario.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, help="Load any existing simulator scenario")
    parser.add_argument("--output", type=Path, help="Initial save-file path (editable in the UI)")
    parser.add_argument("--preset", choices=["blank", "wide", "narrow", "crossing", "l_shaped"], default="wide")
    parser.add_argument("--binary", type=Path, default=ROOT/"build/bin/avoidance_sim")
    parser.add_argument("--runs", type=Path, default=ROOT/"runs")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--snapshot", type=Path, help="Render the editor to PNG without a window")
    args = parser.parse_args()
    document = ScenarioDocument()
    if args.scenario:
        document.load(args.scenario)
    else:
        document.preset(args.preset)
    filename = args.output or (args.scenario.with_name(args.scenario.stem+"_edited.json") if args.scenario else ROOT/"scenarios/custom_environment.json")
    if args.snapshot:
        import matplotlib
        matplotlib.use("Agg")
    editor = ScenarioEditor(document, filename, args.binary, args.runs, args.seed)
    if args.snapshot:
        args.snapshot.parent.mkdir(parents=True, exist_ok=True)
        editor.fig.savefig(args.snapshot, dpi=130)
        editor.timer.stop()
        editor.plt.close(editor.fig)
    else:
        import matplotlib
        if matplotlib.get_backend().lower() in ("agg", "pdf", "svg", "ps", "cairo", "pgf", "template", "module://matplotlib_inline.backend_inline"):
            parser.error("No desktop Matplotlib backend is available. Run in a graphical desktop with Tk/Qt support, or use --snapshot FILE.png for headless preview.")
        editor.plt.show()


if __name__ == "__main__":
    main()
