"""Scenario authoring data, transactions and JSON I/O. No planner or simulator logic."""
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LIMITS = dict(max_forward=1.5, max_reverse=.3, max_angular=1.5,
                      max_curvature=2.5, acceleration=.8, deceleration=1.2,
                      angular_acceleration=2.)


def numbers(value, count, label):
    """Parse comma/space-separated property fields without evaluating expressions."""
    try:
        result = [float(x) for x in value.replace(",", " ").split()]
    except (ValueError, AttributeError) as exc:
        raise ValueError(f"{label}: enter {count} numbers separated by commas") from exc
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError(f"{label}: enter {count} finite numbers")
    return result


def corners(pose, size):
    x, y, yaw = pose
    length, width = size
    c, s = math.cos(yaw), math.sin(yaw)
    return [[x + c * u - s * v, y + s * u + c * v] for u, v in
            [(-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2)]]


def align_headings(robot):
    path = robot["path"]
    for pose, pairs in ((robot["start"], zip(path, path[1:])),
                        (robot["goal"], zip(reversed(path[:-1]), reversed(path[1:])))):
        for a, b in pairs:
            if math.dist(a, b) > 1e-8:
                pose[2] = math.atan2(b[1]-a[1], b[0]-a[0])
                break


def blank_scenario():
    return dict(name="Custom environment", simulation=dict(timestep=.02, planner_period=.1,
                control_period=.02, record_period=.1, duration=35.),
                observations=dict(ground_truth=True, range=30.), planner={},
                obstacles=[], robots=[], events=[])


class ScenarioDocument:
    """Unknown scenario/event/robot fields survive edits and round trips."""
    def __init__(self, data=None):
        self.data = deepcopy(data if data is not None else blank_scenario())
        self._undo = []
        self._redo = []
        self._before = None
        self.saved = deepcopy(self.data)

    @property
    def dirty(self):
        return self.data != self.saved

    def begin(self):
        if self._before is not None:
            raise RuntimeError("an editor transaction is already active")
        self._before = deepcopy(self.data)

    def commit(self):
        if self._before is not None and self.data != self._before:
            self._undo.append(self._before)
            self._undo = self._undo[-100:]
            self._redo.clear()
        self._before = None

    def cancel(self):
        if self._before is not None:
            self.data = self._before
        self._before = None

    def edit(self, operation):
        self.begin()
        try:
            result = operation()
            self.commit()
            return result
        except Exception:
            self.cancel()
            raise

    def undo(self):
        self.cancel()
        if not self._undo:
            return False
        self._redo.append(deepcopy(self.data))
        self.data = self._undo.pop()
        return True

    def redo(self):
        self.cancel()
        if not self._redo:
            return False
        self._undo.append(deepcopy(self.data))
        self.data = self._redo.pop()
        return True

    def replace(self, data):
        replacement = deepcopy(data)
        self.validate(replacement, allow_empty=True)
        self.edit(lambda: setattr(self, "data", replacement))

    def load(self, path):
        data = json.loads(Path(path).expanduser().read_text())
        self.replace(data)
        self.saved = deepcopy(self.data)

    def save(self, path, allow_empty=False):
        self.validate(self.data, allow_empty=allow_empty)
        destination = Path(path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        # Same-directory replace prevents a partially written scenario on interrupted save.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", dir=destination.parent, suffix=".tmp", delete=False) as handle:
                temporary = Path(handle.name)
                json.dump(self.data, handle, indent=2, allow_nan=False)
                handle.write("\n")
            temporary.replace(destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        self.saved = deepcopy(self.data)
        return destination

    @staticmethod
    def validate(data, allow_empty=False):
        """Authoring structure validation; C++ remains authoritative for run validation."""
        if not isinstance(data, dict) or not isinstance(data.get("name"), str) or not data["name"].strip():
            raise ValueError("Scenario needs a nonempty name")
        def vector(value, size, label):
            if not isinstance(value, list) or len(value) != size or any(
                    isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
                raise ValueError(f"{label} must contain {size} finite numbers")
        obstacles = data.get("obstacles")
        robots = data.get("robots")
        if not isinstance(obstacles, list) or not isinstance(robots, list):
            raise ValueError("Scenario must contain robots and obstacles arrays")
        if not robots and not allow_empty:
            raise ValueError("Place at least one robot and give it a path before saving/running")
        for i, obstacle in enumerate(obstacles):
            if "polygon" in obstacle:
                if len(obstacle["polygon"]) < 3:
                    raise ValueError(f"Obstacle {i}: polygon needs at least three vertices")
                for point in obstacle["polygon"]:
                    vector(point, 2, f"Obstacle {i} vertex")
            else:
                vector(obstacle.get("pose"), 3, f"Box {i} pose")
                vector(obstacle.get("size"), 2, f"Box {i} size")
                if min(obstacle["size"]) <= 0:
                    raise ValueError(f"Box {i}: dimensions must be positive")
        ids = set()
        for robot in robots:
            name = robot.get("id")
            if not isinstance(name, str) or not name.strip() or name in ids:
                raise ValueError("Robot IDs must be unique and nonempty")
            ids.add(name)
            vector(robot.get("start"), 3, f"{name} start")
            vector(robot.get("goal"), 3, f"{name} goal")
            path = robot.get("path")
            if not isinstance(path, list) or len(path) < 2:
                raise ValueError(f"{name}: draw a path with at least two points")
            for point in path:
                vector(point, 2, f"{name} waypoint")
            if not any(math.dist(path[0], p) > 1e-8 for p in path[1:]):
                raise ValueError(f"{name}: path must contain two distinct points")
            if math.dist(path[-1], robot["goal"][:2]) > .001:
                raise ValueError(f"{name}: path endpoint must equal goal")
            footprint = robot.get("footprint", {})
            for key in ("length", "width"):
                x = footprint.get(key, 0)
                if not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0:
                    raise ValueError(f"{name}: footprint {key} must be positive")
            limits = dict(DEFAULT_LIMITS, **robot.get("limits", {}))
            for key, x in limits.items():
                if not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0 or (key != "max_reverse" and x == 0):
                    raise ValueError(f"{name}: invalid {key}")
            velocity = robot.get("velocity", [0, 0])
            vector(velocity, 2, f"{name} initial velocity")
            if not -limits["max_reverse"] <= velocity[0] <= limits["max_forward"] or abs(velocity[1]) > limits["max_angular"]:
                raise ValueError(f"{name}: initial velocity exceeds configured limits")
        # Events may reference robots: renaming/deleting must never leave dangling IDs.
        for event in data.get("events", []):
            if "robot" in event and event["robot"] not in ids:
                raise ValueError(f"Event references missing robot {event['robot']}")
        json.dumps(data, allow_nan=False)

    def add_box(self, a, b, kind="wall"):
        if min(abs(a[0]-b[0]), abs(a[1]-b[1])) < .05:
            raise ValueError("Drag a box at least 0.05 m in each dimension")
        item = dict(pose=[(a[0]+b[0])/2, (a[1]+b[1])/2, 0.],
                    size=[abs(b[0]-a[0]), abs(b[1]-a[1])], editor=dict(kind=kind))
        self.data["obstacles"].append(item)
        return len(self.data["obstacles"])-1

    def add_robot(self, position):
        ids = {r["id"] for r in self.data["robots"]}
        index = 1
        while f"R{index}" in ids:
            index += 1
        x, y = position
        robot = dict(id=f"R{index}", start=[x, y, 0.], goal=[x+3., y, 0.],
                     path=[[x, y], [x+3., y]], footprint=dict(length=.8, width=.6),
                     limits=deepcopy(DEFAULT_LIMITS), velocity=[0., 0.], cooperative=True)
        self.data["robots"].append(robot)
        return len(self.data["robots"])-1

    def replace_path(self, index, points):
        clean = []
        for point in points:
            if not clean or math.dist(clean[-1], point) > 1e-8:
                clean.append(list(point))
        if len(clean) < 2:
            raise ValueError("Click at least one destination after the robot start")
        robot = self.data["robots"][index]
        robot["path"] = clean
        robot["start"][:2] = clean[0]
        robot["goal"][:2] = clean[-1]
        align_headings(robot)

    def move_waypoint(self, index, waypoint, point):
        robot = self.data["robots"][index]
        robot["path"][waypoint] = list(point)
        if waypoint == 0:
            robot["start"][:2] = point
        if waypoint == len(robot["path"])-1:
            robot["goal"][:2] = point
        align_headings(robot)

    def rename_robot(self, index, name):
        if not name.strip() or any(i != index and r["id"] == name for i, r in enumerate(self.data["robots"])):
            raise ValueError("Robot ID must be unique and nonempty")
        robot = self.data["robots"][index]
        old = robot["id"]
        robot["id"] = name
        for event in self.data.get("events", []):
            if event.get("robot") == old:
                event["robot"] = name

    def delete(self, kind, index, waypoint=None):
        if kind == "robot" and waypoint is not None:
            robot = self.data["robots"][index]
            if waypoint in (0, len(robot["path"])-1):
                raise ValueError("Endpoints define start/goal; move them or delete the robot")
            del robot["path"][waypoint]
            align_headings(robot)
        elif kind == "robot":
            name = self.data["robots"].pop(index)["id"]
            self.data["events"] = [e for e in self.data.get("events", []) if e.get("robot") != name]
        else:
            del self.data["obstacles"][index]

    def preset(self, name):
        if name == "blank":
            data = blank_scenario()
        else:
            filename = {"wide": "head_on_wide", "narrow": "head_on_narrow", "crossing": "crossing",
                        "l_shaped": "l_shaped_head_on"}[name]
            data = json.loads((ROOT/"scenarios"/f"{filename}.json").read_text())
            if name == "crossing":
                # Actual cross-shaped corridor made from boxes, with open central square.
                half, extent, thickness = 2.5, 10., .25
                data["obstacles"] = []
                def box(x, y, length, width):
                    data["obstacles"].append(dict(pose=[x, y, 0.], size=[length, width], editor=dict(kind="wall")))
                for sign in (-1, 1):
                    for side in (-1, 1):
                        center = sign*(extent+half)/2
                        box(center, side*(half+thickness/2), extent-half, thickness)
                        box(side*(half+thickness/2), center, thickness, extent-half)
                    box(sign*(extent+thickness/2), 0, thickness, 2*half+2*thickness)
                    box(0, sign*(extent+thickness/2), 2*half+2*thickness, thickness)
                data["name"] = "Cross-shaped wide tunnels"
        self.replace(data)
