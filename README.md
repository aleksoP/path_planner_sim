# Cooperative AMR Path Planning Simulator — V0.1

A standalone C++17 research baseline for differential-drive cooperative path
deformation. The reusable `planning_core` has no simulator, JSON, ROS, Python,
Nav2, Gazebo, or MuJoCo dependencies. This directory is isolated from the production
navigation packages elsewhere in the workspace.

The [deployment research review](docs/research/deployment_review/README.md)
assesses the reference collection, the supplied GPT discussion, and the current
code, with a perception-only architecture and staged validation roadmap.

## Build and verify

On Ubuntu 24.04 install the build dependencies (if absent):

```bash
sudo apt install build-essential cmake nlohmann-json3-dev libboost-dev
cd tools/path_planner_sim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
```

Boost.Test is header-only and used only by tests. `nlohmann_json` is used only by
the simulator/runner. No network downloads occur during CMake configuration.

```bash
build/bin/avoidance_sim --scenario scenarios/head_on_wide.json \
  --seed 42 --output runs/my_wide --headless
python3 visualization/replay.py --run runs/my_wide
```

For Python playback use a virtual environment with
`pip install -r visualization/requirements.txt`, or Ubuntu's `python3-matplotlib`.
This workspace's user-installed NumPy conflicts with the system Matplotlib;
the verified system-package invocation is:

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/path-planner-mpl \
  python3 visualization/replay.py --run runs/my_wide
```

Space toggles playback; arrows step; Home seeks to the start; `r` resets the view.
The timeline, speed slider, overlay checkboxes, and Matplotlib pan/zoom toolbar
support inspection. `--debug` on the simulator also records rejected candidate
geometry. The viewer contains no planning logic and never controls execution.

```bash
python3 visualization/replay.py --run runs/my_wide --validate
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/path-planner-mpl \
  python3 visualization/replay.py --run runs/my_wide \
  --snapshot runs/my_wide/passing.png --time 6
python3 scripts/batch.py --output runs/my_batch --experimental --record
python3 scripts/benchmark.py --output runs/my_scaling --counts 1,2,5,10,20
```

Output directories must be fresh: existing recordings are never overwritten.
Headless is the default. CLI options include `--duration`, `--config`, `--seed`,
`--no-record`, `--debug`, and optional wall-clock pacing with `--realtime`.
`--config config/default_parameters.json` applies a JSON merge-patch of planner
parameters over the scenario. Batch mode is the separate `scripts/batch.py` tool.
Exit codes: **0 completed, 2 unresolved timeout, 3 collision, 1 input/I/O error**.
A nonzero outcome must not be discarded as a successful navigation run.

## Components

```text
planning_core/include/cooperative_avoidance/   Public middleware-free API
planning_core/src/                           Geometry, paths, dynamics, prediction, planning
simulator/include/avoidance_sim/              Scenario, controller, engine API
simulator/src/                               JSON loading, independent follower, deterministic engine
simulator/main.cpp                           CLI and recording output
config/default_parameters.json               Full planner defaults and scoring weights
scenarios/                                   Seven primary and six experimental JSON scenarios
visualization/replay.py                       Recorded Matplotlib playback and PNG export
visualization/editor.py                       Box-map / multi-robot desktop authoring
visualization/scenario_document.py            Authoring model, history and JSON persistence
visualization/editor_runner.py                Asynchronous C++ run snapshots
tests/unit/                                  Ten Boost.Test cases
tests/integration/                           Eight C++ scenario suites and thirteen editor tests
examples/core_client/                        Independent consumer of the installed core
scripts/                                     Batch and scaling runners
docs/                                        Architecture, algorithm, model, metrics, integration, results
```

## Map and scenario editor

Launch the Python desktop editor to build maps and robot missions:

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/path-planner-mpl \
  python3 visualization/editor.py
```

It uses the same Matplotlib dependencies as replay and needs a graphical desktop
with an interactive backend (Ubuntu's `python3-matplotlib` and `python3-tk` work).
In a clean virtual environment, the `PYTHONNOUSERSITE` workaround is unnecessary.

* **Wall box / Obstacle box:** drag opposite corners; both are physical geometry.
* **Place robot:** click a start position, then select **Draw path** and click
  waypoints. Finish with Enter or right-click; the last point becomes its goal.
* **Select / move:** drag robot starts, goal stars, path handles, or box bodies.
  Box corner handles resize; the circular handle rotates. Double-click a path
  segment to insert a waypoint. Properties provide exact dimensions, headings,
  speeds, acceleration limits and cooperative flags.
* **Blank / Wide tunnel / Narrow tunnel / Crossing / L tunnel:** start a new environment
  from a template; Undo restores the previous document.
* **Save JSON / Open:** use the editable JSON file path. **Run planner** tests an
  immutable snapshot with the C++ executable; **Replay run** opens its recording.
  Editing may continue while the run executes. Unresolved/collision results are
  displayed explicitly. Map drafts can be saved before robots are added.

```bash
python3 visualization/editor.py --scenario scenarios/head_on_wide.json \
  --output scenarios/my_environment.json
python3 visualization/editor.py --preset crossing
python3 visualization/editor.py --preset l_shaped
```

**L tunnel** includes a 3 m-wide corridor and two opposing robots. A travels from
`(-8, 0)` through the elbow at `(0, 0)` to `(0, 8)`; B follows the same route in
reverse, starting at `(0, 8)` facing down and finishing at `(-8, 0)` facing left.
Robots remain on their routes while the other robot is hidden by walls. They
replan after acquiring a relevant visible obstacle, adapt the passing envelope,
and merge back only after the opposing footprint has cleared. Both reach their
goals in 16.5 s with zero collisions/clearance violations in the supplied case.
It is recorded in `scenarios/l_shaped_head_on.json`. The original
completed single-robot turn remains in `scenarios/l_shaped_corridor.json`.
You can draw another L-shaped route with **Draw path** by clicking an elbow and
then a goal; the preset also includes approach/exit waypoints to control corner
smoothing in the tighter corridor.

The default save path for an opened scenario ends in `_edited.json`, preserving
the source by default. Explicit Save replaces the file named in the path field.
The editor preserves planner settings, unknown metadata, and timed scenario
events; renaming a robot updates event references. There is no planning algorithm
in the editor. See the [editor guide](docs/editor.md) for controls and limitations.

## Core API and algorithm

```cpp
namespace ca = cooperative_avoidance;
ca::CooperativePlanner planner(config); // one persistent instance per robot
ca::PlannerRequest request{timestamp, robot, reference_path, goal,
                          static_polygons, observed_agents};
ca::PlannerResult result = planner.plan(request);
```

Requests carry explicit SI-valued state, path, static convex polygons, tracked
footprints, uncertainty, and optional communicated intentions. The path reference
must remain alive during `plan()`. Results include a timed trajectory, status,
passing side, maneuver phase, minimum predicted separation, candidate rejection
reasons, and prediction diagnostics. Call `reset()` when replacing a mission/path.
Invalid inputs throw `std::invalid_argument`; lack of a safe maneuver is a result
status, with bounded braking explicitly identified when even braking fails the
current prediction model.

The planner evaluates nominal tracking, right/left quintic offsets at configurable
magnitudes, slower profiles, and bounded braking. Candidates roll out a unicycle
with acceleration-limited actuators. Full static bypass geometry and time-indexed
oriented footprints are hard constraints; progress, smoothness, deviation, speed,
clearance, and side consistency determine the cost among feasible candidates.
The passing side remains committed; entry/hold/return geometry can be replanned
from the current lateral position and heading. Compatible right-hand behavior
and stable-ID crossing priority are deterministic policies, without an acknowledged
negotiation protocol. See [algorithm details](docs/algorithms.md).

## Recordings and results

Robot observations default to range 30 m, 360° field of view, wall line-of-sight
occlusion, and 0.5 s stale-track memory. Configure these with
`observations.range`, `field_of_view_degrees`, `line_of_sight`, and `memory` in
scenario JSON. Hidden never-observed robots and their intentions are not supplied
to the planner. Static map geometry remains known. Measurements are ideal
synthetic poses, not simulated lidar detections.

Replay shows the checked timed trajectory as a thick solid line and the longer
spatial merge intent as dash-dot with a hollow endpoint on the original route.
The latter is revalidated during execution. The `observed` display makes sensor
acquisition visible; `*` marks stale remembered tracks. Run again after changing
planning/sensor settings; opening an old recording does not recompute decisions.

Each run writes `run_config.json` (effective parameters and physical paths),
`trace.jsonl` (deterministic sampled states), `events.jsonl` (plans, diagnostics,
predictions, goals, collisions, scenario events), and `metrics.json` (navigation,
safety, dynamics, and separate wall-clock planner performance). `--no-record`
retains configuration and metrics with empty trace/event streams. Seeds are
recorded; the baseline uses no random sampling. Wall-clock fields vary; simulation
states and decision logs are reproducible on the same build.

The validated wide corridor starts **both robots at 1.5 m/s**. They pass to their
own right, return to the original centerline, and both finish in about **13.2 s**,
with **zero collisions, zero clearance violations, and no intermediate full stops**.
The too-narrow corridor is intentionally **unresolved**, with both robots stopped.
See [validation results and exact unresolved conditions](docs/validation.md).

## Portability and next work

To build only the reusable library, without the scenario parser or Boost:

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF -DBUILD_TESTING=OFF \
  -DCMAKE_INSTALL_PREFIX=/tmp/avoidance-core
cmake --build build-core -j 4
cmake --install build-core
cmake -S examples/core_client -B build-client \
  -DCMAKE_PREFIX_PATH=/tmp/avoidance-core
cmake --build build-client
build-client/core_client
```

That client links only `cooperative_avoidance::planning_core`. A future ROS adapter
can use exactly that target. The [ROS 2/Nav2 design](docs/ros2_integration.md)
compares a timed-trajectory adapter, a Jazzy `nav2_core::Controller` plugin, and
MPPI critic reuse; no untested ROS wrapper is included.

V0.1 uses kinematics, synthetic observations, and deterministic policies. Payload
does not change dynamics: model the desired loaded acceleration/braking limits in
configuration. This is not a validated 150 kg rigid-body or braking model.
Arbitrary concave polygons, global replanning, reversing to passing bays,
acknowledged negotiation, sensor noise/latency, transport simulation, and production
Nav2/MuJoCo adapters remain future work. Sudden obstacles can invalidate a committed
envelope and leave the robot safely stopped. Finite-horizon prediction cannot
guarantee safety under arbitrary unmodeled behavior or tracking error.

Recommended next steps: broaden recovery for terminal/blocked maneuvers; add passing-bay
and backtracking behavior; improve curvature-aware speed scheduling and uncertainty;
profile collision checking for dense crowds; then test both Nav2 integration
architectures against the same recorded encounters.
