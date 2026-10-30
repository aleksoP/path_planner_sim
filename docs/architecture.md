# Architecture and ownership

The dependency chain is `planning_core <- simulator <- avoidance_sim`.
`visualization/replay.py` consumes recording files; it has no influence on the
simulation. CMake's `BUILD_SIMULATOR=OFF` builds/installs the core without JSON,
Python, ROS, Boost, or any other external dependency. `examples/core_client`
demonstrates an independently built consumer of that installation.

## Core modules

| Module | Responsibility |
| --- | --- |
| `types.hpp` | World-frame SI value types, limits, observations, results, configurable costs |
| `geometry` | Angle normalization, projections onto segments, convex SAT, exact polygon distances |
| `reference_path` | Corner smoothing, arc lengths, closest projection, tangent/right normal |
| `motion_model` | Replaceable integration interface and bounded unicycle implementation |
| `trajectory` | Time interpolation and dynamic/nonholonomic consistency checks |
| `prediction` | Fresh communicated intentions or constant world-velocity fallback |
| `collision_checker` | Pure, time-indexed footprint clearance evaluation |
| `path_deformation` | Quintic spatial envelopes and feasible actuator rollouts |
| `planner_interface` | `PlannerResult plan(const PlannerRequest&)` boundary |
| `cooperative_planner` | Candidate generation, hard feasibility, cost, explicit encounter memory |

Each planner instance owns its progress, committed spatial envelope, selected
side, encounter phase, and blocked-encounter ID. There is no mutable global state.
Planner requests own observation/static-geometry snapshots and borrow an immutable
`ReferencePath` for the duration of the call. Call `reset()` for a new mission.
Planning outputs are independent values. A planner instance is not thread-safe;
independent instances may be used concurrently against immutable snapshots.

## Simulator boundaries

`Scenario` owns validated JSON-derived world/robot parameters. `observations`
implements ideal sensor range, field-of-view and static-wall occlusion. `Engine` owns robot
states, planner instances, schedules, metrics and recording buffers. It accepts an
injected `MotionModel` implementation to make integration replaceable.
`TrajectoryFollower` accepts just current state, selected timed trajectory and
elapsed time. Command generation and actuator integration are distinct operations.

Planning at a timestamp reads one frozen snapshot of all states **and all previous
intentions**. Newly generated intentions are published together only after all
agents plan. Even if robot iteration order changes, no later robot sees an earlier
robot's new plan. The integration test exercises reversed iteration order.

Synthetic measurements are explicit (`observations.ground_truth=true`) and
filtered by range, field of view and line of sight through physical wall polygons.
The engine buffers the last visible measurement for a configurable memory
interval; stale tracks keep their original timestamp. Never-observed hidden
robots and their intentions are not supplied. The core sees only supplied observations, never an engine pointer
or world singleton. Non-cooperative agents never communicate intentions. Dropped
intentions preserve position/velocity observations and invoke the fallback
predictor. The current event is idealized loss, not simulated networking.

## Extending the system

Implement another `PlannerInterface` for ORCA, MPC or MPPI baselines. Reuse the
motion/geometry/trajectory modules, or replace candidate generation and scoring
inside a separate planner. Add a prediction strategy boundary when introducing
probabilistic predictions. Noise and observation/communication buffering belong
between the engine or sensors and `PlannerRequest`, not inside visualization.
MuJoCo can replace integration without becoming a planning dependency. A future
ROS adapter converts TF, messages, tracks and costmaps into this same core API.

## Scope and resource use

The engine uses fixed deterministic integer tick scheduling. No simulation
decision uses wall time; `steady_clock` only measures planner cost or optionally
paces execution. Logs are in-memory buffers flushed after each run; use
`--no-record` for larger benchmarks. Streaming logs and spatial broad-phase
indexing are future improvements. JSON is not an IPC or real-time control format.
