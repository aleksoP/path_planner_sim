# Recording and metric definitions

All quantities use SI units. JSONL state/decision logs are deterministic for the
same scenario and build, excluding no wall time because wall time is stored only
in the metrics summary. `run_config.json` expands planner and physical defaults,
observation settings, effective smoothed paths, seed and scheduling parameters.
It can be supplied as `--scenario` to reproduce a run. Unknown extra metadata
fields in a recording are harmless to the scenario loader.

## Safety

`collisions` counts contact episode onsets for each robot/robot or robot/static
pair, not every integration sample. `collision_events` includes the pair, sampled
timestamp and approximate location (robot center, or pair midpoint; not a solved
contact point). `clearance_violations` counts episode onsets below configured
clearance. `near_misses` counts episode onsets between required clearance and the
additional tracking margin. `minimum_geometric_separation` is the smallest
sampled exact polygon distance over all physical pairs. Overlap has separation
zero; it is not a signed penetration depth. With no physical pair it serializes
as JSON null.

`predicted_conflict_updates` counts robot planning updates where the nominal
trajectory fails its predicted clearance, not unique encounters.
`no_feasible_updates` counts results where even the candidate set's braking option
failed current prediction feasibility. These are not actual collision counts.

## Navigation and dynamics

Per-robot `goal_time` is the first integration timestamp satisfying position,
heading and speed goal tolerances (null if incomplete). `distance` sums integrated
center travel. Path deviation is distance from the smoothed reference, with
time-average and maximum recorded. `average_speed` is absolute speed averaged
over the complete run duration, including waiting after an earlier robot's goal.
`low_speed_time` uses 0.1 m/s. A `full_stops` event means speed dropped below
0.03 m/s after previously exceeding 0.15 m/s, excluding initial rest.

`avoidance_maneuvers` counts new encounters; `maneuver_replans` counts revisions
of an active same-side envelope (including a revised return); `passing_side_reversals` counts
changes between nonempty sides, even across encounters. `stalled` means no 0.05 m
progress increment in the last 5 s without goal completion. `timeout_events` and
the explicit `unresolved_timeout` outcome prevent indefinite stops counting as
successful navigation.

Dynamics metrics use actual integrated velocities: maximum yaw speed, positive
linear acceleration, linear deceleration and absolute angular acceleration.
Curvature is `abs(omega/v)` above 0.05 m/s; near-zero in-place rotations are
excluded. `command_saturation_substeps` counts model substeps where requested
velocity differed from applied velocity, including normal acceleration slew
limiting. It is not a count of dangerous limit violations.

## Planning performance

Planner calls, candidate counts, rejected-candidate reasons and configured
frequency are recorded. Effective calls per robot per simulation second also
include timestamp-zero and terminal calls and decrease when agents finish early.
`average_wall_seconds`, `maximum_wall_seconds`, and `total_wall_seconds` use
`steady_clock` around `plan()` only. `execution_wall_seconds` includes simulation,
metrics and in-memory recording, but excludes final file writing.

Do not infer real-time capability from 10 Hz simulated scheduling. The scaling
script measures actual wall time for parallel, nonintersecting lanes; it is not a
dense interacting-crowd benchmark. All-to-all prediction/geometry cost remains a
limitation. Record hardware/build/workload when comparing results.

## File formats

`trace.jsonl`: time, collision count, minimum separation so far; per robot: ID,
pose `[x,y,yaw]`, actual `[v,omega]`, target command, phase/status/side, distance,
goal flag. Sampling defaults to 10 Hz. Collision detection runs more frequently.

`events.jsonl`: plan records with relative timed points
`[t,x,y,yaw,v,omega]`, candidate scores and rejection reasons, prediction sources,
state transitions, scenario events, goal events, collision/clearance/near-miss
events, and final outcome. `--debug` adds rejected-candidate geometry; selected
trajectories/predictions are always included in recorded mode.

Plan records also include measured `robot_pose`, filtered `observations` (source
timestamp, visibility, pose and velocity), `maneuver_replanned`, and `rejoin_s`.
`maneuver_path` is the selected spatial intent through the merge and a short
nominal section. Its endpoint lies on the reference route. In replay it is
dash-dot with a hollow endpoint; the thicker solid trajectory is the checked
finite **timed** horizon. Spatial intent beyond that horizon is revalidated on
subsequent replans and is not a promise of future obstacle timing. Observation
IDs marked `*` in replay are stale remembered tracks.

`--no-record`: configuration/metrics remain; trace and event files are empty. This
avoids large buffers and expensive JSON creation in algorithm benchmarks.
