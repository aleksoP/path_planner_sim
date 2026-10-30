# Simulation model and configuration

## Fixed-time execution order

Default integration is 0.02 s, planning 0.1 s, control 0.02 s and recording 0.1 s.
Planning, control and recording periods must be integer multiples of integration
time; unsupported ratios are rejected. Duration is truncated to whole integration
ticks. At tick `k`, simulation time is `k*dt`:

1. Apply due scenario events at the first tick at/after their configured timestamp.
2. On planning ticks, freeze all robot states and previous intentions, construct
   explicit range/FOV/occlusion-filtered observations and recent stale tracks,
   plan all robots, then publish all results.
3. On control ticks, compute commands from the selected timed trajectory and its
   elapsed simulation time. Goal-complete/forced-stop robots receive bounded stop.
4. Record sampled state and decisions on recording ticks.
5. Integrate every robot simultaneously over small common substeps and inspect
   physical geometry after each substep. Accumulate dynamics/navigation metrics.

Stop once all position, heading and velocity goal tolerances pass, or on timeout.
An unresolved timeout is distinct from completed navigation. Wall-time pacing
never changes `dt` or scheduling. The seed is metadata for future stochastic
models; this version has no random sampling.

## Kinematics and control

The model obeys `x_dot=v*cos(yaw)`, `y_dot=v*sin(yaw)`, `yaw_dot=omega`, using
midpoint integration and velocity slew limits. Forward/reverse/yaw speed,
acceleration, deceleration and angular acceleration are configurable per robot.
Opposite-direction commands brake to zero first. A separate timed-trajectory
follower combines feedforward twists with along-track, lateral and heading error
feedback, then the model enforces actuator bounds again.

This model has no tire slip, motor currents, contact force, mass, payload,
suspension, rolling resistance, friction uncertainty or rigid-body dynamics.
To screen a 150 kg design, configure independently established loaded speed and
braking limits; simulated success is not physical validation. `MotionModel` can
later wrap a higher-fidelity backend. Controller gains are currently fixed
validation-tool parameters, while physical and planner parameters are JSON data.

## Geometry and discrete collision monitoring

Footprints are oriented rectangles of configured length/width. Physical walls
are convex polygons, not display lines. Rectangles may be rotated; explicit
convex polygon obstacles are supported. Concave/degenerate input is rejected
rather than passed into a convex SAT algorithm. Environment bounds are represented
by physical wall rectangles in each supplied scenario.

SAT tests actual contact; vertex-to-edge distances provide minimum geometric
separation for disjoint convex polygons. Integration substeps bound a footprint
point's maximum translation plus rotational travel to 0.01 m. This is sampled
collision monitoring, not exact continuous rigid-body contact. Planner checking
adds a conservative velocity-based interpolation allowance to avoid accepting
paths that tunnel between samples. Extremely thin geometry, arbitrary model
replacement, unbounded obstacle acceleration or tracking errors need further
continuous-collision validation before higher-speed deployments.

## JSON scenario contract

See the supplied scenarios for executable complete examples. Fields:

| Field | Meaning |
| --- | --- |
| `name` | Run/scenario label |
| `simulation` | `timestep`, `planner_period`, `control_period`, `record_period`, `duration` in seconds |
| `observations` | `ground_truth` (only true supported), `range`, `line_of_sight`, `field_of_view_degrees`, `memory` |
| `planner` | Overrides of `config/default_parameters.json`; CLI `--config` merges last |
| `obstacles` | `{pose:[x,y,yaw],size:[length,width]}` or `{polygon:[[x,y],...]}` |
| `robots` | Unique ID, start/goal pose, `path`, footprint, initial `[v,omega]`, limits, cooperative flag |
| `events` | Timed `add_obstacle`, `stop_robot`, or `communication_loss` with robot and duration |

Robot paths contain at least two distinct points and must terminate at the goal
position. Goals include heading. `smoothing_passes` defaults to 3. The supplied
scenarios explicitly specify physical dimensions and motion limits. Defaults are
declared in `types.hpp` and expanded into the run configuration.

`stop_robot` commands permanent acceleration-limited braking rather than setting
velocity discontinuously. `communication_loss` suppresses outgoing intentions for
the stated interval, while synthetic position/velocity observations continue.
`add_obstacle` activates real collision geometry and planner observation geometry
at the same event tick. No sensor noise, latency buffer or packet transport model
is claimed in V0.1.

Observation defaults are range 30 m, 360° field of view, static-wall occlusion
enabled, and 0.5 s memory. At least one target footprint sample must be visible.
The resulting measured pose/velocity is ideal synthetic ground truth, not a
simulated lidar detector; partial-visibility estimation errors/noise are absent.
Unknown hidden agents are not sent to the planner or communication predictor.
Set `line_of_sight=false` explicitly to reproduce omniscient range-only robot
observations. Static map geometry remains known input for collision evaluation.
