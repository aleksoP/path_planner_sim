# Implemented cooperative path deformation

## Coordinates and reference paths

World coordinates use metres, yaw counterclockwise, and `wrap_angle` in `[-pi,pi)`.
Path progress `s` is arc length along the direction of travel. Duplicate vertices
are removed. Three endpoint-preserving Chaikin passes round polyline corners by
default; the pass count is configurable. Position is piecewise linear in arc
length. Tangent is estimated over a 0.30 m arc window. The **right normal** is
`(tangent.y, -tangent.x)`; positive offset is always the robot's own right. Thus
opposite headings produce opposite world-space shifts.

Chaikin can cut inside corners; only collision-checked trajectories may be used.
This representation approximates a smooth path; it is not an arbitrary-polyline
Frenet theorem or a C2 spline. Rollout limits and explicit dynamic checks reject
infeasible steering. Progress is monotonic per planner instance, with a small
projection search allowance; self-intersecting global paths need a richer segment
association strategy in a future version.

## Envelopes and rollouts

An offset has `start`, `entry_end`, `hold_end`, and `exit_end` arc coordinates.
The initial entry uses `D*q(u)`, where `q=10u^3-15u^4+6u^5`; hold uses `D`, and exit
uses `D*(1-q(u))`. A revised entry starts at the measured lateral displacement
`d0` and heading-derived slope `m0`:

```text
d(u) = d0 + (D-d0)*q(u) + entry_length*m0*(u-6u^3+8u^4-3u^5)
```

The Hermite basis preserves entry position/slope and ends with zero slope and
second derivative. Actuator rollouts, rather than the polynomial alone, decide
dynamic feasibility. A pure return candidate sets `D=0`, joining the original
route without an instantaneous lateral jump.

The current envelope stays anchored while feasible. If fast moving candidates
become infeasible, the planner evaluates same-side envelopes with revised offset,
transition length and passing hold. Defaults evaluate offsets 0.85/0.95/1.10 m
and transition lengths 3.2/1.6 m, each at multiple speeds. Hold extensions can
postpone a blocked return. Candidate scoring penalizes envelope changes; the
passing side remains committed. Insufficient remaining path length rejects the
envelope. Full static geometry is checked through the rejoin point, even beyond
the finite timed horizon.

For each envelope and configured speed scale, a pure-pursuit-style rollout
converts path-relative targets into forward speed and yaw-rate requests. Curvature
and yaw-rate bounds reduce speed; an acceleration-limited unicycle model produces
the actual candidate trajectory. Goal approach uses braking-distance and distance
speed caps; final heading is approached at near-zero forward speed. The simulator
follows the resulting **timed** poses/twists with a separate feedback controller.
This is a simple feasible rollout search, not an optimal time parameterization.

The dynamic validator checks finite values, ordered timestamps, speed, forward/
reverse bounds, acceleration, deceleration, angular acceleration, curvature above
0.05 m/s, and approximate midpoint unicycle consistency. Near-zero-speed in-place
rotation is supported. The generated candidates use this exact integration model.

## Prediction and hard constraints

Tracked agents carry measurement timestamp, pose, world velocity, footprint,
confidence and position-uncertainty radius. Baseline prediction holds world
translation velocity and yaw rate constant. A cooperative intention is used only
if fresh, internally consistent, and covering at least the next second (or the
horizon if shorter). The small tail beyond its coverage is extrapolated. Old or
malformed intentions fall back to observed velocity. Uncertainty includes the
supplied radius, confidence allowance and configured age plus **future sample
time** growth. A fresh exact measurement does not receive the entire horizon's
uncertainty at time zero.
No model assumes a non-cooperative obstacle will yield.

All candidates are tested against oriented rectangles and static convex polygons
at matching relative timestamps. Spatially bounded substeps and a relative
footprint-point-speed separation allowance conservatively cover interpolation
gaps. Required geometric clearance plus predicted uncertainty is a hard
constraint. An additional tracking buffer ramps from zero at the measured state
to `tracking_margin` over `tracking_recovery_time` (default 0.5 s). This lets a
robot recover from an excursion into the extra buffer while never relaxing
required clearance. Static intent checks likewise restore their additional
tracking/sample buffer over the first lookahead distance. A regression explicitly
rejects an initial state violating the required 0.15 m even during buffer recovery.
Contact/collision, a required-clearance violation and a predicted conflict are
distinct quantities. No weighted cost can rescue a rejected hard constraint.

Only then are candidates scored:

```text
-progress_weight * progress
+deviation_weight * abs(offset)
+speed_reduction_weight * (1-speed_scale)
+smoothness_weight * sum(abs(delta_omega))
+clearance_weight / (0.1+minimum_separation)
+left_side_preference_penalty
+passing_side_switching_penalty
+same_side_envelope_change_penalty
```

All weights, speed scales, offsets, time steps, horizons and margins are configurable.
Order breaks exact ties deterministically. Traversal time is represented indirectly
by finite-horizon progress and speed reduction. No nonlinear solver is used.

## Encounter policy

State transitions are recorded explicitly: `Normal -> ConflictDetected ->
ManeuverSelected -> Shifting -> Passing -> Returning -> Recovery -> Normal`, with
`Yielding` possible whenever slow/stop is selected. A conflict means the **nominal
trajectory** intersects predicted occupancy/clearance, not mere proximity.

A committed side is a hard hysteresis choice until rejoining; candidate scoring
also penalizes changes. A configured minimum commitment time and recovery
cooldown prevent premature release. Every replan reevaluates the remaining
trajectory against current observations. Infeasible passages can change offset
magnitude and timing on the same side, with bounded braking when no moving
solution passes the checks. The unexpected-obstacle test now completes through
this recovery, although arbitrary recovery/global rerouting is not implemented.

Returning is permitted only when the own rear footprint has cleared the
encountered obstacle's front footprint along the route by `return_clearance`
plus uncertainty. This gate is checked at future return samples as well as for
an immediate rejoin. Once passed, a shorter checked Hermite merge can replace
the old return. Lost tracks retain their last measured/extrapolated occupancy;
disappearance is not a clearance acknowledgment. Full time-indexed SAT clearance
checks still apply throughout the maneuver and merge.

The simulator's observation frontend applies range, field of view and wall
occlusion before constructing requests. Hidden never-observed robots, including
their intentions, are absent. An ideal tracked pose is produced when any target
footprint sample becomes visible; recently lost tracks retain their measurement
timestamp for a configurable memory interval. The planning core owns no sensor
or world geometry lookup outside its explicit input.

Right is preferred only when feasible. Both robots independently selecting their
own right produces compatible maneuvers in the primary wide scenario. Fresh
intentions inform collision checks, but there is **no acknowledgment handshake or
formal trajectory negotiation**. At crossings, the larger stable ID yields to
the smaller ID by excluding fast and lateral candidates while a nominal conflict
persists; slow or stop must still pass the safety tests.

When a head-on conflict has no usable static passing envelope, the encounter is
latched as unresolved and both agents hold a checked bounded stop. It is released
when the opposing footprint has cleared behind; it does not periodically creep
toward a stationary opponent. There is no reverse-to-bay recovery. Missing the
latched opponent does not authorize motion blindly. This deliberately conservative
policy can require external recovery and is not a proof of global deadlock freedom.

If no candidate, including braking, is predicted feasible, the result is
`NoFeasibleManeuver` with bounded emergency braking and its rejection reason.
The simulator executes that command normally and still detects any actual
collision. It never freezes bodies or suppresses collision metrics to fabricate
safety. Arbitrarily late obstacles can defeat the available braking distance.
