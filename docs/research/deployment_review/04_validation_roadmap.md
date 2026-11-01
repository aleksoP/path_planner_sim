# Implementation and deployment validation roadmap

This is proposed work, ordered by dependencies and failure risk. It is not a claim that the planner is deployable today. Keep the current simulator footprint **0.8 × 0.6 m**, forward speed limit **1.5 m/s**, and other current configuration values fixed for the initial comparisons, as requested. Sweep environment and fault variables separately. Make hardware parameters tunable later, with explicit configuration versions and regression coverage.

## 1. Establish an honest experiment contract

Preserve the existing planner and deterministic cases as baseline A. Record the checkout, compiler/build flags, scenario, seeds, planner configuration, observation channel, actor policy, compute budget, and outcome definition with every run. Save failing cases and replay inputs, not just screenshots. A collision or solver failure must not silently shorten the scored exposure.

Keep three experiment classes separate:

| Class | Planner information | Purpose |
| --- | --- | --- |
| Oracle | Exact world state and deliberately supplied future trajectories | Isolate geometry, timing, candidate coverage, and execution defects |
| Perception-only | Delayed/noisy detections and tracks, visible-space evidence, static map | Main target: unknown noncommunicating traffic |
| Cooperative | Explicit peer messages with timestamps, delays, losses, and stated commitments | Optional later fleet extension |

Oracle state belongs to the evaluator. A supposedly perception-only actor must not receive ground-truth identity, another robot's goal, cooperative flag, hidden position, or exact future trajectory. For an isolated prediction/planner decision, replay the same observation prefix. For a closed-loop controller comparison, use matched scenarios, sensing models, fault-generation rules, and actor policies; recompute observations from each robot's actual pose. Once ego motions diverge, their visible-space histories can legitimately differ. When information privileges differ, label and explain them.

Define an episode as safe completion only when all required goals are reached within the time limit **and** there is no collision, prohibited clearance violation, intervention, actuator-limit violation, or invalid plan execution under the chosen contract. Retain separate counters for each component. Also report safe timeouts, blocked missions, protective stops, no accepted candidate, deadline misses, and fallback acceptance failures. A protective stop is not automatically a navigation failure or automatically a successful mission.

## 2. Make actors and observations independently configurable

First implementation slice: add an actor/planner factory, a replayable observation interface, and an independent evaluator. Keep the current planner behavior intact as one actor policy. Add a scripted actor that never yields and one that brakes/turns with a different response delay. Then introduce other reactive policies, track-ID switches, and visibility faults.

This slice should answer whether existing successes depend on every robot sharing the same side preference, ID ordering, and planning algorithm. It is the highest-value prerequisite for a predictor comparison.

Separate scored ego missions from background actors. A scripted actor that deliberately continues or parks need not complete an ego-style mission; its missing goal must not make every test a timeout. Continue scoring all physical contacts and relevant clearances. Perception-only requests should contain local association IDs, not a globally agreed priority, and no peer intentions. Record the exact planner input so this boundary can be inspected.

Observation replay serves deterministic estimator/predictor tests, a single decision from a common history, and latency/fault diagnosis. It cannot by itself establish closed-loop navigation performance: changing ego's route changes visibility, and a reactive actor may change its motion. Closed-loop trials must rerun sensing and actor responses. Couple random faults through a documented scheme where possible; do not assume equal seeds guarantee identical faults after event schedules diverge.

Use hand-constructed, falsifiable cases before random sweeps:

- Head-on passage at widths below, near, and above the physical passing threshold.
- Perpendicular crossing with early/late arrival, opponent braking, and unexpected continuation.
- Turning robot whose center follows an arc, including a zero-turn-rate limit.
- Slow following, stopped obstacle, overtaking with oncoming traffic, and an occupied destination.
- Blind corner, robot emerging from behind another robot, and track disappearance/reappearance.
- A one-robot-wide passage with a retreat bay; the same passage without a feasible escape.
- Asymmetric field of view, sensor latency, localization bias, and temporary missed detections.
- In-place rotation near a wall, a necessary reversal, and a global route update during an encounter.
- A blocked side commitment, repeated symmetric reactions, and a third robot obstructing the first maneuver.
- Planner timeout, stale command, tracking deviation, actuator saturation, and a controller process failure.

Do not require completion in a geometrically impossible or permanently blocked scene. Require a correctly diagnosed outcome and the defined protective response. Liveness claims must say which traffic and infrastructure assumptions exclude permanent obstruction.

## 3. Correct prediction and model contracts

Implement CV and CTRV as distinct models and add acceleration/stopping branches. Validate predictions from a common epoch and compare them with measured history. Reject or downgrade a communicated intention that disagrees with current observation beyond its declared tolerance. A well-formed message can still describe the wrong physical state.

Track position, velocity, uncertainty, existence, and freshness. Track yaw only when observable. Propagate lost tracks and occluded occupancy under explicit motion bounds; use reliable negative observations to clear occupancy. Test both safety failures from deleting a track too early and navigation freezes from retaining impossible occupancy indefinitely.

Gate: coherent model rollouts, no hidden truth leakage, correct handling of stale/invalid inputs, and reported occupancy coverage on held-out traffic. A predictor that improves ADE but misses a rare crossing mode must not pass solely on mean error. Evaluate at 0.5, 1, 2, and 3 s forecast horizons where the data support them; these are evaluation horizons, not changes to the robot speed limit.

## 4. Add acceptance and continuation before more elaborate optimization

Provide one executable-trajectory acceptance interface shared by the baseline and new generator. Check initial-state agreement, timing, drive-model constraints, footprint sweep, static/dynamic occupancy, latency, and a defined continuation. The collision oracle used in offline scoring should use a different calculation path from the planner's sampled checker.

For a first bounded implementation, validate selected switching points along the executable prefix with braking continuations. State precisely how occupancy after the stop is handled. Record why a fallback failed rather than relabeling it feasible. Retain and revalidate the previous accepted plan; accept a new one only before its deadline and expiry.

Gate: no commands drawn from rejected or expired plans in fault-injection tests; known geometry counterexamples are caught; a late solver result cannot replace a still-valid continuation; failures are replayable. This establishes software behavior under a stated model. It does not establish real-robot braking performance.

## 5. Improve trajectory choice in bounded steps

Establish three controller/planner experiments before committing to a full replacement:

| Experiment | Implementation and purpose |
| --- | --- |
| Tuned Nav2 MPPI | Real package behind a small ROS/Nav2 adapter; benchmark the conventional controller under the declared observation interface |
| MPPI with dynamic occupancy | Same package/configuration plus an explicit time-indexed other-agent occupancy interface; measure the value of that added information |
| Bounded Frenet/SE(2) prototype and timed follower | Test whether explicit follow/stop/arrival modes and maneuver families provide sufficient benefit to justify a separate planner |

Use a common static route/reference for local comparisons and a declared acceptance/monitoring policy. Log proposed motion, accepted/executed motion, and protective interventions separately. A new critic with a high penalty is not the independent acceptance gate. Stock MPPI and its occupancy extension intentionally use different future-occupancy information; their comparison isolates that extension. Compare the occupancy extension with the candidate prototype using equivalent track/prediction inputs and the same uncertainty assumptions.

The minimal ROS/Nav2 adapter is a prerequisite for testing the actual MPPI implementation, even though full robot integration comes later in section 8. Keep it outside the middleware-free planning core. A custom sampling controller implemented here would need its own name and validation; it is not evidence about Nav2 MPPI merely because it uses similar mathematics.

For the candidate prototype, start with bounded Frenet lateral candidates and explicit velocity-keeping, following, stopping, and arrival-after-conflict modes. Match current boundary state and re-use revalidated continuations. Validate the right-normal transformations in [the analytical checks](05_analytical_checks.md). Add a bounded set of SE(2) primitives where needed for in-place turns, reverse recovery, and unreliable Frenet charts. Expand the generator only when held-out failures identify missing feasible maneuvers.

Compare additions through ablations:

| Comparison | Question it resolves |
| --- | --- |
| Existing planner vs corrected prediction only | How much failure comes from forecasts rather than candidate geometry? |
| Lateral offsets only vs longitudinal modes | Does waiting/following avoid unnecessary detours and oscillation? |
| No encounter memory vs memory with checked release | Does persistence help without trapping recovery? |
| Current acceptance vs continuation-aware acceptance | What speed/progress cost buys modeled fallback availability? |
| CV vs CTRV/map branches vs interaction-conditioned forecasts | Does added prediction complexity improve closed-loop outcomes? |
| Candidate-only vs seeded numerical refinement | Are missed feasible motions actually caused by candidate resolution? |
| Tuned Nav2 MPPI baseline vs explicit dynamic occupancy extension | Does time-indexed other-agent modeling improve navigation under equal sensing? |

Do not add SFM, diffusion, graph learning, a TEB refiner, and a fleet scheduler simultaneously. That would make causal attribution and debugging difficult. A refiner must pass the same acceptance gate as a sampled candidate.

## 6. Measure real-time behavior without stopping simulated time

The current scheduling periods are research defaults, not measured deadlines. An initial engineering target is to preserve the 100 ms planning period and 20 ms control period with bounded workloads; revise those targets after measuring target hardware. Reserve time for perception, validation, and publication rather than allocating the whole period to optimization.

Record end-to-end age of observations and commands, queue delay, planning/validation time, p50/p95/p99/max, deadline misses, CPU/memory usage, actor count, and candidate count. Include cold starts, saturated logging, visualization load, and expensive narrow-passage cases. A mean runtime below a period is insufficient. Set explicit caps and define what happens at each cap.

Run two modes: deterministic simulation time for algorithm analysis and wall-clock-coupled execution for deadline analysis. If the simulator slows down, disclose the real-time factor. Never let all other robots pause while our planner computes and then call the result real time.

Gate: bounded work, no unchecked timeout behavior, and an execution monitor demonstrated under injected missed deadlines. The acceptable miss rate and maximum latency need an application-level requirement; do not invent a deployment certificate from a percentile.

## 7. Statistical comparison and failure accounting

Use paired maps, seeds, and actor trajectories where possible. Separate tuning, regression, and held-out evaluation sets. Include policies and map layouts absent during tuning. Report distributions and confidence intervals for completion, collisions, interventions, clearance, trip time, delay, throughput, oscillation, and compute. Publish both all-episode results and successful-only travel times so selective failure cannot make a planner look fast.

For learning-based predictors, split by scene/session or robot policy where appropriate; randomly splitting overlapping trajectory windows can leak near-duplicate futures. Fix sample count when comparing best-of-K metrics. Include calibration, mode miss rate, and latency. Learned joint forecasts should be tested for scene consistency, not only independent per-agent error.

Use the [trial-count example](05_analytical_checks.md#risk-and-trial-counts) to interpret zero observed events. Do not use repeated deterministic runs as independent safety evidence. Keep near-miss logs: decreasing clearance, emergency braking, and repeated protective stops can reveal a problem before collision counts do.

There is no universal numerical “pass” threshold derived from these papers. Set required exposure, intervention rate, clearance, and mission throughput from the intended operating domain, then freeze those requirements before the held-out comparison. Report the observed envelope of failure rather than a single success percentage.

## 8. ROS integration and controlled physical testing

Choose one owner for maneuver geometry and timing. A timed trajectory tracker is the most direct experiment. An MPPI extension is viable if it explicitly consumes dynamic occupancy and checks timing/retiming validity. Merely publishing the new planner's geometry as an ordinary path leaves the timing contract unresolved [R69].

Build on the minimal MPPI benchmark adapter from section 5 and use the production track interface in a ROS simulation before hardware. Pin ROS distribution, dependencies, controller versions, and configurations. Validate frame transforms, time synchronization, stale data, lifecycle transitions, cancellation, command arbitration, and restart behavior. Treat GPU/model loading as part of startup and failure handling when relevant. No external controller has been built or benchmarked in this review.

Physical testing proceeds from isolated motion characterization to controlled encounters, then representative workspace trials. Measure loaded/unloaded stop distance and delay, track timing/pose error, sensor coverage, and turning behavior. Begin encounters at reduced speeds and increase within measured acceptance bounds; retain the fixed 1.5 m/s as a maximum target rather than a required speed in every scene. Use independently available protective functions and controlled access during these tests.

Gate: the measured motion/latency bounds fit the assumptions used by the acceptance checker; faults trigger the documented behavior; corner and track-loss cases are reproduced with actual sensing. Robot deployment also requires the applicable organizational and machinery safety process. This report does not select a legal standard, certify conformity, or substitute paper-level theorems for that process.

## 9. Optional coordinated fleet layer

Only after the perception-only baseline is characterized, add a communication experiment with explicit peer membership, plan epochs, commitments, passage ownership, expiry, and release. Model complete loss as well as delay. Test deviations by a nominally cooperative robot. A released reservation is not evidence that the physical region is empty.

Start with a small critical-region protocol and a few known agents. SIPP-IP/CTC, delay-robust CCBS, RHCR/LaCAM, and RMADER then address different parts of the problem. They should be compared under the infrastructure assumptions they require. Unknown robots still enter through observation and occupancy, not the shared scheduler's authority.

## The next concrete implementation task

Create an actor factory and an observation-replay benchmark with one nonyielding actor, one turning actor, and independent collision scoring. Use it to expose truth leakage and compare current predictions with CV/CTRV. In the following slice, add explicit plan expiry and continuation acceptance. These slices make later claims about interaction-aware or optimized planning testable while preserving the current working baseline.
