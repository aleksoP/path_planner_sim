# Assessment of the follow-up GPT review

Date: **1 November 2026**. Input: the newly supplied “Research assessment: Deployable predictive navigation for our AMRs” text. This note evaluates it against the existing review, the current simulator source, and newly checked official sources. The original 70-work coverage count is unchanged. This follow-up does not reproduce a planner or add production behavior.

## Decisions to adopt

The assessment agrees with the evidence-backed architecture: onboard observations, coherent alternative predictions, timed motion selection, explicit acceptance, and monitored execution. Unknown robots remain independent actors. The existing footprint and speed limit remain fixed for this research baseline.

Its most useful refinement is to make the full Frenet planner a conditional choice. The research does not establish that it will beat a tuned MPPI controller with explicit dynamic occupancy. We should build the common experimental boundaries first, then compare:

1. Tuned Nav2 MPPI under the declared sensing interface.
2. The same controller with time-indexed other-agent occupancy.
3. A bounded Frenet/SE(2) prototype with explicit longitudinal modes and a timed follower.

A prototype is still needed to measure the third option. The comparison should decide whether to expand it, rather than committing to a complete replacement architecture up front. A TEB/MPCC refiner remains conditional on demonstrated candidate-coverage failures. MPPI already predicts ego motion and supports differential-drive models and extensible critics; adding useful other-agent occupancy and valid execution semantics is separate engineering work. [Official Jazzy MPPI documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/).

## Details to correct or make explicit

**Observation replay is not a complete closed-loop benchmark.** The simulator's `observable(observer, target, ...)` depends on the observer's pose. If two planners choose different routes, replaying identical sensor observations can give one robot observations it could not obtain. Reactive background actors can also respond differently. Replay identical prefixes for isolated decisions and prediction tests. For navigation trials, use matched scenario/policy/sensing definitions and regenerate observations from actual states. The previous roadmap wording requiring the same observation stream has been corrected.

**The real MPPI experiment needs early integration.** The proposed Stage 3 cannot test the actual Nav2 package while postponing all ROS work until Stage 5. Build a minimal ROS/Nav2 benchmark adapter before that experiment, outside `planning_core`; leave full sensor/hardware/lifecycle integration for later. A standalone implementation inspired by MPPI is a different baseline until equivalence is established.

**Independent actors require mission and information semantics.** Current `Engine::run` instantiates `CooperativePlanner` for each robot and constructs observations from exact visible robot states; it can also attach peer intentions. The existing `cooperative` flag does not make an actor a different controller. Add explicit actor policy selection, disable peer intentions in perception-only requests, and use local association IDs without shared priority semantics. Score ego goals separately from scripted background actors while retaining all collision accounting.

**Acceptance must inspect what gets executed.** Comparing MPPI against a candidate planner requires specifying how each proposal becomes an accepted control prefix, how it expires, and which fallback the monitor executes. Weighted obstacle critics alone do not establish that contract. Report raw proposals, accepted trajectories, executed motion, and intervention counts so a protective layer cannot silently hide planner failures.

**A reported cross-check is not a reproduction.** The follow-up repeats the paper metrics and analytical examples; it does not add experimental evidence that either architecture is superior. ORCA-AVO remains a useful compatible-policy benchmark, but its representation and actuator assumptions must be exposed in a rectangular, acceleration-limited comparison. No reciprocal-policy guarantee becomes valid for unknown traffic merely through adding sensor noise.

## New deployment-reference checks

The ISO catalog confirms that ISO 3691-4:2023 addresses driverless industrial trucks and their systems, including AMRs within its defined scope. As checked on this date, it is listed as published and marked for revision, with ISO/DIS 3691-4 under development. Therefore, track the applicable edition and scope during deployment review rather than freezing a standard citation now. Only the public catalog/abstract was checked; no clause-level conformity assessment was performed. [Official ISO record](https://www.iso.org/standard/83545.html).

The Nav2 claim is supported: its Jazzy Collision Monitor documentation states that it does not provide hard real-time safety certification. It is a useful software response layer, but should not be treated as evidence of a certified protective chain. [Official Jazzy Collision Monitor documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/).

These sources refine the deployment discussion; they do not change any paper's evidence level or certify this system.

## Resulting implementation order

First add heterogeneous actors, a planner-input recording/replay boundary, and independent offline collision scoring. Distinguish isolated replay tests from closed-loop sensing tests. Then correct prediction/model contracts and add plan expiry, continuation acceptance, and fault handling. After those foundations, implement the minimal MPPI adapter and the bounded candidate prototype needed for a measured architecture decision. Full robot integration and physical characterization follow.

The detailed changes are incorporated into [the system design](02_system_design.md) and [the validation roadmap](04_validation_roadmap.md). The follow-up supplies useful prioritization; the first implementation task remains simulator integrity.
