# Future ROS 2 Jazzy and Nav2 integration

No ROS wrapper has been implemented or compiled in this standalone project.
Production navigation packages are unchanged. The installed target
`cooperative_avoidance::planning_core` already supports an external CMake client
without the simulator. An `ament_cmake` adapter should link that target and depend
on ROS messages, TF, costmaps and Nav2 only in the adapter package.

## Conversion and timing boundaries

| ROS-side data | Core value / boundary |
| --- | --- |
| TF-transformed robot pose | `Pose2D`, planar yaw in one documented local/world frame |
| Odometry / current twist | `RobotState.velocity`, actual bounded body-forward speed and yaw rate |
| Global `nav_msgs/Path` | Smoothed `ReferencePath`, per-mission progress reset |
| Robot parameters | `RobotFootprint`, `MotionLimits`, `PlannerConfig` |
| Costmap and walls | Convex static polygons; conservative costmap extraction in adapter |
| Tracked moving objects | `ObstacleState` IDs, measurement time, velocity, footprint, uncertainty |
| Cooperative trajectory topic | Intention epoch plus `TimedTrajectory` and freshness/identity metadata |
| Core result | Timed local trajectory, target twists, maneuver state and diagnostic topics |

Perform TF conversions at consistent measurement times. Convert ROS clock epochs
to one monotonic relative mission timeline; do not mix wall-clock seconds,
simulation time and message stamps. Drop/handle out-of-order and future tracks.
Preserve uncertainty/freshness rather than labeling every robot cooperative.
Reset planner progress/commitment for changed global missions, localization jumps,
time jumps, or lifecycle transitions. Synchronize observation snapshots and access
to each planner instance outside its implementation.

## A — Local timed trajectory modification

The core produces a time-parameterized local trajectory from the current global
path, observed moving agents and static geometry. A compatible controller must
follow its timing/speed profile. A custom stamped trajectory message or explicit
trajectory-and-velocity interface is needed: ordinary `nav_msgs/Path` does not
express velocity/time profiles and publishing geometry alone cannot enforce a
cooperative crossing time. Test tracking errors, delayed plans, time jumps and
controller refusal/expiry. Recheck occupancy when a controller slows or deviates.

This architecture keeps trajectory selection and low-level tracking separate and
permits comparisons with the standalone follower. It requires careful integration
of path replacement, timing semantics and Nav2 goal/progress checking. It should
not silently commandeer Nav2's global path or bypass recovery behavior.

## B — Custom Jazzy Nav2 controller

A ROS package may implement `nav2_core::Controller`, owning one core planner and
an appropriate tracker. Jazzy's interface includes lifecycle configure/activate/
deactivate/cleanup, `setPlan`, `computeVelocityCommands`, and `setSpeedLimit`.
The command method accepts pose, current twist and a goal checker and returns a
stamped velocity command. The wrapper would snapshot costmap/tracked state,
construct `PlannerRequest`, call the core, follow the selected short-horizon
trajectory, and report yielding/failure through defined Nav2 semantics.
Speed limits must be applied to the core and tracker consistently; cancellation
and recovery need bounded braking. The official [controller plugin tutorial](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/)
describes plugin export and integration. The local workspace's
`navigation2/nav2_core/include/nav2_core/controller.hpp` was also inspected for
the actual Jazzy-shaped interface; the future adapter must compile against its
selected Nav2 release.

This approach gives the adapter direct control of time-indexed trajectory
execution but makes lifecycle, threading, cancellation and costmap integration
part of the controller's responsibility. The standalone core remains untouched.

## MPPI component reuse

The official [Jazzy MPPI configuration guide](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
documents trajectory critics for costs such as obstacles and path alignment.
The local `nav2_mppi_controller/critic_function.hpp` exposes `score(CriticData&)`.
Our pure oriented geometry, deformation/reference representation, and time-indexed
prediction evaluator could inform a cooperative critic over MPPI's candidate
trajectories. Adapter code must align MPPI trajectory times, footprint conventions,
costmap data and obstacle observation epochs.

A weighted critic alone does not guarantee the core's hard-constraint semantics,
timing commitment or bilateral agreement. Determine how infeasible batches,
fallback controls, intended-trajectory publication and post-selection validation
would work before claiming equivalent behavior. A standalone deformation planner
and an MPPI critic are different experiments and should share regression inputs.

## Evaluation before choosing production architecture

Replay the same straight, narrow, crossing, stale-message and sudden-stop
encounters through both adapters. Compare tracking error envelopes, actual
clearance, latency sensitivity, CPU tails, stopping behavior, progress checker
interaction, goal completion and recovery. Add acknowledged intentions and
fault-injection tests if genuine negotiation is required. No final production
architecture is selected in V0.1; MuJoCo validation and physical braking
characterization remain separate steps.
