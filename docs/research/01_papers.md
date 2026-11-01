# Analysis of the five supplied papers

This document answers what each method actually optimizes, what it assumes, why
it works, where it fails, and what is transferable to our differential-drive AMRs.
Recommendations are our engineering judgment. Paper results retain their original
robot models and test conditions. Supplemental sources and subsequent corrections are in the [expanded review](deployment_review/01_paper_review.md).

## 1. Werling et al.: trajectory generation in a Frenet frame

**Source W:** Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun,
*Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*,
ICRA 2010, pp. 987-993, DOI `10.1109/ROBOT.2010.5509799`.
[Supplied PDF](../references/01_existing_pdfs/01_Werling_2010_Frenet.pdf).

### 1.1 The actual problem and its structure

Werling's problem is a car moving near a reference route, given behavioral
objectives and predictions of surrounding traffic. It is a **single-ego trajectory
generator**, not a fleet coordinator or an algorithm for discovering a route
through an arbitrary warehouse.

The representation is

\[
  p(t)=r(s(t))+d(t)n_L(s(t)),
\]

where `s` is reference arc length and `n_L` is the left normal used in the paper.
The algorithm generates lateral and longitudinal candidate families, transforms
their combinations into Cartesian trajectories, rejects infeasible/colliding
ones, and selects by costs. A behavioral layer supplies goals such as desired
speed, following, stopping, or merging. The paper explicitly says the short
planning horizon does not replace farsighted behavior decisions. [W, PDF pp. 2-3,
6-7, Sections I-III, VI-IX.]

This is unusually close to our original intent: follow a route, move aside for a
local conflict, then return. Its most useful contribution is the structured
search over *where and when* a maneuver ends, not merely use of a fifth-degree
curve.

### 1.2 Why quintic polynomials arise

For fixed initial and final scalar position, velocity and acceleration, minimize

\[
 J_t[p]=\int_0^T \left(p^{(3)}(t)\right)^2 dt.
\]

The unconstrained variational solution is a polynomial of degree at most five.
Werling adds duration and terminal-error terms and samples terminal conditions
and durations within this family. A standard lateral objective is

\[
 C_d=k_jJ_t[d]+k_tT+k_dd(T)^2.
\]

The result is not an unrestricted constrained-trajectory optimum: collision and
vehicle constraints can require shapes outside the free-problem polynomial
family. The paper makes this approximation explicit. [W, PDF pp. 3-4, Proposition
1 and Section IV-A.]

The familiar `q(u)=10u^3-15u^4+6u^5` is only the special connection from zero
initial velocity/acceleration to zero final velocity/acceleration. Our current
Hermite entry also accepts an initial slope but resets the second spatial
derivative. It is not the full arbitrary-boundary-state connection or a proof of
minimum physical jerk.

For implementation, normalize the interval with `u=t/T` (or `u=(s-s0)/S`). Set
the first three coefficients from the starting position and first two derivatives,
then solve a fixed 3-by-3 system for the remaining coefficients to match the final
three conditions. This is fast, deterministic and avoids a general nonlinear
optimizer. Scale derivatives by `T,T²` or `S,S²`; confusing these changes units
and can produce an apparently smooth but incorrect connection.

### 1.3 Low speed is central to our adaptation

Werling distinguishes:

* **Higher-speed mode:** lateral motion `d(t)` and longitudinal motion `s(t)` are
  generated separately, then combined and checked.
* **Low-speed mode:** lateral geometry is `d(s)`, with longitudinal timing applied
  through `s(t)`. This avoids requesting lateral movement while forward progress
  is near zero, which otherwise produces excessive/nonholonomic curvature.

The spatial smoothness cost becomes

\[
 J_s[d]=\int_{s_0}^{s_1}(d'''(s))^2ds,
 \qquad C_d=k_jJ_s+k_tS+k_dd(s_1)^2.
\]

The paper's low-speed discussion is on PDF p. 4, Section IV-B; the candidate
boundaries continue on p. 5. We should adopt this formulation for ordinary AMR
passing. The choice should depend on conditioning and maneuver type, not import
a road-vehicle speed threshold or assume that 1.5 m/s is a high-speed regime.

**Crucial limit:** low-speed Frenet still does not represent a differential-drive
robot rotating in place or backing out of a bay with one monotonic route
coordinate. Include explicit rotate/stop/reverse SE(2) primitives when permitted.
Switching representation must preserve actual pose, twist and acceleration
bounds; it is not a license to jump from one curve to another.

Spatial and temporal jerk are different. With constant `s_dot=v` and
`d=D*q(s/S)` on a straight reference,

\[
  \int_0^{S/v}\dddot d(t)^2dt=720D^2v^5/S^5.
\]

For `D=0.85 m`, `S=3.2 m`, the values at `0.5,1.0,1.5 m/s` are approximately
`0.04845,1.55032,11.77272 m²/s⁵`. The same shape is not the same comfort demand.
This is our derivation, reproduced by [research_checks.py](../../scripts/research_checks.py),
not a performance result reported by Werling.

### 1.4 Longitudinal modes we should actually implement

Werling uses quintic connections when terminal position matters (following,
merging and stopping). Speed keeping has free terminal position and specified
terminal velocity/acceleration, so a quartic suffices. [W, PDF p. 5, Section V.]

| Mode in the paper | AMR translation | Necessary extra condition |
| --- | --- | --- |
| Velocity keeping | Maintain route speed where visibility/geometry allows | Respect wheel, turn, braking and uncertainty limits |
| Following | Follow a slower AMR | Derive gap from braking and latency; a time-gap heuristic alone is insufficient |
| Stopping | Stop before a gate, obstacle or blocked region | Full footprint stays outside the exclusion zone during deceleration |
| Merging | Reach a place/time between traffic participants | Replace highway heuristics by a feasible conflict-region time window |
| Lateral return | Target `d=0`, aligned heading and derivatives | Keep collision evaluation active through return and continuation |

These give a crossing solution that our fixed speed-scale set often misses:
arrive just after another agent clears, instead of choosing a large lateral
offset or a full stop. Sample a small set of target arrival times/speeds, not only
scale a pursuit rollout's maximum speed.

### 1.5 Temporal consistency is conditional

Werling starts continuation from the previously planned trajectory state and
retains absolute terminal-time samples so the previous solution's suffix remains
available. This supports Bellman-style consistency in the unobstructed/free
problem under its assumptions. [W, PDF pp. 3-4, Fig. 1 and Fig. 3 discussion.]

It does **not** prove that an obstacle-constrained sampled planner with changing
predictions remains globally optimal, recursively feasible or stable. The paper
also distinguishes its reactive heuristic from the free case. Our implementation
should therefore:

1. Include the time-shifted old trajectory as a candidate, after rechecking it.
2. Preserve the selected terminal event/time while it remains useful.
3. Reconcile small tracking errors with a bounded splice instead of independently
   resetting every derivative on every cycle.
4. Reinitialize when the measured state leaves the valid tracking tube, the route
   changes, or newly observed occupancy invalidates the continuation.
5. Let safety checks override commitment. Reusing an unsafe suffix is not temporal
   consistency.

Our current code's fixed side and envelope-change penalty provide behavioral
hysteresis, but they are not this optimal-continuation construction.

### 1.6 Geometry conditions and our sign convention

The appendix assumes `1-κr*dL>0` and heading deviation within the forward chart.
It requires reference curvature derivative `κr'` for its higher-order transforms.
A continuous curvature derivative is important for continuous output curvature.
[W, PDF p. 8, Appendix I, Eqs. 4-9.]

Our project uses a **right** normal, so `dR=-dL`; the corresponding factor is
`A=1+κr*dR`. Importing the paper's equations unchanged would invert the offset
meaning and put the singularity on the wrong side. A left-turn reference with a
right offset is the outside of the turn and has `A>1`. The inside offset can
become singular when its radius reaches the reference radius.

`A>0` is only a local regularity condition. At hairpins, crossings or a U-shaped
route there may still be multiple nearest projections. Use branch/segment IDs,
continuity constraints and a local chart, or switch to Cartesian planning. A
90-degree bend can work well in Frenet **if** it has a valid rounded reference;
an intersection is not automatically a Frenet failure and a spline is not
automatically a safe route through a corner. See [the derived equations and
code gaps](deployment_review/02_system_design.md#right-normal-frenet-conversion).

### 1.7 Collision handling and evidence

Werling combines lateral/longitudinal candidates, derives Cartesian heading,
curvature, speed and acceleration, and rejects invalid motion. Collision
acceptance is a hard binary contour test with clearance inflation, rather than
making collisions an ordinary weighted objective. The checked contour expands
toward the horizon to represent increased conservatism. [W, PDF p. 6, Section VI.]

This supports our hard-feasibility-first design. It does not provide a calibrated
probabilistic occupancy model or a braking-safety proof under arbitrary surprise
obstacles. Likewise, expanding a contour by a hand-tuned amount is not equivalent
to an uncertainty confidence bound.

Evidence is narrower than many retellings suggest: the paper reports on-vehicle
testing of nonreactive motion without obstacles at a 100 ms planning cycle. Its
dangerous highway avoidance demonstration, with behavioral restrictions disabled,
is simulated; the authors defer practical reactive validation. [W, PDF pp. 6-7,
Section VIII.] It is not evidence that our 150 kg loaded AMR can safely pass at
1.5 m/s in a blind warehouse corner.

**Adoption decision:** use Werling as the main candidate-generation foundation,
starting with low-speed `d(s)` and multiple longitudinal modes. Build the reference
geometry, continuity handling and Cartesian feasibility conversion first. Keep
free-space/reverse recovery and bottleneck ordering outside this parameterization.

## 2. Rösmann et al., ROBOTIK 2012: the original TEB

**Source T12:** Christoph Rösmann, Wendelin Feiten, Thomas Wösch, Frank Hoffmann
and Torsten Bertram, *Trajectory Modification Considering Dynamic Constraints of
Autonomous Robots*, ROBOTIK 2012, printed pp. 74-79.
[Supplied PDF](../references/01_existing_pdfs/02_Roesmann_2012_TEB.pdf).

### 2.1 Representation and optimization

The band contains poses `q_i=(x_i,y_i,θ_i)` and positive time intervals `ΔT_i`:

\[
 B=(Q,\tau),\qquad \min_B\sum_k\gamma_k f_k(B).
\]

Instead of only stretching a geometric rubber band, the optimizer moves poses
and changes their timing. Local factors penalize path/waypoint deviation,
obstacle proximity, speed, acceleration and nonholonomic inconsistency. Most
factors involve two or three neighboring poses. That locality creates a sparse
optimization problem. [T12, PDF pp. 2-4, Sections 2-2.5.]

Approximate translational and angular speeds are chord/heading differences over
`ΔT`; acceleration uses adjacent segment velocities:

\[
 v_i\simeq\|p_{i+1}-p_i\|/\Delta T_i,
 \quad\omega_i=\Delta\theta_i/\Delta T_i,
 \quad a_i\simeq2(v_{i+1}-v_i)/(\Delta T_i+\Delta T_{i+1}).
\]

The nonholonomic factor makes consecutive pose headings compatible with their
connecting arc. It additionally addresses 180-degree ambiguities. The paper
explicitly relates differential-drive wheel velocities to `v±Lω`, where its `L`
is **half** the wheel separation. [T12, PDF p. 3, Eqs. 9-17.] That is directly
relevant to our robot; using the car-like 2017 model unchanged is not.

### 2.2 What makes it effective

Geometry and time can trade off usefully: slow to make a curved segment feasible,
stretch a transition to respect acceleration, or shorten an unnecessarily long
detour. The previous band provides a strong warm start. Insert/delete operations
adapt resolution, with hysteresis to prevent oscillatory resampling. g2o exploits
the sparse graph structure. [T12, PDF p. 4, Fig. 6.]

This is a useful refinement strategy for a polynomial candidate that almost
works, or for a tight corner where a single offset profile is too restrictive.
It is not a reason to optimize every robot's entire mission every 20 ms.

### 2.3 Constraints are approximate penalties

The implemented formulation converts constraints into weighted smooth penalties.
Finite weight means a violation can remain in the optimum. Increasing weights
may worsen conditioning. A small objective value is not an execution certificate.
The paper's own control flow has a trajectory verification step. [T12, PDF pp. 2,
4, Fig. 3 and Fig. 6.]

For our use, allow the refiner to propose candidates, then independently verify
oriented footprint clearance, timing, motion and actuator limits. Infeasible
optimization output must fall back to the last valid continuation/braking policy.
Do not tune penalty weights until collisions merely disappear in a demonstration
and call that a hard constraint.

### 2.4 The time objective has a practical subtlety

T12 Eq. 18 is `(Σ ΔT_i)²`, a global term. The paper notes that its time-variable
block becomes dense. A later local sum of squared intervals is computationally
different, not just a notation change.

For fixed total time, `(Σ ΔT_i)²` alone cannot determine interval uniformity.
For `Σ ΔT_i²`, equal intervals minimize cost at fixed total time, but increasing
the number of intervals also lowers that cost. Example: 2 s split into 10 versus
20 equal intervals produces `0.4` versus `0.2 s²`. Adaptive discretization and
objective interpretation must be considered together. This is our algebraic
observation, not a claim that the implementation blindly varies its point count
to cheat the cost. A new refiner should report actual traversal time separately
and compare solutions consistently across resolutions.

### 2.5 Evidence, limitations and adoption

The paper demonstrates waypoint and static-obstacle cases and a Pioneer 2
avoiding a walking person. It reports a 25 ms control cycle on a Core2Duo laptop;
that is not a p99 solver-time benchmark for a fleet. The person experiment shows
reactive deformation as detections change, not prediction of arbitrary shared
future robot trajectories. [T12, PDF pp. 5-6, Figs. 9-13.]

The conclusion explicitly says it is local optimization, without global-optimality
guarantees, and discusses exploring the opposite side of obstacles as future
work. It does not solve bottleneck deadlocks or show loaded dynamics.

**Adoption decision:** reuse the pose/time, wheel-constraint and sparse-factor
ideas. Keep numerical refinement optional and budgeted. Preserve explicit hard
verification and topology seeds; do not replace our modular core with the ROS
package merely to obtain the method.

## 3. Rösmann et al., IROS 2017: car-like TEB refinement

**Source T17:** Christoph Rösmann, Frank Hoffmann and Torsten Bertram,
*Kinodynamic Trajectory Optimization and Control for Car-Like Robots*, IROS 2017,
pp. 5681-5686, DOI `10.1109/IROS.2017.8206458`.
[Supplied PDF](../references/01_existing_pdfs/03_Roesmann_2017_TEB.pdf).

This is **not** *Integrated Online Trajectory Planning and Optimization in
Distinctive Topologies*, a separate 2017 paper by the same authors. Treating
the supplied file as the latter would misattribute its contributions.

### 3.1 Model and control elimination

The paper models a bicycle-like car with rear-axle reference pose, longitudinal
speed `v`, steering angle `φ`, wheelbase `L`, and

\[
 \dot x=v\cos\theta,\quad\dot y=v\sin\theta,
 \quad\dot\theta=(v/L)\tan\phi.
\]

It assumes motor controllers follow commanded speed/steering. Forces, torques,
mass and inertia are not modeled. Speed and acceleration bounds are included;
the word *kinodynamic* here does not mean a full dynamic vehicle/load model.
[T17, PDF p. 3, Section II-A.]

Controls are inferred from the optimized pose/time sequence rather than making
all controls independent decision variables. A signed velocity distinguishes
forward/reverse motion; adjacent poses must lie on a consistent circular arc;
a minimum-turning-radius constraint imposes the car steering limit. [T17, PDF
pp. 3-4, Eqs. 1-9.]

**AMR adaptation:** our differential drive can turn in place and does not have
this car wheelbase/steering relation. Keep `v,ω` and wheel limits, angular
acceleration and permitted reverse motion. A configured curvature limit may
still be a useful operating envelope during translation, but it should not
forbid legitimate zero-speed rotation by accident.

### 3.2 Sparse approximation and feedback

The ideal NLP minimizes `Σ ΔT_i²` under start/goal, kinematic, timing, obstacle
and actuator constraints. The actual solver uses equality and one-sided
inequality penalties with sparse Levenberg-Marquardt in g2o. The paper explains
why finite penalties abandon the exact constrained minimizer, and why enormous
weights cause numerical difficulties. [T17, PDF pp. 4-5, Eqs. 11-14.]

Its outer loop updates the band, adapts point count using a target interval with
hysteresis, updates obstacles, solves, checks feasibility and extracts the next
command. The optimized band warm-starts the next cycle. A shrinking horizon is
used toward a fixed local goal. [T17, PDF p. 5, algorithm in Section II-D.]

Use this lifecycle if we introduce a refiner: cap iterations/time, preserve the
best independently accepted iterate, check the controller's next command, and
record both failure-to-converge and residual constraint violation. Solver
termination is not synonymous with a valid trajectory.

### 3.3 Initialization, reversals and topology

A seed biases which local solution is found. The paper studies forward/reverse
parking and shows competing minima. Reeds-Shepp comparisons concern obstacle-free
car geometry with acceleration constraints disabled for that comparison; they
are not a globally optimal solution of the warehouse fleet problem. [T17, PDF
p. 6, Sections III-A/B.]

For AMRs, give a local refiner distinct seeds: pass right, pass left, wait before
the conflict, stop/rotate, and approved reverse-to-bay recovery. Do not expect
warm-starting one band to discover a different passage through a wall or to
choose a fundamentally different agent ordering. Homotopy class search is a
separate mechanism from local optimization.

### 3.4 Obstacle and experimental scope

Rectangular car footprints against polygon obstacles are used in the parking
analysis. The formulation allows prediction models to be added to obstacle
constraints, but its main evidence is not a multi-agent arbitrary-intent
prediction system. It confirms feasibility with an external collision check
before applying commands. Stability analysis is explicitly outside the paper's
scope. [T17, PDF pp. 4-6.]

Reported CPU times are 2.6-35.4 ms for selected converged obstacle-free cases on
a 3.4 GHz i7, with 27-104 waypoints. Real experiments use a Lego EV3 car, motion
capture localization and a 0.15 m/s speed limit. The sensor/local-goal scale is
about 1 m. [T17, PDF pp. 6-7, Table I and Fig. 5.] These validate useful mechanics,
not our embedded 20-AMR throughput or loaded braking behavior.

**Adoption decision:** use it as a reference for bounded SE(2) trajectory
refinement and reverse/parking recovery. Do not transplant Ackermann constraints
or claim its experiments validate our robot model. A first Frenet baseline does
not require this entire solver.

## 4. Chung et al.: Distributed Timed Elastic Band

**Source D:** Yiu Ming Chung, Hazem Youssef and Moritz Roidl,
*Distributed Timed Elastic Band (DTEB) Planner: Trajectory Sharing and Collision
Prediction for Multi-Robot Systems*, ICRA 2022, pp. 10702-10708,
DOI `10.1109/ICRA46639.2022.9811762`; uploaded as arXiv v1 in March 2023.
[Supplied PDF](../references/01_existing_pdfs/04_Chung_2022_DTEB.pdf).

### 4.1 What it adds beyond local TEB

DTEB combines four distinct mechanisms:

1. TEB trajectory optimization with other robots as **time-indexed** obstacles.
2. Exchanging trajectories, timestamps, footprints and priorities.
3. Alternative trajectory/topology seeds, including a narrow-aisle roadmap.
4. A two-stage priority policy for longer-range ordering and nearby reciprocal
   avoidance.

Its assumptions include communicating robot computers, heterogeneous/nonholonomic
agents, and asynchronous planning at 20 Hz. This is not the same as a synchronized
central joint trajectory optimization. [D, PDF pp. 1-2, Section III.]

### 4.2 Shared trajectories are better than straight-line extrapolation

At each optimized ego timestamp `T_k`, the obstacle is evaluated at `O_j(T_k)`.
Summed intervals define absolute-relative arrival times. Other-agent pose is
interpolated along the shared path; the paper uses straight interpolation for
holonomic motion and circular-arc propagation for nonholonomic motion:

\[
 x(t+h)=x(t)+\frac v\omega[\sin(\theta+\omega h)-\sin\theta],
\]
\[
 y(t+h)=y(t)-\frac v\omega[\cos(\theta+\omega h)-\cos\theta].
\]

Use the straight-line limit near zero yaw rate. Then transform the actual
footprint to that pose for separation evaluation. [D, PDF p. 3, Eqs. 11-12.]

This matters directly at our L bend. A world-velocity CV model sends a turning
robot straight through a wall; a fresh credible curved intention predicts the
turn. However, a timestamped intended plan is a **claim about future execution**.
It must be checked against current observed state and actual progress.

The paper describes ROS pub/sub and timestamp compensation. Its comment that
TCP handles packet loss concerns transport delivery, not bounded latency,
freshness, process failure, clock skew or whether the robot executed the plan.
Do not turn this into a distributed safety guarantee. [D, PDF p. 3, Section III-D.]

### 4.3 Priorities are not a complete negotiation protocol

At larger separation, lower-priority agents avoid higher-priority intentions.
The paper assigns higher priority to newer tasks, and allows a robot to raise
priority when no feasible local solution exists. At close range all robots are
considered as obstacles, independent of priority, enabling reciprocal response.
[D, PDF pp. 4-5, Section III-G.]

This explains why the right ordering matters in a doorway/bay: the robot that
can wait in a pocket may need to yield before the other commits to a passage.
It is not enough to brake after both are inside a single-width tunnel.

The paper does not provide an acknowledged, versioned reservation transaction
with proven behavior under partitions or simultaneous escalations. Its results
demonstrate useful avoidance in selected scenarios; they do not establish
deadlock freedom for arbitrary maps and fleets.

For our design, borrow conflict-local ordering and escalation, but do not let
priority suppress immediate physical obstacle validation. Use a stable total
order, bounded priority aging and an explicit conflict epoch. When mutual
exclusion is required, an authoritative critical-region reservation is simpler
to validate than both robots independently increasing their rank.

### 4.4 Topology sampling and the H-signature caveat

DTEB notes that random roadmaps waste samples in warehouse walls. Its generalized
Voronoi approach generates candidate routes along free-space structure; it reports
10-20 ms for diagram construction and search on a 300-by-300 local costmap.
[D, PDF p. 4, Section III-F and Fig. 4.] This is a promising way to seed alternate
routes, not proof that a point-centerline is wide enough for rotating AMR bodies.
Annotate seeds with orientation/clearance and validate full footprints.

Its space-time H-signature replaces a more expensive integral with a time-varying
planar field. Importantly, the paper itself analyzes its **nonzero curl** and
describes the equivalence as an approximation under suitable separation/motion
conditions. [D, PDF pp. 3-4, Eqs. 13-18.]

Consequently, do not use exact equality of these approximate signatures as a
collision or topology certificate. Two collision-free paths that continuously
deform into one another can have slightly different signatures around a moving
obstacle. Our reproducible example gives `0.0576764` and `0.0557658` for two
same-endpoint, same-class forward-time paths, without crossing the singularity.
This does not refute its usefulness for heuristic candidate diversity. It shows
why tolerance and explicit validation are needed, especially near tight
encounters where the approximation is weakest.

Our first implementation should use explainable labels such as side, corridor
branch and before/after conflict ordering; do not claim those labels exhaust all
homotopy classes. Add sophisticated signatures only if benchmarks show the
simpler diversity mechanism loses needed solutions.

### 4.5 Evidence and its limits

The six-robot circular Stage test is repeated five times on a Xeon X5675 at
3.07 GHz. The simulation runs at 60% speed because multiple planner instances
are computationally burdensome. The reported comparison is:

| Metric | DTEB with priorities | HRVO |
| --- | ---: | ---: |
| Total travel distance | 62.16 ± 1.06 m | 60.64 ± 0.34 m |
| Completion duration | 12.86 ± 0.53 s | 14.92 ± 1.22 s |
| Computation per robot | 10.30 ± 9.45 ms | 3.94 ± 0.70 ms |

[D, PDF p. 5, Table I.] This is evidence of a tradeoff, not universal superiority:
the paper also notes differences in the goal-homing behavior of the HRVO setup.

The eight-robot warehouse results report about 20 ms average computation per
robot, with about 9 ms standard deviation. Completion is 22.80 s versus 43.58 s
for TUW in one layout/task, and 44.42 s versus 47.30 s in the asymmetric task.
The TUW computation entries refer to the whole offline routing computation,
unlike DTEB's online cycle. [D, PDF p. 6, Table II and footnote.] Comparing those
numbers as if both were online per-robot timings would be misleading. The paper
calls for real-robot validation in future work.

**Adoption decision:** this is our strongest source for cooperative architecture.
Use its arbitrary timed intentions, topology-aware alternatives and priority
reasoning. Add explicit intent-validity and reservation semantics; keep the
optimizer optional. Do not equate publishing intentions with an agreement or
repeat its reported fleet timing as our own budget.

## 5. van den Berg et al.: ORCA

**Source O:** Jur van den Berg, Stephen J. Guy, Ming Lin and Dinesh Manocha,
*Reciprocal n-body Collision Avoidance*.
[Supplied 16-page manuscript](../references/01_existing_pdfs/05_VanDenBerg_ORCA.pdf). The author project lists
the work in the 2011 ISRR proceedings volume; the local PDF metadata predates
that publication. Analyze the supplied manuscript's assumptions, rather than
silently adding properties from later ORCA extensions.

### 5.1 The velocity-space construction

For disc agents A/B, the relative velocity obstacle contains velocities that
intersect the combined footprint before horizon `τ`:

\[
 VO_{A|B}^{\tau}=\{v:\exists t\in[0,\tau],\;tv\in
 D(p_B-p_A,r_A+r_B)\}.
\]

Find the shortest correction `u` from the chosen relative optimization velocity
to the boundary of this set. With outward normal `n`, each reciprocal agent
takes half the correction:

\[
 (v-(v_A^{opt}+u/2))\cdot n\ge0.
\]

Intersect such half-planes with the speed disc, then select the permitted velocity
closest to the preferred velocity. [O, PDF pp. 4-9, Eqs. 2, 5-9.] The preferred
velocity guides progress, while optimization velocity anchors the avoidance
construction; conflating the two changes feasibility and behavior.

The low-dimensional solver has expected linear cost in the number of included
constraints. Efficient neighbor queries and independent agent solves enable
large crowds. This is not a claim that complete sensing, tracking, neighbor
building and rectangular dynamic feasibility all become linear automatically.

### 5.2 Exact scope of the guarantee

The supplied analysis assumes holonomic agents, exact observable state and shape,
and the same reciprocal selection strategy on both sides. Instantaneous new
velocities must be realized. The disc derivation can extend to translating convex
polygons, but rotating rectangles add another dimension of motion not solved by
simply substituting a polygon in the same 2D velocity program. [O, PDF pp. 2, 4.]

With current velocity as the optimization velocity, a dense configuration can
make the admissible intersection empty. The fallback minimizes maximum constraint
penetration using a 3D LP; avoiding collisions is then not guaranteed. A
zero-optimization-velocity variant retains a zero-velocity solution in an
initially nonoverlapping ideal configuration, but can deadlock. Static obstacles
take full, not half, avoidance responsibility; their constraints are not relaxed
in the dense-case fallback. [O, PDF pp. 10-13, Sections 5.2-5.4.]

This is more precise than saying either “ORCA guarantees collision avoidance” or
“ORCA has no guarantee.” It has a useful sufficient condition under specified
feasibility, observation and execution assumptions.

### 5.3 Why naïve differential-drive conversion fails

A rectangular AMR cannot immediately adopt an arbitrary planar velocity while
its body orientation and actuators remain bounded. Rotating toward an ORCA
velocity changes both the swept footprint and the future relative trajectory.
Acceleration clipping can leave it inside the velocity obstacle during the
transition. The supplied paper explicitly says post-clamping kinematics/dynamics
does not retain the strict guarantee. [O, PDF p. 15, Section 7.]

An acceleration disc in planar velocity space is also not a complete fix:
differential-drive reachability is constrained by heading and yaw dynamics, and
finite transition occupancy must still be checked. An ORCA half-plane checked
only at the first trajectory point cannot certify an accelerating curved rollout.

There is a separate published **ORCA-DD** method, by Snape et al., with a
specialized differential-drive construction and real iRobot Create experiments.
It should be an additional baseline candidate, not treated as a property already
proved by the supplied manuscript. The author project is linked in
[the ORCA-DD review](deployment_review/01_paper_review.md#r14--orca-dd-differential-drive-constraints).

### 5.4 Geometry and non-cooperative agents

Conservative circumscribed discs are acceptable for a clearly labeled benchmark,
but they reduce passing capacity. Our `0.8×0.6 m` rectangle has a 0.5 m enclosing
radius. Two aligned rectangles need `1.2 m` of physical side-by-side width, while
their discs need `2.0 m`; adding 0.15 m at both walls and between robots gives
`1.65 m` versus `2.45 m`. Rotation may increase rectangle requirements. This is
our illustrative calculation, not a claim that every such corridor is traversable.

Never assign half responsibility to a person, stopped forklift, unknown AMR or
robot that has not adopted the compatible protocol. Treat them as independently
moving obstacles with a conservative response model. Shared branding or an ID
does not prove reciprocal compliance.

### 5.5 Evidence and adoption

The manuscript reports 1,000-agent demonstrations and, on eight Xeon cores,
about 8 ms for 5,000 circle agents and 15.6 ms for an office scenario. These are
ideal simplified-agent simulation workloads; they are not 5,000 physically
accelerating rectangular AMRs. It does not provide global route planning:
preferred velocities must guide agents around obstacles. [O, PDF pp. 13-15,
Figs. 8-10 and Section 7.]

**Adoption decision:** implement an independently measured ORCA/ORCA-DD baseline,
and borrow reciprocal responsibility and neighbor pruning. Use ORCA proposals
as hints or optional scoring in our executable trajectory search, not as an
unvalidated high-speed safety filter. A single-step constant-velocity obstacle
must not reject a curved candidate unless the rejection is conservative for
that entire candidate's possible motion.

## 6. Comparative conclusions

| Requirement | Werling | TEB 2012 | TEB 2017 | DTEB | Supplied ORCA |
| --- | --- | --- | --- | --- | --- |
| Primary representation | Route-relative scalar polynomials | Poses + time intervals | SE(2) poses + time intervals | TEB + others' timed trajectories | Planar velocities |
| Target model | Road car near smooth route | Differential drive | Car-like, reversible | Heterogeneous/nonholonomic | Ideal holonomic; disc derivation |
| Joint timing/geometry | Candidate combinations | Optimized together | Optimized together | Optimized with predicted occupancy | One new velocity over horizon |
| Opposing-agent intent | Predicted traffic input | Not a shared-plan protocol | Prediction can be added | Explicitly broadcast | No communication required |
| Narrow corridor ordering | Behavioral layer needed | Not solved | Not solved | Priority + topology heuristics | Not generally solved |
| Constraint mechanism | Candidate rejection | Penalty approximation + verify | Penalty approximation + verify | Penalties + collision prediction | Half-plane sufficient conditions if feasible |
| Return to nominal route | Natural terminal `d=0` target | Via points/goal objective | Via points/local goal | Via band/goal | Requires preferred-velocity/path policy |
| Useful role here | Main next generator | Differential-drive factors | Optional recovery/refinement | Cooperation architecture | Fast comparison baseline |

No paper removes the need to decide **who goes first**, establish **what was
actually observed or communicated**, and verify **what the bounded actuator will
execute**. The next useful design is the smallest set of separate components
that closes those gaps. [The updated design](deployment_review/02_system_design.md) gives that design and the
experiments needed before judging it robust.
