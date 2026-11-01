# Paper-by-paper review and adoption decisions

Review date: **2026-11-01**. Scope: the five existing PDFs, all 23 linked downloads, and all 69 extended-catalog records in `docs/references/paper_manifest.json`. Their union contains **70 distinct works: 68 papers and two software documentation records**. ORCA-AVO is present in the download list but has no separate extended-catalog row; it is R70 below. Repeated versions and repeated download links do not count as additional works.

The [source ledger](sources.json) provides canonical local paths, download sources, hashes, version notes, and access failures. **A downloaded PDF is not automatically a fully verified result.** No external planner's experimental results were reproduced. Paper results below are author-reported; adoption decisions and counterexamples are our analysis.

Evidence labels:

- **A — detailed full-text review:** main method, relevant equations/assumptions, experimental evidence, and limitations examined. This does not mean every reference or appendix proof was independently verified.
- **B — selected full-text review:** method and evaluation sections examined; broader claims and all proofs have not been exhaustively checked.
- **C — abstract/project-page review:** primary publication or author record checked; full-text access was unavailable or insufficient. No theorem or implementation endorsement is inferred from an abstract.
- **D — documentation review:** versioned software documentation, not a research paper.

Some abstracts and project pages report strong guarantees. When their proof assumptions were unavailable, the review explicitly leaves those guarantees unverified. These C entries remain an access-limited part of the requested in-depth review.

## Trajectory generation, optimization, and interaction

### R01 — Werling et al., 2010: Frenet trajectory generation

**A.** [Primary record](https://doi.org/10.1109/ROBOT.2010.5509799). Local source: `01_existing_pdfs/01_Werling_2010_Frenet.pdf`; article begins after the cover. Methods and appendices motivate distinct high/low-speed formulations, polynomial boundary matching, longitudinal velocity-keeping/following/stopping modes, and rejection of infeasible candidates. At low speed, spatial `d(s)` avoids a time-only lateral description becoming ill-conditioned near a stop. Continuation of the previous optimum matters to consistency.

**Transfer:** retain the bounded candidate family and explicit timing modes. The car/road setting, reference convention, curvature limits, and endpoint assumptions need conversion for differential drive. A polynomial is neither an actuator model nor a recursive-safety argument. The repository's right normal changes signs and introduces `1+kappa*d`; [our derivation and checks](05_analytical_checks.md) cover this explicitly. Independent longitudinal speed multipliers cannot reproduce all following and arrival-time decisions. Keep SE(2) alternatives where a regular Frenet chart is unavailable. This is the strongest basis for the next inspectable candidate generator, not a claim of global optimality for our problem.

### R02 — Rösmann et al., 2012: Timed Elastic Band

**A.** [Authors' implementation](https://github.com/rst-tu-dortmund/teb_local_planner); local original `02_Roesmann_2012_TEB.pdf`, all six pages examined. TEB jointly adjusts poses and time differences, representing local objectives and constraints as optimization residuals. Its differential-drive treatment includes wheel-related kinematics; the original total-time-squared objective has different coupling from a sum of squared segment durations.

**Transfer:** useful sparse optimization structure and joint geometry/timing refinement. Finite residual weights do not make collision or drive limits inviolable. Local minima, an inappropriate initial passing class, and stopping feasibility remain separate problems. Refinement should start from accepted maneuver seeds and pass an external acceptance check. Do not call a changed time objective equivalent merely because both favor shorter duration. The catalog links a repository, so distinguish this 2012 paper from features subsequently added to the package. The existing five-paper analysis provides additional equation-level discussion.

### R03 — Rösmann et al., 2017: kinodynamic control for car-like robots

**A.** [Primary record](https://doi.org/10.1109/IROS.2017.8206458). Original PDF `03_Roesmann_2017_TEB.pdf`; cover plus six article pages examined. SE(2) pose/time optimization accommodates reversing, nonholonomic motion, adaptive discretization, and warm starts. The paper also uses external feasibility checking before command execution. Its reported small-robot demonstration and scenario computation times are not a loaded industrial-AMR benchmark.

**Transfer:** use reverse seeds and adaptive local refinement when simpler candidates demonstrably miss feasible maneuvers. Car minimum steering radius is not differential-drive wheel physics. Summed squared time differences are resolution dependent, so changing sample density changes more than numerical precision. Reject the inference that sufficiently large penalty weights establish hard feasibility. A solver returning a numerical solution still requires footprint, dynamics, timing, and fallback acceptance. Full-text access came from the user's PDF even though the online metadata request was rate-limited.

### R04 — Xin et al., 2022: distributed MPCC

**A.** [Publisher record](https://doi.org/10.23919/CSMS.2022.0017), [author manuscript](https://jianbinxin.github.io/pubs/2022_CSMS.pdf). PDF pp.3–13 examined; Eq.4 on PDF p.5 rendered and inspected. The controller balances contour/lag error and speed tracking around a reference, with multi-circle/ellipse geometry and communicated peer predictions. The experiments use small cooperative teams and a short prediction horizon; communication faults are not the demonstrated problem.

**Transfer:** contouring/lag costs and progress consistency are useful alternatives to purely geometric tracking. Do not treat all MPCC formulations as identical: this paper's progress update and speed objective differ from a freely optimized virtual progress reward. The printed `a²+r²,b²+r²` collision denominators fail a simple circle-sum geometry check without additional inflation; see [the counterexample](05_analytical_checks.md#ellipse-inflation). Its flowtime improvement is not a makespan improvement of the same magnitude. Use this as an optimization reference, with independent geometry and communication validity checks, rather than importing a safety certificate.

### R05 — Schöneberg et al., 2025: MPC with prediction uncertainty

**A.** [Primary record](https://doi.org/10.1016/j.ifacol.2025.10.245), [public manuscript](https://arxiv.org/abs/2504.19193). Six-page body examined and uncertainty equations rendered. A VAR/Gaussian predictor produces time-varying uncertainty used in an acceleration-controlled unicycle MPC. Two simulated skid-steer robots illustrate the approach. Typical solver times and occasional large spikes differ substantially; a nominal planning period is not a worst-case latency bound.

**Transfer:** an inspectable estimation-to-occupancy interface is valuable. The ellipse-plus-radius construction is approximate, and optimizing confidence size through a soft penalty does not establish an immutable risk threshold. Pointwise Gaussian coverage does not establish episode collision probability. Recheck the actual inflation geometry, calibrate on held-out local traffic, and allocate risk across agents/time if a probabilistic gate is used. The study does not validate 1.5 m/s loaded braking or a complete mixed static/dynamic hardware stack. Use as a practical uncertainty baseline, not a fixed “95% safe navigation” guarantee.

### R06 — Sun et al., 2025: CL-CBS with MPC-APF

**C.** [Publisher abstract](https://doi.org/10.1016/j.simpat.2025.103122). The framework combines coordinated global car-like conflict search, temporally informed quintic fitting, local MPC/APF replanning, and tracking in a virtual warehouse. Full equations, execution validity conditions, and complete evaluation were not available for review.

**Transfer assessment:** relevant to the boundary between coordinated planning and local control. Ackermann kinematics and authority over the fleet differ from our differential-drive, unknown-robot requirement. A local potential-field adjustment can change the timing or geometry used to establish global separation; an interface must specify when the coordinated plan needs revalidation. Before adoption, obtain the full text and check whether local updates preserve other agents' space-time constraints, how APF local minima are handled, and what happens when the tracker cannot meet its schedule. The abstract alone does not establish these properties.

### R07 — Zhang et al.: D4orm

**A.** [Public manuscript and version history](https://arxiv.org/abs/2503.12204). Main body PDF pp.1–6 examined; downloaded revision is later than its initial 2025 submission. D4orm optimizes joint control trajectories through learning-free diffusion/Monte Carlo denoising and dynamics rollout. Experiments include simulated differential-drive/holonomic teams and physical drones. Joint optimization and many GPU rollouts are materially different from a small CPU local planner.

**Transfer:** dynamics-aware control-sequence sampling is a useful optional refiner after candidate generation. Collision enters a finite fitness formulation, so denoising is not itself an acceptance certificate. Initial-solution/deadline results must not be relabeled a measured AMR control frequency. Benchmark the full JAX/GPU dependency and candidate budget if pursued. We should first establish that candidate resolution, rather than missing observations or tactical decisions, is limiting performance. This is not a pretrained policy and does not require adopting learning just because its method uses diffusion terminology.

### R08 — Shankar, Okumura, Prorok, 2025: LF

**A.** [Public manuscript](https://arxiv.org/abs/2507.11464). Main method and evaluation PDF pp.1–7 examined. LF asynchronously couples full-horizon discrete coordination with robot-wise optimal tracking for holonomic robots under full-environment information. It uses previous-plan continuity and demonstrates coordinated aerial and ground teams.

**Transfer:** the planner/controller contract is more valuable to us than its raw agent count. Hardware ground platforms and flat tracking are not the same as acceleration-limited differential drive with local perception. The initialization procedure can perturb a planning query until feasible; moving a mathematical start is not a physical recovery action. A deployment interface must reconcile measured state, tracking tolerance, and the high-level plan rather than silently repairing input geometry. Keep LF in the later cooperative track and borrow continuity/contract ideas now. Its long-term coordination claims do not establish deadlock freedom against nonparticipating robots.

### R09 — SICNav-Diffusion, 2025

**A.** [Journal record](https://doi.org/10.1109/LRA.2025.3585713), [public manuscript](https://arxiv.org/abs/2503.08858). Main eight-page text examined. Joint multimodal diffusion predictions feed an interaction-aware controller, addressing consistency that independent marginal predictions can miss. The outdoor system includes onboard sensing/tracking, unlike the original VICON-based SICNav demonstration. Its outdoor autonomy figure measures operating time, with laptop-class GPU computation.

**Transfer:** stronger evidence that interaction-aware prediction can be integrated with real sensing, but pedestrians and vendor-specific robots are different domains. Choosing/refining one joint mode does not cover every plausible future. Measure missed modes, inference latency, fallback behavior, and unknown robot policies. Diffusion is a later experiment after CV/CTRV/map branches and track calibration. Retain a nonreactive alternative even when the interaction model expects others to give way. Autonomous-time percentage should never substitute for safe goal-completion probability.

### R10 — Trepella et al., 2026: SFM-NMPC

**A.** [Public manuscript](https://arxiv.org/abs/2607.10374), [authors' code](https://github.com/PIC4SeR/sfm-nmpc). All nine pages examined; result table rendered. The robot's candidate controls influence social-force predictions of surrounding humans inside nonlinear least squares. The reported study is simulated pedestrian navigation with multiple conventional and learning-based baselines.

**Transfer:** an unusually direct reference for candidate-conditioned forecasts and passing behind a crossing person. Social reactions remain predictions, not commands that other robots obey. The reported social/goal-success advantage comes with a travel-time tradeoff against its MPPI baseline. Hard velocity bounds and soft acceleration costs do not constitute loaded dynamic feasibility. Our tests should cross actor models and use independent safety metrics. Source inspection found a three-agent optimization state at the pinned reviewed revision; selection, freshness, and compatibility need end-to-end checks before reuse. The [GPT audit](03_gpt_claims.md#trepella-useful-interaction-model-limited-transfer) records version, license, and compatibility evidence.

### R11 — Decoupled time-optimized multi-robot path parameterization

**B.** [Primary article](https://doi.org/10.1007/s40747-025-02110-5). Methods/coordination-space construction and selected experiments examined. The method starts from specified spatial paths, obtains time-optimal profiles along them, identifies pairwise collision regions in path-coordinate space, and introduces start delays to coordinate execution. Both manipulator and car-like examples are discussed.

**Transfer:** illustrates why spatial route choice and timing are separate decisions. Time optimality along a fixed path does not imply globally time-optimal multi-robot navigation with alternative routes and maneuver orders. Pairwise forbidden delay regions introduce ordering choices; do not infer a globally convex joint problem from a final convex delay subproblem. This belongs to known-path cooperative scheduling, not forecasting unknown traffic. Check path interpolation, shape sweep, waiting occupancy, and tracker timing before adapting its timing machinery. Catalog year 2026 and online publication in 2025 are compatible.

### R12 — Qu et al., 2026: safety corridors and sequential MPC

**C.** [Publisher abstract](https://doi.org/10.1177/17298806261483759). The abstract describes multidirectional rectangular static corridors, pedestrian prediction, a robust dynamic CBF component, and point-cloud detection, evaluated in Gazebo. Full formulation/proofs were unavailable.

**Transfer assessment:** separating static free-space corridors from dynamic-agent constraints is sensible and potentially relevant to tight workspace geometry. “Robust CBF” is not sufficient evidence of feasibility under actuator saturation, sampled execution, track loss, or multiple simultaneous constraints. Before use, inspect the disturbance set, relative-degree treatment, initial safe-set conditions, relaxation policy, and infeasible-solver response. Corridors must contain the swept footprint and tracking error, not merely the optimized center. This remains a promising source to obtain in full; the abstract does not support a hardware robustness or loaded-speed claim.

## Reciprocal and distributed continuous motion

### R13 — van den Berg et al.: ORCA

**A.** [Author manuscript](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf); original user PDF retained. Pairwise velocity obstacles become reciprocal half-plane conditions; each agent selects a nearby admissible velocity through a small optimization. The ideal result relies on reciprocal behavior, geometric/state assumptions, and applying admissible velocities consistently.

**Transfer:** retain a low-cost baseline and a clear example of responsibility sharing. Unknown other-vendor robots need not contribute the expected share, and converting a holonomic velocity into delayed differential-drive motion changes the premise. An empty admissible set also needs a defined response. Large crowd throughput does not demonstrate uncertain-rectangle clearance or braking feasibility. Test ORCA under shared-policy conditions first, then deliberately violate reciprocity so its operating boundary is visible. “No communication” is a property of the information channel, not proof of safety under arbitrary other-agent control.

### R14 — ORCA-DD: differential-drive constraints

**A.** [Authors' project](https://gamma-web.iacs.umd.edu/ORCA-DD/), [PDF](https://gamma.cs.unc.edu/ORCA-DD/ORCA-DD.pdf). All six pages examined. An effective point ahead of the robot and an enlarged disc permit reciprocal avoidance followed by a wheel-speed mapping. The construction trades geometric conservatism for kinematic compatibility. Hardware used light robots and external localization.

**Transfer:** useful baseline conversion, not a justification for feeding ordinary ORCA output into any unicycle controller. Shared reciprocal behavior, valid observations, feasible velocity sets, and actuator response remain material. The forward offset expands the represented body, potentially eliminating solutions in narrow aisles. Measure that loss against explicit oriented-footprint planning. Loaded braking and uncertain onboard tracking are not established by the small-robot demonstration. R70 provides a related acceleration-aware formulation with different low-speed limitations.

### R15 — Zhou et al., 2017: buffered Voronoi cells

**B.** [Author source](https://msl.stanford.edu/papers/zhou_fast_2017.pdf). Cell construction, QP/geometric controller, high-order caveat, and experiments examined. Shrinking Voronoi cells accounts for robot size without requiring neighbors' velocities. Separation follows if robots remain in their appropriate cells; deadlock remedies are heuristic. The paper acknowledges difficulties from higher-order dynamics and uses additional buffering/softened initialization in practice.

**Transfer:** simple position-based defensive geometry and a useful baseline under shared behavior. Not estimating neighbor velocity does not remove the need for neighbors to respect the construction. A robot already outside its cell or unable to brake inside it is not repaired by the static separation theorem. For our target, use cells as candidate constraints only when the assumptions fit; compare with motion occupancy and stopping-aware checking. Do not label the deadlock heuristics a global liveness guarantee.

### R16 — Wang et al., 2019: probabilistic buffered Voronoi cells

**B.** [Author manuscript](https://msl.stanford.edu/papers/wang_distributed_2019.pdf). Particle-based safety levels, sampled/line-search controller, and probability discussion examined. PBVC represents uncertain relative positions and selects movement with a desired cell-membership level. This extends deterministic BVC geometry rather than predicting arbitrary robot intentions.

**Transfer:** uncertainty belongs in the motion-selection interface, but estimated particle mass is conditional on the estimator/model. Independence assumptions behind products of robot safety levels need scrutiny when robots share localization or map errors. Sample-based cell approximation also needs coverage analysis. A pointwise membership probability is not a whole-mission collision bound, and maintaining cell membership still depends on compatible behavior/control. Prefer a calibrated, interpretable uncertainty baseline before using such a probability as an acceptance threshold.

### R17 — Zhu et al., 2022: B-UAVC

**A.** [Author manuscript](https://autonomousrobots.nl/assets/files/publications/22_zhu_auro.pdf), [implementation record](https://github.com/tud-amr/mrca_vc). Main construction, controller, safety assumptions, hardware, and limitations examined. Gaussian state uncertainty modifies Voronoi separating regions; examples include nonholonomic and second-order systems. The work states estimation-consistency assumptions. Its higher-order stopping buffer is heuristic, and laboratory localization/injected uncertainty differ from a noisy detection pipeline.

**Transfer:** valuable account of how estimator consistency enters distributed safety. It does not establish safety against another robot that ignores its cell. Empty/violated cells and unknown correlated error remain crucial. Do not transfer a Gaussian probability without checking whose estimate and covariance it uses. Use the paper to design uncertainty and infeasibility tests; keep shared-cell proofs in the cooperative/compatible-policy experiment class.

### R18 — EGO-Swarm

**B.** [Public manuscript](https://arxiv.org/abs/2011.04183). Trajectory optimization, alternate-side seeds, localization-drift correction, networking, and experiments examined. The drone system broadcasts trajectories, compares them at matched times, and uses depth observations to correct relative localization disagreement. Its initialization includes ordered trajectory generation and timing management.

**Transfer:** particularly useful evidence that a communicated trajectory needs agreement with physical observation. The inexpensive alternative-side initialization also motivates checking multiple local maneuver classes. It is not a communication-free unknown-robot solution, and drone spline dynamics do not directly match our wheels. Treat published submillisecond components separately from complete planning/perception/communication latency. Borrow state-consistency checks and seed diversity before considering its full optimizer.

### R19 — MADER

**B.** [Journal record](https://doi.org/10.1109/TRO.2021.3080235), [public manuscript](https://arxiv.org/abs/2010.11061). Assumptions, MINVO/spline geometry, checking protocol, and evaluation examined. Tight convex trajectory bounds support continuous-interval separation, including obstacles whose actual motion lies within stipulated prediction/discretization error bounds. Multi-agent plans are communicated and checked asynchronously.

**Transfer:** continuous occupancy bounds and explicit prediction-error assumptions are useful. The planner cannot infer a valid bound merely by fitting a trajectory; it must be supplied or calibrated. Its communication protocol and higher-order aerial motion differ from local sensing around unknown AMRs. Prefer the later RMADER when investigating bounded communication delay. For the current simulator, investigate conservative sweep bounds and retained feasible trajectories without importing the entire spline/communication stack.

### R20 — RMADER

**A.** [Public manuscript](https://arxiv.org/abs/2303.06222). Candidate/committed trajectory protocol, delay-check proof cases, and reported tests examined. A candidate is checked for a sufficiently long interval before commitment; peers retain information needed to avoid mutually inconsistent updates. The result depends on delay and delivery assumptions and an initially valid continuation. Complete communication drops are explicitly outside its guarantee.

**Transfer:** strong reference for optional future commitments, and for why a candidate is not yet an execution promise. Preserve a valid old plan while a new plan is uncommitted. Physical tracking error, time synchronization, sensing, and dropouts still need separate treatment. Collision-free generation does not imply deadlock-free operation. Use RMADER's protocol logic as a later experiment, not as evidence that the current latest-message cache establishes coordinated safety.

### R21 — DTEB

**A.** [Public manuscript](https://arxiv.org/abs/2303.11279); original user manuscript retained. All seven pages examined. DTEB incorporates arbitrary timestamped peer trajectories and footprint interactions into TEB, with priority handling and topology-inspired maneuver alternatives. Its reported multi-robot tests are simulated; their real-time factor matters when interpreting update rates.

**Transfer:** exchange shape and timing, keep encounter decisions, and evaluate more than a spatial detour. A TCP stream or latest timestamp is not a mutual commit protocol. The approximate topology signature is not an exact classifier for every curve; use it as a maneuver-generation heuristic. Soft optimization still needs external checking. In sensor-only operation, replace peer intent with uncertain observed occupancy and remove shared-priority assumptions. It is a natural refiner reference after the bounded candidate baseline is measured.

### R22 — Distributed MPC with social preferences

**C.** [Publisher abstract](https://doi.org/10.1016/j.ifacol.2025.11.587). The proposed nonholonomic controller uses responsibility-aware social value orientation and pairwise barrier-function constraints for asymmetric, semicooperative interactions. The accessible evidence describes simulation; full constraint definitions and proof assumptions were unavailable.

**Transfer assessment:** asymmetric responsibility is directly relevant to unknown traffic, but an estimated preference is not an enforceable action by the neighbor. Determine whether a noncooperative extreme still admits a feasible ego control, how responsibility weights are obtained, and whether reciprocity/compatibility is required for pairwise safety. Treat preferences as efficiency hypotheses until those questions are resolved. This is a useful research direction, with insufficient reviewed evidence to serve as the first safety layer.

### R23 — Muhammed et al., 2026: cooperative object transport

**B.** [Primary article](https://doi.org/10.1038/s41598-026-41881-w). Hardware architecture, coupled-payload model, estimation, peer filtering, and selected experiments examined. Two differential-drive TurtleBots transport a mechanically coupled payload using decentralized MPC, ROS 2, and fused estimates. Different phases use overhead vision or odometry/IMU fusion; delayed received peer state remains part of the architecture.

**Transfer:** relevant to command/estimation integration and communicating peer-state consistency. Mechanical coupling and a common transport objective sharply restrict behavior compared with independent robots. This is not evidence for arbitrary traffic avoidance or industrial payload scale. It can inform a future transport task, while our first navigation planner should not inherit the formation/payload constraints. Check the complete sensing configuration when citing the hardware evidence rather than calling the entire study onboard-perception-only.

## Safe intervals, coordination, and scalable search

### R24 — Phillips and Likhachev, 2011: SIPP

**A.** [Author PDF](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf). Main algorithm, dominance argument, transitions, and robot example examined. SIPP compresses a time-expanded search into configuration/safe-interval states, retaining earliest arrival when waiting can realize later departures. Dynamic occupancy is an input to the search; it is not established by the search itself.

**Transfer:** strong representation for an ego robot's predicted free windows or a cooperative reservation table. Every transition's swept motion matters, not just free endpoints. Basic instantaneous start/stop and waiting assumptions do not automatically hold for our loaded differential-drive system. The paper's observed/predicted obstacle motion is not a guarantee those obstacles follow the forecast. Use SIPP-IP when kinodynamic state and feasible arrival intervals matter, and keep unilateral windows distinct from honored reservations. A full search over uncertain traffic can be conservative or stale even when the underlying search is correct.

### R25 — SIPP-IP: kinodynamic constraints

**A.** [Publication](https://doi.org/10.1609/aaai.v37i10.26453), [public manuscript](https://arxiv.org/abs/2302.00776). Main interval-projection construction, theoretical argument, and experimental setup examined. Arrival intervals are propagated through kinodynamic primitives instead of discarding all but the earliest arrival at a state. An agent moving at speed may be unable to wait, invalidating basic SIPP dominance.

**Transfer:** the most directly relevant scheduling correction in the collection. Completeness/optimality are relative to the supplied graph, primitive dynamics, time representation, and known obstacle occupancy. They do not imply continuous-space global optimality or robustness to unseen traffic. A future scheduler must include velocity/direction and valid stopping states, and validate primitive sweeps against the actual footprint. Useful later for bottlenecks; first establish tracking/timing contracts and uncertainty so its occupancy inputs mean something.

### R26 — Kasaura et al., 2022: PSIPP/CTC

**B.** [Publisher](https://doi.org/10.1109/LRA.2022.3187265), [public PDF](https://ieeexplore.ieee.org/ielx7/7083369/9831196/09811344.pdf), [authors' project](https://omron-sinicx.github.io/PSIPP-CTC/). Conflict predicates, annotation/search algorithms, wheeled simulation, and limitations examined. Continuous-time vertex/edge conflicts are precomputed and reused by prioritized SIPP. The wheeled demonstration adds rotation waits and approximates acceleration/deceleration by waiting/constant-speed actions; it is not the same kinodynamic construction as SIPP-IP. The method explicitly remains nonoptimal and incomplete.

**Transfer:** precomputation can reduce repeated geometry work on a stable roadmap. Annotation cost and narrow-region ordering can still limit effectiveness. Determine which shapes, headings, traversal profiles, and uncertainty margins an annotation covers. Fixed-disc/profile tables cannot silently represent changing oriented rectangles and tracking error. Use this as an optional cooperative timing accelerator after motion contracts exist, and keep the dynamics approximation visible in any comparison.

### R27 — Andreychuk et al., 2019: continuous-time CBS

**B.** [Proceedings](https://ijcai.org/proceedings/2019/6). Methods/proof and practical collision-detection discussion examined. CCBS detects action conflicts and branches on unsafe start intervals, using constrained SIPP for low-level paths. It targets sum of costs in continuous-time roadmap coordination. Its correctness argument explicitly depends on accurate collision and maximal unsafe-interval detection.

**Transfer:** a useful small-instance oracle for cooperative timing, and a warning against substituting sampled collision detection into a theorem that assumes exact geometry. Roadmap actions, constant-speed/no-inertia examples, known joint state, and fleet compliance differ from observed unknown robots. Use it offline for simple canonical cases to measure solution quality; do not put an unbounded joint search into the control deadline without an accepted continuation. Add execution uncertainty explicitly before claiming physical separation.

### R28 — Tan et al., 2024: robust continuous-time MAPF

**C.** [Primary record](https://doi.org/10.1609/icaps.v34i1.31519). Publisher abstract and indexed primary-paper passages checked; direct full PDF requests were blocked. The paper defines delay-robust MAPF in continuous time, with exact delayed-agent collision detection and a claimed complete/optimal solver under its defined model. Full proof and implementation were not verified.

**Transfer assessment:** execution delays deserve representation in the planning abstraction. The published definition uses a total delay budget per agent. That differs from arbitrary actuator error, clock drift, prediction failure, or a robot choosing a new route. Before implementation, obtain the full paper and inspect how that budget can be consumed, what motion occurs during a delay, whether agents can stop on edges, and how occupied sets are computed. Do not convert its robust-optimality claim into a guarantee for the current sampled simulator.

### R29 — Okumura et al., 2019: PIBT

**B.** [Proceedings PDF](https://www.ijcai.org/Proceedings/2019/0076.pdf). Priority inheritance, backtracking, finite-arrival condition, and communication assumptions examined. Agents recursively coordinate their next graph moves and dynamically age priorities. The finite-arrival result requires a graph where adjacent vertices lie on suitable cycles; it is not unrestricted deadlock freedom on every warehouse map.

**Transfer:** aging and backtracking suggest tactical recovery mechanisms for compatible robots. The decentralized form still uses local/multihop communication and shared decisions. A blind single-lane passage or an unknown robot ignoring a requested displacement does not satisfy that setting. Use PIBT as an optional discrete cooperative baseline and adversarial test generator. Do not substitute a stable-ID yield rule for its recursive coordination protocol, or assume instantaneous graph moves remain safe under braking and tracking error.

### R30 — Li et al., 2021: rolling-horizon collision resolution

**B.** [Publication](https://doi.org/10.1609/aaai.v35i13.17344), [author PDF](https://jiaoyangli.me/files/2021-AAAI-2.pdf). RHCR's window/replanning relationship, multi-goal search, deadlock discussion, and warehouse setting examined. It resolves conflicts in a bounded window and executes a shorter prefix before replanning. The abstraction assumes synchronized unit-time graph actions and perfect execution; physical postprocessing is separate.

**Transfer:** useful separation of planning horizon and executable prefix. Neither a longer window nor frequent replanning alone guarantees liveness; the paper discusses horizon-related deadlock and throughput/fairness tradeoffs. A local AMR can adopt prefix validity and previous-plan reuse without adopting centralized full-fleet authority. Any physical implementation needs a feasible continuation if the next solve fails. Evaluate rolling-horizon effects instead of assuming a five-second horizon is always better than a shorter one.

### R31 — Li et al., 2022: MAPF-LNS2

**B.** [Publication](https://doi.org/10.1609/aaai.v36i9.21266), [author PDF](https://idm-lab.org/bib/abstracts/papers/aaai22a.pdf). Repair algorithm, SIPPS hard/soft obstacles, neighborhood selection, and evaluation examined. It begins with potentially colliding paths and replans subsets to reduce conflict count. This is a feasibility-repair method; intermediate plans can remain infeasible, and the algorithm does not provide a general completeness guarantee.

**Transfer:** repair only the affected part of a known fleet plan, while retaining a separately valid executable plan. A low number of remaining collisions is not acceptable for our motion acceptance gate. Large-agent/minute-scale search results are not local-controller timing evidence. For our perception-only target, other robots are not decision variables whose paths we may repair. Keep LNS2 as a later coordination baseline and a source of repair heuristics, with strict separation between search workspace and issued commands.

### R32 — Huang et al., 2022: ML-guided LNS

**B.** [Publication](https://doi.org/10.1609/aaai.v36i9.21168), [author PDF](https://idm-lab.org/bib/abstracts/papers/aaai22d.pdf). Oracle training, features, linear ranking, online algorithm, and experiments examined. The model ranks candidate agent subsets for improving an existing MAPF solution. A slow offline oracle supplies imitation targets; online ranking aims to spend search effort more effectively.

**Transfer:** an appropriate later use of learning is choosing among verified discrete search/refinement actions. It does not require learning unrestricted velocity commands. The reported policy is not a general robot-intention predictor, and map-specific generalization results do not establish transfer to new geometries. First record which candidate/refinement choices actually matter in our simulator. Preserve a nonlearned fallback and validate any repaired result before execution. Distinguish this solution-quality improvement from LNS2's initial-feasibility repair.

### R33 — Okumura, 2023: LaCAM

**B.** [Publication](https://doi.org/10.1609/aaai.v37i10.26377), [public manuscript](https://arxiv.org/abs/2211.13432). Lazy constraint trees, configuration generation, completeness argument, and selected evaluation examined. LaCAM avoids enumerating every joint successor immediately, while eventually exploring configurations needed for completeness on the discrete model. PIBT supplies promising successors but is enclosed by a broader search.

**Transfer:** heuristic guidance can be aggressive while a complete search structure preserves eventual exploration. “Complete” does not imply a feasible answer by a 100 ms deadline or execution with continuous footprints. This is a known-fleet planner, not an inference algorithm for unknown traffic. Use offline or in a later cooperative layer. Our immediate use is architectural: heuristic candidate generation and independent validity checks should remain distinct, with a retained executable fallback.

### R34 — LaCAM*: eventually optimal search

**B.** [Proceedings](https://ijcai.org/proceedings/2023/28). Cost propagation, incumbent improvement, proof conditions, swap-assisted generation, and selected experiments examined. LaCAM* adds anytime improvement and eventual optimality to configuration search. The objective must be expressible as appropriate accumulated transition costs; the paper distinguishes sum-of-loss from history-dependent flowtime.

**Transfer:** compare objectives precisely before reporting throughput or completion-time improvements. Optimality at full search termination is not an optimality guarantee for an interrupted incumbent. Guidance and swap heuristics improve empirical search but do not remove physical timing, footprint, or observation requirements. A bounded cooperative solver could provide an incumbent behind the same acceptance contract. It is unnecessary complexity for the first unknown-robot local planner.

### R35 — Kato et al., 2025: congestion mitigation path planning

**B.** [Public manuscript](https://arxiv.org/abs/2508.05253). Sparse graph formulation, multiplicative inflow cost, search/bound discussion, and experiments examined. CMPP supplies **time-independent** routes that reduce crossing/merging congestion; local collision avoidance is delegated to ORCA or PIBT. The paper reports benefits in those combined settings.

**Transfer:** route guidance and local avoidance solve different problems. Congestion-aware alternative routes may be more useful than increasing lateral maneuver complexity. For sensor-only operation, learn observed congestion without pretending all traffic is controllable. Do not label its route cost a collision constraint or a schedule. Check exact-search bound assumptions before importing them: the practical low-level forced-edge ordering uses an approximation. Measure route stability and detour cost as well as local clearance. This is a later global-routing improvement, with immediate value as a reminder to test more than one route.

### R36 — Amir et al., 2025: ReCoDe

**B.** [Primary paper](https://proceedings.mlr.press/v305/amir25a.html). Constraint parameterization, feasibility/regularity assumptions, theoretical discussion, and robot example examined. A learned policy chooses additional constraints for an optimization-based controller. Base constraints preserve the intended model-level safety boundary; learning shapes coordination. The analysis assumes nonempty convex feasible sets and regular solution mappings. Hardware includes communicating holonomic robots.

**Transfer:** attractive after a reliable acceptance/controller layer exists. It is not evidence that any learned constraint can be safely appended to a saturated differential-drive controller. Empty feasible sets, sampled execution, estimator error, and message loss still need treatment. First test small nonlearned rules for follow/yield/recover. If learning is justified, rank or constrain verified maneuvers and keep the existing hard limits independent. Do not transfer rewards or claims across robot dynamics without checking feasibility assumptions.

### R37 — Concrete multi-agent path planning, 2026

**B.** [Primary article](https://www.nature.com/articles/s44182-026-00083-2) and its local supplementary PDF. Main architecture and selected supplement sections on state/collision abstraction, search, implementation, MPC, and deployment examined. Motion-dependent spatial and temporal deviations connect discrete coordination to reference generation and control. A heterogeneous hardware study uses external tracking and ROS 2 communication.

**Transfer:** probably the collection's strongest reference for exposing an execution contract rather than treating a graph plan as directly executable. Its exact theoretical search and deployed approximations must be distinguished: supplement p.12 explicitly describes pruning/duplicate-detection choices that can compromise completeness. Uncertainty learned for a platform is not automatically calibrated for our robot. Keep the model-to-execution boundary, but do not transfer a heterogeneous mocap fleet demonstration into a sensor-only industrial-AMR result. This is later cooperative architecture evidence, not the first implementation dependency.

### R38 — Kim et al., 2026: asynchronous lifelong MAPF

**C.** [Publication](https://doi.org/10.1109/TII.2026.3681327), [authors' institutional abstract](https://pure.korea.ac.kr/en/publications/a-scalable-framework-for-lifelong-multiagent-path-finding-with-as/). The framework separates path planning from action scheduling, uses vertex precedence for asynchronous execution, and reuses residual paths through resynchronization. Large warehouse simulation results are reported; full method/proofs were unavailable.

**Transfer assessment:** explicitly scheduling asynchronous completion is more realistic than assuming all robots traverse an edge simultaneously. Precedence still needs participating robots and observable action completion. Before use, check edge occupancy, braking in progress, communication failure, cycle handling, and whether the scheduler can stall without violating collision conditions. Abstract-level scale/throughput numbers do not establish our real-time controller or safety envelope. Useful later for known-fleet execution, with the local monitor retained.

### R39 — Zheng et al.: learning-guided lifelong prioritized planning

**B.** [Primary article](https://doi.org/10.1613/jair.1.20611). Rolling-horizon prioritized planning, priority learning, discrete repair, and evaluation protocol examined. Reinforcement learning chooses total priority orders for a classical planner. Timeout repair inserts compatible moves/waits; the text explicitly distinguishes safe repair from progress, completeness, and deadlock resolution.

**Transfer:** good example of retaining classical planning while learning a discrete coordination decision. Synchronized graph safety repair is not physically instantaneous braking. Our actor policies should include priority choices different from ego's, since unknown robots do not share the learned ordering. Use as an optional cooperative throughput baseline after action execution is modeled. Its explicit separation of safety and liveness should also inform our result categories now.

### R40 — Klančar et al.: elevating priorities on roadmaps

**C.** [Publisher](https://doi.org/10.1016/j.robot.2025.105295). Publisher summary/abstract checked. Prioritized lifelong SIPP is enhanced with safe locations, occupancy-interval tests, and conflict resolution. The publication claims completeness in its infrastructure/task setting. Full proof assumptions were not available.

**Transfer assessment:** map-supported waiting/retreat locations are directly useful even without fleet communication. Their existence must be checked for footprint, visibility, reachability, and traffic occupancy. “Safe location” is not a universal property of any waypoint where ego can stop. Obtain the full paper before adopting its completeness result, and determine how safe locations are allocated and released. For our first implementation, explicit retreat bays and blocked-route outcomes are sufficient; do not add a global priority mechanism that assumes authority over unknown robots.

### R41 — Tang et al., 2026: greedy PIBT

**C.** [Publication](https://doi.org/10.1109/TASE.2026.3703194), [authors' institutional abstract](https://researchportal.hkust.edu.hk/en/publications/greedy-priority-inheritance-with-backtracking-for-multi-agent-pat/). Two variants improve PIBT move quality: backflow-based adjustment and a MILP-based single-step choice. The abstract states a centralized coordination scheme and claims preserved completeness. Full proofs were unavailable.

**Transfer assessment:** greedier one-step quality does not remove PIBT's graph/participation conditions or make the result a decentralized sensor-only policy. Centralized global data and low-overhead grid updates are not the same as local tracks and wheel-limited execution. If investigated later, compare added compute with ordinary PIBT under the same map and time budget, and separate first-arrival objective from physical mission completion. Not a priority for the current local planner.

### R42 — Local guidance for configuration-based MAPF

**B.** [Publication](https://doi.org/10.1609/aaai.v40i35.40169), [public manuscript](https://arxiv.org/abs/2510.19072). Guidance construction, reuse, cost design, and evaluation examined. Short local time-dependent paths guide LaCAM's joint search; repeated refinement and cached guidance reduce congestion and improve solution quality. Guidance itself may contain conflicts; the enclosing solver remains responsible for valid configurations.

**Transfer:** a useful separation between an informative heuristic and an executable plan. Our local performance predictor can likewise be informative without being sufficient for acceptance. Scaling results for thousands of discrete agents do not specify robot dynamics or 10 Hz execution. This is primarily a later cooperative search improvement; first measure whether tactical instability and forecast errors, rather than search guidance, dominate our failures.

### R43 — LaGAT: graph attention-guided search

**B.** [Publication](https://doi.org/10.1609/aaai.v40i35.40192), [public manuscript](https://arxiv.org/abs/2510.17382). Neural guidance, map-specific training, deadlock detection/rollback, and evaluation examined. A compact attention policy guides LaCAM; detected local stagnation causes selective fallback to nonlearned PIBT guidance. The complete search structure is distinct from the neural policy's standalone success.

**Transfer:** learning can guide a solver without replacing its validity machinery. Training effort, map adaptation, inference budget, and rollback logic are part of the system, not free improvements. An attention network receiving shared agent messages is not automatically available from uncertain local observations. Keep as a later coordination experiment. For the present planner, implement explicit commitment release/recovery and a nonlearned baseline before learning when to invoke them.

### R44 — Lifelong LaCAM with local guidance

**B.** [Proceedings](https://doi.org/10.1609/socs.v19i1.43067), [public manuscript](https://arxiv.org/abs/2605.16855). Receding windows, guidance reuse, search truncation, and selected evaluation examined. LLLG replans each graph timestep and executes the first joint move. Guidance and/or previous joint plans warm-start subsequent searches; finite-window LaCAM can be refined with anytime techniques. Reported throughput/runtime improvements concern discrete lifelong tasks, with seconds-scale per-step evaluation budgets.

**Transfer:** warm starts can preserve useful traffic information, but a previously computed window is not necessarily physically executable after tracking error or changed observations. Truncating search at a finite horizon changes the problem from full-horizon completeness/optimality. Measure progress and deadline behavior explicitly rather than inferring them from the LaCAM name. This is an incremental known-fleet coordination option; it does not remove unknown-robot uncertainty or justify delaying the perception-only baseline.

### R45 — Kumar et al., 2026: SIPP and learning-guided repair

**B.** [Proceedings](https://ijcai.org/proceedings/2026/842). Formulation, pseudocode, goal reservation, experiment assumptions, and repair comparison examined. Prioritized SIPP with bounded goal reservation is followed by XGBoost-guided LNS. The experiments use holonomic, instantaneous start/stop grid motion without execution uncertainty. Goal release assumes robots leave the active navigation grid after completion.

**Transfer:** goal occupancy semantics are critical. Releasing a cell after a fixed buffer is unsafe if our physical AMR remains parked there. This is a two-stage classical/learned search reference, not evidence of real-time loaded-robot deployment. Its runtime claims also include harder cases with much longer times; do not quote only typical milliseconds. Keep valid incumbent/fallback plans separate from partial repair results. Use only in the optional cooperative track after the departure/goal model is specified.

### R46 — Huang et al., 2026: SMART

**B, public preprint; journal version may differ.** [Publisher](https://doi.org/10.1016/j.trc.2026.105793), [public manuscript](https://arxiv.org/abs/2509.15737). Formulation, priority search, corridor/trust-region SQP, and selected simulation/hardware results examined. Ackermann vehicles share state and coordinated plans; learned priority guidance initializes search. Independent convex subproblems are embedded in nonlinear trajectory refinement. The hardware is a connected-vehicle platform. Large simulation solve times and small hardware planning examples span different regimes.

**Transfer:** supports separating maneuver order from continuous refinement, but not sensor-only operation around arbitrary robots. Search completeness/optimality claims require the actual discrete representation and search assumptions; they were not independently proved here. Small SQP iterate changes alone do not establish original nonlinear feasibility or between-sample separation. Keep an explicit final acceptance check and measure full latency. Preprint/journal author lists and versions differ; do not silently combine their evidence.

### R47 — Guo et al., 2026: risk index and dynamic priorities

**C.** [Publisher](https://doi.org/10.1016/j.robot.2026.105592). The accessible publisher material describes staged cooperative planning, improved RRT*/APF global search, and collision-risk/dynamic-priority mechanisms. Full definitions, proofs, and experiments were unavailable.

**Transfer assessment:** a risk index can rank conflicts or trigger more planning effort. It is not automatically a calibrated collision probability or an enforceable avoidance constraint. Check normalization, threshold sensitivity, compatibility of priority updates, and treatment of stationary/occluded robots before using it. Our current need is independent actor behavior and a transparent acceptance contract; a more elaborate risk score would not resolve missing sensing or absent right-of-way agreement. Keep this as an access-limited source rather than asserting performance superiority.

## Prediction, uncertainty, and evaluation

### R48 — Salzmann et al., 2020: Trajectron++

**B.** [Public manuscript](https://arxiv.org/abs/2001.03093). Scene graph, CVAE architecture, dynamics integration, ego-plan conditioning, and evaluation setup examined. It models heterogeneous agents, uses map/history context, and integrates predicted controls through appropriate dynamics. Pedestrian and driving datasets demonstrate forecasting rather than industrial robot navigation.

**Transfer:** dynamically coherent, multimodal prediction is more useful than independent position samples that violate robot motion constraints. Ego-plan conditioning can support maneuver comparison, but does not establish that the other agent will react as predicted. Dataset classes/maps and tracking quality differ from our workspace. Compare against simple models with occupancy calibration and closed-loop outcomes, not just best-of-K error. A retained nonreactive mode and detection/freshness handling are still required. This is a later learned-prediction baseline, not a prerequisite for the first planner.

### R49 — Sun et al., 2022: M2I

**B.** [Primary paper](https://openaccess.thecvf.com/content/CVPR2022/html/Sun_M2I_From_Factored_Marginal_Trajectory_Prediction_to_Interactive_Prediction_CVPR_2022_paper.html). Relation labels, marginal/conditional factorization, sample selection, and Waymo evaluation examined. M2I identifies influencer/reactor relationships, forecasts the influencer, and conditions reactor forecasts on those samples. Its main formulation is pairwise; broader chaining requires assumptions about influence structure.

**Transfer:** independently plausible forecasts may form an impossible joint encounter. Preserve joint hypotheses and test consistency. “Influencer” is a learned prediction relation, not an assigned priority that another robot accepts. Teacher-forced training and inference with predicted influencer motion also differ. Assess compounding model error, relation switches, and sample-budget cost on recorded workspace traffic. Use the idea of coherent joint modes before adopting a large driving-dataset model.

### R50 — Alahi et al., 2016: Social LSTM

**B.** [Primary PDF](https://www.cv-foundation.org/openaccess/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf). Social pooling, recurrent prediction, training, and ETH/UCY evaluation examined. Neighbor hidden states are pooled spatially to forecast interacting pedestrian trajectories. Inputs are preprocessed tracked coordinates; the work is not a detection/data-association pipeline.

**Transfer:** useful historical baseline for interaction encoding. It does not impose our differential-drive dynamics, unknown robot task logic, or physical clearance. Dataset prediction error does not imply safe navigation, and the controller must choose an action before knowing which prediction matches the future. Keep as an optional predictor baseline if local trajectory data become available. First measure whether a simpler motion/history model misses enough useful behavior to justify training and inference complexity.

### R51 — Gupta et al., 2018: Social GAN

**B.** [Primary PDF](https://openaccess.thecvf.com/content_cvpr_2018/papers/Gupta_Social_GAN_Socially_CVPR_2018_paper.pdf). Generator/pooling/discriminator, variety loss, and benchmark protocol examined. Stochastic trajectory generation represents multiple plausible pedestrian futures, with best-of-sample training/evaluation components. A discriminator's notion of socially plausible motion is not a geometric collision certificate.

**Transfer:** multiple hypotheses can expose alternatives a single mean misses. But a low minimum error after drawing many samples does not tell our controller which sample is true, nor guarantee coverage of a rare unsafe mode. Hold sample counts and compute fixed, evaluate calibration/occupancy miss rate, and add feasible robot dynamics. Treat samples as performance predictions until validated for the acceptance use case. This remains a historical prediction baseline rather than a preferred first deployment model.

### R52 — Yang et al., 2024: IA-LSTM

**B.** [Journal record](https://doi.org/10.1109/TCYB.2024.3359237), [public manuscript](https://arxiv.org/abs/2311.15193). Correntropy-based personal-space representation, interaction module, recurrent architecture, and dataset evaluation examined. It weights social interactions and forecasts bivariate position distributions on pedestrian trajectory benchmarks. The manuscript's template header is not its publication date.

**Transfer:** interaction weighting can be more selective than treating every neighbor equally. Human personal-space modeling does not directly describe vendor robot stop/yield logic. Robustness of an intermediate similarity measure to outliers does not establish robustness to missed detections, ID switches, or unseen robot policies. Evaluate those faults explicitly and compare with simple predictors. Without local evidence of a benefit, this adds model complexity before the current estimator/prediction contracts are resolved.

### R53 — Samavi et al.: SICNav

**A.** [Public manuscript and revisions](https://arxiv.org/abs/2310.10982). Main formulation, KKT assumptions, numerical implementation, simulations, and hardware sections examined. A bilevel planner models agents through ORCA-like optimization and embeds optimality conditions into ego planning. Constraint regularity and multiplier degeneracy matter; the implementation also introduces relaxations for feasibility. Original hardware uses external tracking.

**Transfer:** strong evidence for testing candidate-conditioned responses, with a transparent model of what reaction is expected. Do not transfer a model-level constraint into a guarantee against arbitrary noncooperation. Its reported goal-success rate coexists with nonzero collision-frequency metrics. Solver scaling with nearby interactive agents and fallback behavior need measurement. Use a small selected interaction set for efficiency only if excluded agents still appear in acceptance occupancy. A nonreactive branch and tracking uncertainty are necessary for our target.

### R54 — Autoware map-based prediction documentation

**D.** [Versioned documentation](https://autowarefoundation.github.io/autoware_universe/refs-tags-v1.0/perception/map_based_prediction/), retrieved 2026-11-01. The module associates objects with lanelets, predicts maneuvers, and produces paths/probabilities using map/history context. Documented limitations include constant-velocity assumptions and straight-line fallback in several cases; dynamics may be inadequately represented.

**Transfer:** good concrete reference for map-compatible hypotheses, association gates, and explicit known limits. A warehouse route graph can constrain plausible turns without requiring other robots to publish goals. Incorrect map association must retain alternatives rather than force all occupancy onto one branch. Lanelet probability is a forecast weight, not a reservation. Do not treat this tagged documentation as a statement about every current Autoware release. Inspect APIs and tests only if integration becomes an actual requirement.

### R55 — Intent-aware spatiotemporal risk fields, 2026

**B.** [Primary article](https://doi.org/10.1007/s44443-026-00997-9). Problem split, predictor, risk fields, coordination, and ablation/evaluation sections examined. The framework distinguishes external noncommunicable obstacles from communicating peers, combining prediction/risk-aware search with peer conflict resolution. It reports grid-based results and module tradeoffs; risk fields can sometimes induce counterproductive detours.

**Transfer:** this information-channel split is useful. Our initial experiment should use only the external-obstacle side, and must not retain peer path-sharing benefits in a sensor-only result. A risk-weighted path is not a hard occupancy-separation constraint, especially when nonzero collisions remain an evaluation outcome. Check predictor calibration, risk thresholds, and equal-prediction baselines. The reported runtime is not proof of a full sensing-to-actuation deadline or continuous footprint feasibility. Use as a mixed-traffic architecture reference, not a ready safety layer.

### R56 — Meng et al., 2026: SceneAware-Att

**C.** [Publisher abstract](https://doi.org/10.1049/itr2.70252). Scene/history attention and risk-aware decoding are evaluated with pedestrian displacement error, collision/risk-exposure measures, and joint-scene metrics. The full formulation and detailed protocol were unavailable.

**Transfer assessment:** prediction quality should include scene consistency and risk exposure. However, predictions avoiding one another do not prove their distribution covers observed traffic or ego's executed trajectory. “Risk-aware” can describe a training/evaluation preference rather than a calibrated probability bound. Before use, obtain the full paper and inspect risk-field construction, sample count, calibration, train/test leakage controls, and latency. This is an access-limited predictor candidate, with no reason to prefer it over simple local models before collecting workspace data.

### R57 — Pan et al.: mixed crowd navigation survey

**C.** [Publisher record/abstract](https://doi.org/10.1146/annurev-control-032024-023929). The review spans perception, interaction, planning, and control. Advance publication in 2025 and volume publication in 2026 explain the catalog date. A complete full-text review was not achieved here.

**Transfer assessment:** valuable as a broader literature map, but secondary summaries should not substitute for checking the original theorem or experimental setup of a chosen algorithm. Human mixed crowds and robot-only unknown fleets overlap in partial observability and interaction, while differing in drive constraints and behavior. Use the survey to identify missing perception/evaluation questions. No numerical result or deployment guarantee in this report is inferred from the survey abstract.

### R58 — Firoozi et al.: OA-MPC

**A.** [Public manuscript/version history](https://arxiv.org/abs/2211.09156). Both the older linked PDF and later 13-page revision were examined, including the safety formulation; Eq.2e was rendered. Occluded and observed-agent sets are propagated under motion bounds and updated with measurements. A stopping terminal construction supports its stated feasibility argument.

**Material limit:** the collision condition is relaxed at a stationary ego state. Its “safe” formulation can count a stopped robot being hit as safe; it does not establish indefinitely collision-free terminal occupancy. Measurement-set containment, reachable-motion bounds, initial feasibility, and execution/model assumptions also matter.

**Transfer:** use hidden occupancy and verified continuation ideas, with a precisely chosen promise. Avoid blindly growing occupancy until navigation freezes; incorporate reliable visible-empty evidence. Our acceptance layer needs its own geometry, loaded braking, latency, and terminal-occupancy reasoning. See [the full correction](03_gpt_claims.md#oa-mpc-the-material-safety-correction).

### R59 — Haraldsen et al., 2024: VO theory for nonholonomic vehicles

**B.** [Publication](https://doi.org/10.1109/TCST.2024.3375023). Obstacle model, avoidance construction, invariant-set conditions, and vessel evidence examined. The analysis explicitly bounds obstacle speed/turning/acceleration and derives sufficient conditions for evasive turning. Conditions include speed and initial-separation requirements; the marine extension has additional dynamics assumptions.

**Transfer:** unusually useful for seeing exactly what a nonreciprocal avoidance theorem needs. It is not a general theorem for stopping, reversing, cluttered multi-robot aisles, or every combination of obstacles. Our AMR may lack the speed advantage and open maneuvering space required by the construction. Use the stated conditions to define adversarial tests and identify operating-envelope violations. Do not cite the vessel demonstration as evidence that the same law is suitable for loaded differential-drive robots near walls.

### R60 — Zheng et al.: occlusion-aware contingency planning

**B.** [Publication](https://doi.org/10.1109/TCYB.2025.3632366), [public manuscript](https://arxiv.org/abs/2502.06359). Reachability/risk construction, shared-prefix exploration/fallback trajectories, optimization, and selected results examined. Phantom traffic informs speed limits; both efficient and fallback branches share initial motion so a future choice remains possible.

**Transfer:** a concrete alternative to merely attaching a brake after selecting an aggressive path. Shared prefixes can preserve options as visibility improves. Road structure, phantom-agent bounds, model assumptions, finite iterations, and the feasibility of both branches still need verification in our workspace. A reported driving solve time does not establish our latency. Consider a small contingency candidate family after the basic continuation checker exists; retain simpler bounded behavior when a second branch cannot be validated.

### R61 — Ye et al., 2026: SCU-T-MPC

**C.** [Publisher abstract](https://doi.org/10.1049/csy2.70051). The proposed method combines uncertain/multimodal topological guidance, parallel risk-aware optimization, chance constraints, and learned social preference. The abstract claims robustness and simulation benefits, but full proof and implementation were unavailable.

**Transfer assessment:** multiple maneuver classes can prevent a local optimizer from committing to the wrong side. The unresolved questions are how uncertain classes are represented, how total risk is allocated, whether constraints are softened, and what occurs if every class is infeasible or late. Parallel computation also needs an explicit resource/deadline bound. This is a later refiner direction; our first bounded generator can test class diversity without assuming its unreviewed theorem applies.

### R62 — Layered legged-robot coordination with reachable-set CBFs

**B.** [Publication](https://doi.org/10.1109/ICRA57147.2024.10610589), [public manuscript](https://arxiv.org/abs/2312.08689). Reachable collision regions, high-order CBF conditions, layered model, and hardware description examined. A reduced-order safety planner, rigid-body force planner, and whole-body controller distribute responsibilities across timescales.

**Transfer:** safety conditions should match the model/control authority of the layer enforcing them. A continuous-time barrier theorem needs feasible controls, the right relative degree, valid state/obstacle bounds, and execution fidelity. Leg dynamics and kilohertz controller layers are not our wheel-driven plant or sensing rate. Borrow layered acceptance/monitoring concepts, not numerical gains or a theorem detached from those assumptions. This paper does not remove the need for a braking/footprint contract in our simulator.

### R63 — Social navigation through constrained optimization

**B, public preprint; journal expansion not fully reviewed.** [Journal page](https://www.sciencedirect.com/science/article/pii/S0921889024002148), [available preprint](https://arxiv.org/abs/2305.02859). The preprint compares Euclidean, Mahalanobis, ellipse, and adaptive uncertainty constraints/costs for unicycle MPC. Its formulation and simulation tables were examined. The expanded journal title includes real-world study; those additional results were not verified from the shorter local preprint.

**Transfer:** uncertainty in an objective and uncertainty in a hard condition are different design choices. Adaptive relaxations can improve progress while changing the implied risk. Tables explicitly include collisions/timeouts, so this is comparative evidence rather than proof that the chosen metric prevents collision. Recheck footprint inflation and uncertainty calibration rather than assuming a Mahalanobis distance directly represents body separation. Useful baseline for uncertainty ablations, with version boundaries preserved.

### R64 — Arena 4.0

**B.** [Public manuscript](https://arxiv.org/abs/2409.12471). World generation, ROS 2 migration, human simulation, and validation approach examined. The framework combines environment/scenario generation, semantically annotated assets, multiple simulation backends, and benchmarking tooling. Its user/world-generation studies validate platform capabilities, not our navigation algorithm.

**Transfer:** useful later for ROS integration, sensing, and varied maps. Generated diversity is not automatically representative industrial traffic, and shared human-model assumptions can still favor a predictor. Preserve our fast deterministic simulator for falsifiable geometry/timing cases, then use a richer simulator to test sensing and integration. Pin dependencies and report real-time factor. Do not let graphical realism stand in for correct wheel/braking dynamics or realistic independent robot behavior.

### R65 — Demonstrating Arena 5.0

**B.** [Public manuscript](https://arxiv.org/abs/2610.11220). Architecture, world/scenario generation, sensor and benchmark modules examined. The full title is “Demonstrating Arena 5.0: A Photorealistic ROS2 Simulation Framework for Developing and Benchmarking Social Navigation.” It extends the platform/tooling around richer simulation and data collection.

**Transfer:** consider as an integration test environment, not as evidence that our planner is deployable. A recent arXiv upload and a feature's first implementation date are different facts; the manuscript also discusses earlier platform activity. Verify the actual supported ROS/simulator versions and reproducible installation before making it a dependency. Sensor variety is useful for testing the future track interface, while our primary challenge remains unknown robot behavior and correct timed execution. Keep photorealistic benchmarking secondary to the deterministic failure suite.

### R66 — Pérez-Higueras et al.: HuNavSim

**B.** [Public manuscript](https://arxiv.org/abs/2305.01303). Behavior-tree/SFM architecture, simulator wrapper, metrics, and example comparisons examined. Configurable human behaviors and an extensible evaluator support passing/crossing scenarios. Metrics distinguish goal arrival, proximity, motion, and collision attribution.

**Transfer:** adopt the breadth of outcome accounting and controlled behavior variation. Attribution such as person-on-robot versus robot-on-person is informative, but our evaluator should count every physical contact separately from any passive-safety definition. SFM/behavior trees are models of humans, not automatically plausible policies for forklifts or AMRs. Add nonyielding and delayed-response robot actors alongside any HuNavSim tests. Do not evaluate only the social quantity optimized by a controller.

### R67 — Alyassi et al., 2025: review and learning benchmark

**B.** [Primary review](https://doi.org/10.3389/frobt.2025.1658643). Simulator/evaluation taxonomy, benchmark setup, information access, and discussion examined; not every surveyed paper was independently read. The benchmark combines maps, sensor simulation, crowd models, and multiple policy families. Some baselines have privileged map/position access, which is explicitly described.

**Transfer:** report observation privileges, actor models, and evaluation scope alongside a ranking. A visual-sensor policy and an exact-position policy are not equal-information comparisons. The review's concern about realistic human reactions reinforces cross-model testing. Our robot-only benchmark must additionally cover drive constraints, independent vendor behavior, track faults, and planning deadlines. Useful evaluation checklist; its comparative scores are not a ranking of candidates for our workspace.

### R68 — Karwowski et al., 2024: requirements/planning/evaluation review

**B.** [Primary full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086376/). Requirements discussion and selected perception, evaluation, and future-work sections examined. The review connects human preferences to algorithmic objectives and evaluation methods. It also exposes context dependence and disagreement among user studies.

**Transfer:** begin with intended behavior and measure whether users/operations find it acceptable. Universal “keep right” or fixed personal-space costs should not be presented as laws derived for all environments. Social comfort, physical safety, predictability, throughput, and intervention burden need distinct measures. For the initial robot workspace, derive priorities from the actual operating context and traffic rules; add human-specific studies when people are in scope. This survey supplies a requirements framework, not a control safety proof.

### R69 — Nav2 MPPI, Jazzy documentation

**D.** [Official versioned documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/), retrieved 2026-11-01. The controller rolls out motion models, scores candidates through critics, supports drive constraints and obstacle/path costs, and exposes sample/horizon/budget choices. It is already predictive with respect to ego motion.

**Transfer:** use a tuned implementation as a deployment baseline, with the same map and observations. A conventional geometric path and obstacle costmap do not by themselves encode our externally planned arrival windows and time-indexed uncertain other-agent occupancy. Extending critics is an integration point, not automatically a hard-constraint or timed-execution solution. Validate any retiming against the planner's contract. Current documentation defaults and performance examples are not settings or measured results for our simulator/hardware.

### R70 — ORCA-AVO: smooth differential-drive coordination

**A; additional work from the download list.** [Author PDF](https://gamma.cs.unc.edu/ORCA-AVO-DD/ORCA-AVO-DD.pdf). Main body of the 13-page chapter examined. It relates effective-center/disc inflation to acceleration-velocity-obstacle coordination, including a proportional response model and mapping to robot motion. The acceleration-aware variant has low/zero-speed limitations and assumptions shared by agents.

**Transfer:** an important corrective to treating ORCA-DD as the end of dynamics handling. Longitudinal acceleration and lateral acceleration `v*omega` differ from a car-like minimum curvature constraint. Loaded actuator behavior and a nonparticipating opponent do not inherit the reciprocal guarantee. Compare geometry conservatism, near-stop behavior, and controller tracking in a shared-policy baseline before any deployment consideration. Its reported small-robot speedup is scenario-specific, not a general runtime or throughput factor for this project.

## Selection for this project

This is an engineering ordering, not a common-benchmark ranking:

| Role | First sources to use | Decision |
| --- | --- | --- |
| Bounded geometry and timing | R01, R02, R03 | Candidate generation first; optional seeded refiner later |
| Unknown/occluded traffic and continuation | R58, R60, with R59's explicit operating conditions | Adopt occupancy/contingency concepts; rederive acceptance for our dynamics and promise |
| Inspectable uncertainty | R05, R63, R54 | Start simple; verify geometry and calibration |
| Interaction experiment | R10, R53; R09 after data/compute justify it | Efficiency forecasts with nonreactive alternatives retained |
| Low-cost compatible-policy baselines | R13–R17, R70 | Test assumptions explicitly; no arbitrary-traffic guarantee |
| Timed cooperative scheduling | R24–R30, R37–R40 | Optional future track, after measured execution contracts |
| Communication commitments | R20, contrasted with R21 | Candidate/commit and delay/loss are separate concerns |
| Learned search/coordination | R32, R36, R39, R43 | Only after identifying a measured decision bottleneck |
| Evaluation/integration | R64–R69 | Richer simulation and outcome accounting alongside fast deterministic tests |

The central result of the review is the [system boundary and design](02_system_design.md), followed by the [validation gates](04_validation_roadmap.md). Combining named algorithms does not compose their guarantees: observation, geometry, dynamics, timing, other-agent behavior, solver feasibility, and fallback assumptions all have to agree.
