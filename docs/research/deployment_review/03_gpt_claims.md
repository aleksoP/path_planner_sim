# Audit of the supplied GPT conversation

Input: the attached “Check Trepella et al 2026…” conversation, reviewed through its final perception-only revision. This is an audit of that text against primary sources and this checkout. The conversation itself is not experimental evidence. Paper IDs and evidence levels are in [the source review](01_paper_review.md).

## The requirement changed during the conversation

The early architecture emphasizes a cooperative fleet, shared priorities, reservations, and communicated trajectories. The final requirement is an AMR detecting unfamiliar robots with onboard sensors. Those are different information and authority models. The final requirement governs this proposal.

Keep the useful hierarchy: global route, tactical encounter decision, timed local motion, monitored execution. Move commitments, fleet reservations, PIBT, LaCAM, RMADER, and coordinated SIPP to an optional cooperative extension. A sensor-only planner may predict a passage window, but cannot reserve it against an unknown robot. A globally sorted simulator ID does not establish right of way between vendors.

The later GPT text is substantially more careful than the early suggestions: it already distinguishes proposed work from demonstrated performance, recognizes MPPI as predictive, and rejects direct transfer of industrial safety claims. Preserve those qualifications.

## Claim-by-claim findings

| Claim or implied transfer | Finding | Consequence |
| --- | --- | --- |
| Trepella et al. 2026 exists and has public Nav2 code | Verified: arXiv `2607.10374`, submitted 11 July 2026; public `PIC4SeR/sfm-nmpc` | It is a relevant implementation reference, not an invented future citation |
| Trepella predicts human reactions to the candidate robot trajectory | Supported by its coupled SFM rollout inside the residual evaluation | Compare candidate-conditioned prediction with independent forecasts |
| Its result establishes superiority over MPPI for our AMR | Unsupported transfer; reported comparison is a particular social-costmap MPPI setup in simulated pedestrian scenarios | Reproduce equal-information, equal-budget comparisons on our maps and actor policies |
| A 20 Hz controller in the paper proves a loaded robot can run it at 20 Hz | Unsupported; processor, solver configuration, scene, and dynamics differ | Measure complete sensing-to-command latency and tails on the target compute |
| SFM acceleration/smoothness costs enforce loaded braking limits | Incorrect as a general inference; soft costs are not hard acceleration constraints | Put calibrated actuator constraints in rollout and acceptance |
| OA-MPC guarantees an unseen robot cannot collide with a stopped ego | Incorrect reading of the guarantee | Its collision condition is relaxed when ego is stationary; see the detailed correction below |
| No inter-robot communication means no assumptions about others | Incorrect for ORCA, ORCA-DD, and buffered Voronoi approaches | Their shared avoidance/cell behavior differs from arbitrary nonparticipating robots |
| SICNav guarantees collision-free real-world motion regardless of pedestrian policy | Too strong | Distinguish model-level constraints, softened implementation, prediction mismatch, and execution |
| SICNav-Diffusion's 99.03% means collision-free task success | Incorrect metric substitution | It reports autonomous operating time in the outdoor study |
| A Gaussian “95% ellipse” establishes 95% collision-free navigation | Unsupported | Check calibration, footprint geometry, optimized confidence slack, continuous time, and total risk |
| SIPP handles acceleration by adding a braking penalty | Incorrect | Earliest-arrival dominance can fail; SIPP-IP propagates feasible arrival intervals with kinodynamic state |
| SIPP planning against predicted traffic creates a reservation | Incorrect | Only participating agents can honor a reservation |
| DTEB broadcasts are equivalent to committed executable trajectories | Incorrect | Latest received intent is useful, but does not provide RMADER's candidate/commit delay-check protocol |
| RMADER tolerates arbitrary communication failure | Incorrect | Its delay-bound argument does not handle complete communication drops |
| LF/Concrete's hardware validates unknown-robot onboard perception | Unsupported transfer | Their coordinated, externally tracked deployments solve a different problem |
| D4orm is a pretrained diffusion policy | Incorrect | It performs learning-free sampling/denoising optimization; its finite collision penalty still needs acceptance checking |
| CSTP is a single established algorithm with an inherited theorem | Unsupported | In this conversation it names a proposed combination of mechanisms; specify and test each contract |
| Standard Theta* + MPPI has no prediction | Incorrect, and correctly corrected later in the conversation | MPPI rolls out ego dynamics; the relevant extension is explicit time-indexed other-agent occupancy and execution contracts |
| A normal path message enforces a timed crossing | Unsupported | A custom timed interface or appropriate controller extension is required |
| A quintic offset is automatically smooth and executable | Too strong | Boundary matching, reference regularity, temporal derivatives, wheel limits, and curvature all matter |
| A successful L-shaped run proves robust navigation | Unsupported; the conversation itself notes infeasible updates | Preserve it as one regression case and report failed acceptance separately from eventual arrival |

## OA-MPC: the material safety correction

Read the latest public manuscript, not just the older linked conference PDF. Its Eq.2e on PDF p.4 multiplies the collision constraint by the magnitude of the robot's state increment. When that increment is zero, the product condition permits an obstacle's occupancy to overlap the stopped robot. The text explicitly treats a stopped vehicle being hit as safe for this formulation. The recursive feasibility construction relies on that stopping relaxation [R58].

Therefore, use this paper for occlusion occupancy propagation and stopping-aware planning, but label its guarantee precisely. It does **not** prove a stopped robot's footprint remains collision-free forever against unseen moving agents. A terminal stop is a useful fallback objective; a collision-free invariant terminal region is a stronger property that needs its own assumptions. Our proposed acceptance checks must state whether they establish collision-free continuation, a passive stopping condition, or only a best available protective response.

This does not invalidate the paper's result. It changes the promise we may responsibly transfer to this application. See [the stopping calculation](05_analytical_checks.md#stopping-with-an-unknown-opponent).

## Trepella: useful interaction model, limited transfer

The reported aggregate success is 98% versus 94% for its MPPI/social-costmap baseline, while mean time to goal is 18.52 versus 16.90 s. Those numbers support a tradeoff in the tested scenarios, not a general speed advantage. The study is simulation-based; physical validation and perception uncertainty remain future work in the manuscript [R10, Table I and conclusion].

The implementation repository was inspected at commit `13203ca6bed824f4dd5538d1d665c7014fb5891e`. Its [README](https://github.com/PIC4SeR/sfm-nmpc/tree/13203ca6bed824f4dd5538d1d665c7014fb5891e) documents ROS 2 Humble. It was not built here and no Jazzy compatibility claim was verified. The [license file](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/LICENSE) is GPLv3. Record this before deciding whether to incorporate code.

Two concrete source observations matter for experimentation. [Optimizer::people_to_status](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/sfm_nmpc/src/optimizer.cpp) pads/truncates the optimization state to three agents. The inspected [PeopleInterface](https://github.com/PIC4SeR/sfm-nmpc/blob/13203ca6bed824f4dd5538d1d665c7014fb5891e/sfm_nmpc/src/people_interface.cpp) stores the last message without an age check in that class. These observations do not establish how every caller handles freshness, or that all future repository versions have the same behavior. They are reasons to inspect selection and freshness end to end before importing the controller.

Our engineering inference: SFM-controlled simulated pedestrians can favor an SFM-based predictor. A social-work objective evaluated with social work also rewards agreement with the chosen model. Both are legitimate experiments, but add independent shape clearance, interventions, delay, and tests with nonreactive robots and different reactive policies.

## Bibliographic and metric corrections

- Several 2026 records are real as of this review date, 1 November 2026. Do not reject them merely because the DOI contains another year or an arXiv identifier is recent. Publication online, conference acceptance, and issue publication can differ.
- R11 appeared online in November 2025 and in a 2026 journal volume. R57 was a 2025 advance review and a 2026 volume publication. OA-MPC's arXiv revision and journal year also differ.
- The original five files include cover pages; use **PDF page** locators, not an assumed printed-page offset.
- Arena 5's supplied short title is not its complete title. The downloaded manuscript is “Demonstrating Arena 5.0: A Photorealistic ROS2 Simulation Framework for Developing and Benchmarking Social Navigation” [R65]. Its 2026 arXiv upload does not by itself establish when every feature first appeared.
- Concrete's 40-robot demonstration is heterogeneous; it is not 40 industrial differential-drive AMRs [R37].
- Xin's sum of individual completion times is flowtime, distinct from the maximum completion time/makespan [R04].
- SICNav's goal-success metric can coexist with nonzero collision frequency. Goal arrival alone is not our acceptance criterion [R53].
- Report planning time, control frequency, simulation real-time factor, and end-to-end wall-clock latency separately. DTEB's reported simulation slowdown is particularly relevant [R21].

## What this checkout already has

The current implementation already has timed trajectories, limited-visibility observations, dynamic predictions, footprint checking, acceleration-bounded rollout, and encounter memory. The next work should strengthen their semantics and coverage. Reimplementing them under a new acronym would not establish an improvement.

Current omissions include physically coherent turning predictions, uncertain tracks and missed detections, heterogeneous actors, checked stop continuations, enforceable plan expiry, and robust recovery from blocked commitments. The fuller file-level map is in [the system design](02_system_design.md#9-immediate-mapping-to-this-repository).

The attached conversation is treated here as research input. No runtime GPT feature has been added. If conversational robot commands are later desired, they should produce validated mission requests and explanations through a separate supervisory interface; the motion acceptance and emergency response must remain available independently of that service.
