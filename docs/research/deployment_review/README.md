# Research review: robust navigation among unknown moving robots

**Recommendation:** develop a perception-driven, timed local planner with explicit trajectory acceptance and monitored execution. Use simple calibrated motion predictions first, then evaluate interaction-conditioned prediction for efficiency. Keep fleet coordination optional. The current simulator is the baseline for testing these choices; no reviewed paper makes the proposed combination deployable by itself.

Review date: **1 November 2026**. This review covers the reference directory, the attached GPT conversation, and the current planning/simulation code. Keep the current **0.8 × 0.6 m footprint and 1.5 m/s speed limit fixed**, as requested; make hardware parameters tunable later.

## Read the results

| Document | What it answers |
| --- | --- |
| [Paper-by-paper review](01_paper_review.md) | What each work contributes, what assumptions it needs, what evidence it provides, and whether to adopt it |
| [Proposed system design](02_system_design.md) | Perception, prediction, candidate generation, acceptance, timing, recovery, and mapping to current source files |
| [Audit of the GPT conversation](03_gpt_claims.md) | Verified references, corrected guarantee/metric interpretations, and the final perception-only requirement |
| [Implementation and validation roadmap](04_validation_roadmap.md) | Ordered implementation slices, ablations, fault injection, benchmarks, and physical deployment gates |
| [Reproducible analytical checks](05_analytical_checks.md) | Turning prediction, stopping gaps, ellipse geometry, right-normal Frenet formulas, jerk, and risk accounting |
| [Source coverage and local PDFs](06_source_coverage.md) | Clickable retained files and explicit review depth for every source |
| [Follow-up GPT assessment](07_followup_assessment.md) | Decisions adopted, corrections to experimental sequencing/replay, and newly checked deployment references |
| [Machine-readable source ledger](sources.json) | Source URLs, versions, download attempts, PDF pages/hashes, and access limits |

Start with this page and the system design. Use the individual paper entries when evaluating a specific mechanism, and the roadmap when choosing the next implementation task.

## Coverage and limits

The five original PDFs, 23 linked downloads, and 69 catalog rows resolve to **70 distinct works: 68 papers and two software documentation records**. ORCA-AVO is an additional distinct paper present only in the download list.

| Review depth | Works |
| --- | ---: |
| Detailed main-text method/assumption/evaluation review | 19 |
| Selected full-text method/evaluation review | 38 |
| Abstract/publisher/project-page review only | 11 |
| Versioned software documentation | 2 |

Thus **57 papers were reviewed from full text**, with varying depth; **11 remain access-limited and are not represented as completed in-depth reviews**. The latter are R06, R12, R22, R28, R38, R40, R41, R47, R56, R57, and R61. Their entries identify what remains unverified. Public author/preprint alternatives and open-access metadata were checked; publisher restrictions, bot checks, and unavailable manuscripts limited access. Available preprints, particularly R63, do not necessarily cover every part of their expanded journal versions.

All 23 requested download-list PDFs were retrieved. Additional public manuscripts and a supplement are retained under `docs/references/03_additional_verified_pdfs`. Only duplicate extra copies created during this review were removed; the original collection and download-list paths remain. The ledger identifies reviewed files by hash. Important equations/tables were also rendered for inspection; derivations and selected claims were checked analytically.

No external planner was built or benchmarked, and no production planner behavior was changed. Reported paper results are the authors' results. Analytical examples are our calculations. The proposed design and numerical timing targets are engineering proposals. The previously completed repository build/test checks establish the existing baseline only.

## The decisions supported by the review

**1. Unknown robots must remain independent actors.** ORCA, buffered Voronoi methods, and coordinated MAPF are useful references, but “no communication” does not mean “safe against arbitrary behavior.” Their guarantees have participation, model, geometry, and execution assumptions. Our primary benchmark must include nonyielding, delayed, turning, and differently reactive robots under local sensing.

**2. Separate useful forecasts from accepted motion.** CV, CTRV, stopping, and map-compatible branches provide an inspectable starting point. SFM/SICNav/diffusion predictions can improve passing/yielding choices, but another robot's predicted reaction cannot be enforced. Acceptance should use an explicitly stated occupancy/uncertainty model, executable dynamics, latency, and a defined continuation.

**3. A stop needs a precise promise.** OA-MPC's formulation permits collision-condition relaxation when ego is stationary; it does not prove a stopped robot can never be hit [R58]. A collision-free horizon ending at speed, a passive stopping condition, and a collision-free terminal region are different properties. The architecture and evaluator must name which one is established.

**4. Improve timing choices before adding a large optimizer.** A bounded lateral family plus explicit follow, stop, and arrival-after-conflict modes can resolve encounters that pure detours handle poorly. Werling supplies useful structure; TEB/MPCC supply refinement ideas. Every result still needs the same footprint/dynamics/time acceptance. Use SE(2) recovery for reversals and turns where Frenet coordinates fail.

The choice of a full separate candidate planner remains open. Compare a minimal prototype with tuned MPPI and an MPPI dynamic-occupancy extension after establishing the common experimental and execution contracts. A basic ROS/Nav2 benchmark adapter is needed for that comparison before full robot integration.

**5. Execution owns the plan's clock.** An ordinary geometric path does not enforce a crossing slot. A timed tracker or a deliberately extended controller needs expiry, state/timing tolerances, monitoring, and missed-deadline behavior. Retain a valid previous continuation while new plans are being computed. A late or merely numerically usable solution is not automatically an accepted command.

**6. Keep coordination as a separate later experiment.** SIPP-IP/CTC, RHCR/LaCAM, Concrete, and RMADER are relevant when participants share information and commitments. Sensor-derived free windows are predictions, not reservations against unknown traffic. Deadlock freedom cannot be promised if a nonparticipating robot permanently blocks the only passage.

## What needs changing in this simulator

The current code already has timed predictions, footprint checks, bounded rollout, limited visibility, and encounter memory. The largest gaps are their contracts and test coverage:

- The fallback predictor translates at constant world velocity while rotating heading; this differs materially from CTRV.
- Observations use ideal states and simulator-known identities/footprints; the benchmark does not yet reproduce real detection, association, covariance, and track-loss behavior.
- All robots use the same planner, so shared side/priority policy can hide unknown-robot failures.
- A finite accepted horizon does not currently establish a checked stop/hold continuation; even the braking candidate can be infeasible.
- Plan expiry, deadline failure, tracking deviation, and heterogeneous model behavior need explicit handling.
- Forward-only candidate/speed families and persistent side commitments can miss feasible timing or recovery options.

The [file-level design map](02_system_design.md#9-immediate-mapping-to-this-repository) describes the reviewed checkout. The first implementation slice should add independent actor policies, replayable observations, and independent collision scoring. Then correct prediction/model contracts and add continuation acceptance. This makes later optimizer and interaction-model comparisons meaningful.

## Reproduce the calculations

From the repository root:

```bash
python3 docs/research/deployment_review/analytical_checks.py
```

The script uses only Python's standard library and writes [analytical_results.json](analytical_results.json). Checks include finite-difference agreement for varying-curvature right-normal Frenet geometry and explicit counterexamples to unsafe ellipse inflation. These checks support mathematical/design conclusions; they are not robot-navigation experiments.

The attached GPT conversation has been incorporated and audited. This work does not add a runtime conversational interface. Any later natural-language mission interface should remain separate from deterministic motion acceptance and protective response.
