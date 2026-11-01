# AMR navigation research: PDF reading & download index

> Research update, 1 November 2026: all 23 linked PDFs have now been downloaded into `02_downloaded_open_pdfs`. Additional public manuscripts are in `03_additional_verified_pdfs`. The original five remain in `01_existing_pdfs`. See the [review and access limits](../research/deployment_review/README.md), [clickable source coverage](../research/deployment_review/06_source_coverage.md), and [acquisition ledger](../research/deployment_review/sources.json). The supplied manifest and reading index are preserved as the original collection description.

## First reading order for sensor-only robot

1. OA-MPC (blind corners and safe stopping)
2. ORCA-DD and ORCA-AVO (unknown differential-drive agents)
3. Werling 2010 (Frenet motion), TEB 2012 (timing), Trepella 2026 (interactive prediction)
4. SICNav, SICNav-Diffusion, Trajectron++ (prediction)
5. Schöneberg 2025 uncertainty-aware MPC (ROS 2 integration)

## Included as local PDF files

- `01_Werling_2010_Frenet.pdf` — Frenet trajectory generation
- `02_Roesmann_2012_TEB.pdf` — Original timed elastic band
- `03_Roesmann_2017_TEB.pdf` — Kinodynamic trajectory refinement (car-like)
- `04_Chung_2022_DTEB.pdf` — Distributed timed trajectories (optional cooperation)
- `05_VanDenBerg_ORCA.pdf` — Independent reciprocal collision avoidance

## Direct open PDF links (additional sources)

### 01 Sensor-only core

- **P0 [Occlusion-Aware MPC for Guaranteed Safe Robot Navigation With Unseen Dynamic Obstacles](https://msl.stanford.edu/papers/firoozi_occlusion-aware_2022.pdf)** — Unseen agents, reachable occupancy and terminal stop feasibility. (Sensor-only useful)
- **P0 [Smooth and Collision-Free Navigation for Multiple Robots Under Differential-Drive Constraints](https://gamma.cs.unc.edu/ORCA-DD/ORCA-DD.pdf)** — Reciprocal motion without communication, differential-drive kinematics. (Sensor-only useful)
- **P0 [Smooth Coordination and Navigation for Multiple Differential-Drive Robots](https://gamma.cs.unc.edu/ORCA-AVO-DD/ORCA-AVO-DD.pdf)** — Acceleration-aware reciprocal collision avoidance. (Sensor-only useful)
- **P1 [SIPP: Safe Interval Path Planning for Dynamic Environments](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf)** — Space-time route availability with dynamic obstacles. (Sensor-only useful)
- **P0 [Trajectory Planning with MPC for Obstacle Avoidance Considering Prediction Uncertainty](https://arxiv.org/pdf/2504.19193)** — Uncertainty-aware dynamic obstacle prediction with ROS 2/Nav2. (Sensor-only useful)

### 02 Interaction prediction

- **P0 [Navigating the Crowd: Non-linear MPC with Social Forces Dynamics for Human-Aware Robot Navigation](https://arxiv.org/pdf/2607.10374)** — Robot-conditioned pedestrian motion inside MPC; Nav2 plugin. (Sensor-only useful)
- **P0 [SICNav: Safe and Interactive Crowd Navigation using MPC and Bilevel Optimization](https://arxiv.org/pdf/2310.10982)** — Robot-conditioned prediction via ORCA based human-response model. (Sensor-only useful)
- **P1 [SICNav-Diffusion: Safe and Interactive Crowd Navigation With Diffusion Trajectory Predictions](https://arxiv.org/pdf/2503.08858)** — Multimodal human trajectories and coupled planning. (Sensor-only useful)
- **P1 [Trajectron++: Dynamically-Feasible Trajectory Forecasting With Heterogeneous Data](https://arxiv.org/pdf/2001.03093)** — Map-aware multi-hypothesis prediction; optional ego conditioning. (Sensor-only useful)
- **P2 [Social LSTM: Human Trajectory Prediction in Crowded Spaces](https://www.cv-foundation.org/openaccess/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf)** — Foundational model of human-agent interaction. (Sensor-only useful)

### 03 Trajectory planning

- **P1 [D4orm: Multi-Robot Trajectories with Dynamics-aware Diffusion Denoised Deformations](https://arxiv.org/pdf/2503.12204)** — Anytime dynamical trajectory refinement including differential-drive models. (Sensor-only useful)
- **P1 [Reciprocal n-body Collision Avoidance](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)** — Foundational reciprocal velocity-obstacle analysis. (Sensor-only useful)

### 04 Optional group coordination

- **P2 [Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding](https://www.ijcai.org/Proceedings/2019/0076.pdf)** — Local priority concepts; assumptions do not fit arbitrary factory agents. (Optional peer coordination)
- **P2 [Distributed Timed Elastic Band Planner: Trajectory Sharing and Collision Prediction](https://arxiv.org/pdf/2303.11279)** — Communicated timed plans, future optional enhancement. (Optional peer coordination)
- **P2 [Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay](https://arxiv.org/pdf/2303.06222)** — Delayed intent/commitment; optional robots with communications. (Optional peer coordination)
- **P2 [LF: Online Multi-Robot Path Planning Meets Optimal Trajectory Control](https://arxiv.org/pdf/2507.11464)** — Global-knowledge multi-robot architecture comparison. (Optional peer coordination)
- **P2 [Multi-Agent Pathfinding with Continuous Time](https://ijcai.org/proceedings/2019/0006.pdf)** — Continuous time conflict search; only if future coordination included. (Optional peer coordination)
- **P2 [Real-Time Multi-Robot Motion Planning with Safe-Interval Search and Learning-Guided Repair](https://ijcai.org/proceedings/2026/0842.pdf)** — Learning-guided congestion repair; requires coordinated robots. (Optional peer coordination)
- **P2 [Congestion Mitigation Path Planning for Large-Scale Multi-Agent Navigation](https://arxiv.org/pdf/2508.05253)** — Coarse traffic-aware routing, requires multi-agent context. (Optional peer coordination)
- **P2 [EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System](https://arxiv.org/pdf/2011.04183)** — Local topology alternatives, aerial-model and communication caveats. (Optional peer coordination)

### 05 Benchmarking

- **P1 [HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation](https://arxiv.org/pdf/2305.01303)** — Reusable difficult pedestrian behavior scenarios for ROS 2. (Sensor-only useful)
- **P1 [Arena 4.0: A ROS2 Development and Benchmarking Platform](https://arxiv.org/pdf/2409.12471)** — Human-aware scenario generation, large PDF. (Sensor-only useful)
- **P1 [Arena 5.0: A ROS2 Social Navigation Benchmark](https://arxiv.org/pdf/2610.11220)** — Modern scenario and evaluation tools, large PDF. (Sensor-only useful)


## Full 69-reference literature catalog

This also contains journal/publisher entries where a full open-access PDF link is not established. Such links may lead to a publisher landing page instead.

### Trajectory generation / control

- [Werling et al. — Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://doi.org/10.1109/ROBOT.2010.5509799) — 2010; P0. Quintic lateral and quartic/quintic longitudinal maneuver families, replanning continuity.
- [Rösmann et al. — Trajectory Modification Considering Dynamic Constraints of Autonomous Robots](https://github.com/rst-tu-dortmund/teb_local_planner) — 2012; P1. Pose/time optimization, local sparse factors, wheel/differential-drive constraints.
- [Rösmann et al. — Kinodynamic Trajectory Optimization and Control for Car-Like Robots](https://doi.org/10.1109/IROS.2017.8206458) — 2017; P2. Warm-started SE(2) optimization and reverse seeds.
- [Xin et al. — Distributed Model Predictive Contouring Control for Real-Time Multi-Robot Motion Planning](https://doi.org/10.23919/CSMS.2022.0017) — 2022; P0. Separate route progress from contouring error; robots share predicted routes.
- [Schöneberg et al. — Trajectory Planning with MPC for Obstacle Avoidance Considering Prediction Uncertainty](https://doi.org/10.1016/j.ifacol.2025.10.245) — 2025; P1. ROS 2/Nav2-connected stochastic obstacle forecasting and chance-oriented MPC design.
- [Sun et al. — Integrated Multi-AMR CL-CBS and MPC-APF in Warehousing](https://doi.org/10.1016/j.simpat.2025.103122) — 2025; P2. Discrete conflict search with continuous local replanning and tracking.
- [Zhang et al. — D4orm: Multi-Robot Trajectories with Dynamics-aware Diffusion Denoised Deformations](https://arxiv.org/abs/2503.12204) — 2025; P1. Anytime trajectory refinement from dynamical rollout and stochastic denoising; no trained network required.
- [Shankar, Okumura and Prorok — LF: Online Multi-Robot Path Planning Meets Optimal Trajectory Control](https://arxiv.org/abs/2507.11464) — 2025; P0. Asynchronous high-level MAPF plus low-level optimal trajectory controllers with real experiments.
- [Samavi et al. — SICNav-Diffusion: Safe and Interactive Crowd Navigation With Diffusion Trajectory Predictions](https://doi.org/10.1109/LRA.2025.3585713) — 2025; P1. Multimodal joint pedestrian prediction coupled to a bilevel robot controller.
- [Trepella et al. — Navigating the Crowd: Non-linear MPC with Social Forces Dynamics](https://arxiv.org/abs/2607.10374) — 2026; P0. Candidate-conditioned human response forecasts embedded in real-time NMPC.
- [Sun et al. — A Decoupled Time-Optimized Multi-Robot Path Parameterization Method](https://doi.org/10.1007/s40747-025-02110-5) — 2026; P1. Separate path geometry and synchronized time parameterization under motion constraints.
- [Qu et al. — Safety Corridor and Pedestrian Prediction Integrated Sequential MPC](https://doi.org/10.1177/17298806261483759) — 2026; P1. Corridor constraints, Kalman motion forecasting, occlusion/track-loss padding, fallback.

### Distributed multi-robot avoidance

- [van den Berg et al. — Reciprocal n-body Collision Avoidance (ORCA)](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf) — 2011; P1. Fast reciprocal velocity-obstacle baseline with explicit theoretical conditions.
- [Snape et al. — ORCA for Differential-Drive Robots (ORCA-DD)](https://gamma-web.iacs.umd.edu/ORCA-DD/) — 2010; P1. Nonholonomic reciprocal-avoidance adaptation; more suitable benchmark than clamped ORCA.
- [Zhou et al. — Fast On-line Collision Avoidance for Dynamic Vehicles Using Buffered Voronoi Cells](https://msl.stanford.edu/bibliography/zhou_fast_2017) — 2017; P2. Lightweight localized convex safe regions from neighboring positions.
- [Wang and Schwager — Distributed Collision Avoidance with Probabilistic Buffered Voronoi Cells](https://msl.stanford.edu/bibliography/wang_distributed_2019) — 2019; P2. Explicitly treats relative-state uncertainty in reciprocal safe cell construction.
- [Zhu, Brito and Alonso-Mora — Decentralized Probabilistic Multi-Robot Collision Avoidance with Buffered Uncertainty-aware Voronoi Cells](https://github.com/tud-amr/mrca_vc) — 2022; P1. Differential-drive simulation examples and uncertainty-aware buffered cells.
- [Zhou et al. — EGO-Swarm: Fully Autonomous and Decentralized Quadrotor Swarm](https://arxiv.org/abs/2011.04183) — 2021; P1. Fast asynchronous trajectory exchange, multi-topology seeds, lightweight optimization.
- [Tordesillas et al. — MADER: Trajectory Planner in Multiagent and Dynamic Environments](https://doi.org/10.1109/TRO.2021.3080235) — 2022; P1. Committed trajectories, separating hyperplanes, collision checking/rechecking.
- [Kondo et al. — Robust MADER: Decentralized Multiagent Trajectory Planner Robust to Communication Delay](https://arxiv.org/abs/2303.06222) — 2024; P0. Two-phase trajectory publication, delay check, committed trajectory compatibility.
- [Chung et al. — Distributed Timed Elastic Band (DTEB) Planner](https://arxiv.org/abs/2303.11279) — 2022; P0. Arbitrary time-indexed peer intentions, priority and topology-aware multi-robot passing.
- [IFAC — Distributed MPC for Safety-Critical Obstacle Avoidance with Social Preferences](https://doi.org/10.1016/j.ifacol.2025.11.587) — 2025; P2. Asymmetric responsibility allocation and control barrier constraints for nonholonomic robots.
- [Muhammed et al. — Real-time Decentralized MPC for Cooperative Multi-Robot Object Transport](https://doi.org/10.1038/s41598-026-41881-w) — 2026; P2. ROS2 physical experiments, event-triggered communication, differential-drive MPC.

### Traffic scheduling / MAPF

- [Phillips and Likhachev — SIPP: Safe Interval Path Planning for Dynamic Environments](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf) — 2011; P0. Time intervals for available corridor/intersection occupancy.
- [Ali and Yakovlev — Safe Interval Path Planning with Kinodynamic Constraints (SIPP-IP)](https://doi.org/10.1609/aaai.v37i10.26453) — 2023; P0. Safe-interval search with acceleration and deceleration limitations.
- [Kasaura et al. — Prioritized SIPP with Continuous Time on 2D Roadmaps (PSIPP/CTC)](https://doi.org/10.1109/LRA.2022.3187265) — 2022; P0. Precomputed roadmap conflict intervals, feasible coordination on fixed warehouse graph.
- [Andreychuk et al. — Multi-Agent Pathfinding with Continuous Time (CCBS)](https://ijcai.org/proceedings/2019/6) — 2019; P1. Continuous-time conflict splitting, small-group reference solver.
- [Tan et al. — Robust Multi-Agent Pathfinding with Continuous Time](https://doi.org/10.1609/icaps.v34i1.31519) — 2024; P0. Bounded execution delay robustness for continuous-time multiagent plans.
- [Okumura et al. — Priority Inheritance with Backtracking (PIBT)](https://www.ijcai.org/Proceedings/2019/0076.pdf) — 2019; P1. Fast, adaptive local priority handoff and backtracking.
- [Li et al. — Lifelong MAPF in Large-Scale Warehouses (RHCR)](https://doi.org/10.1609/aaai.v35i13.17344) — 2021; P1. Rolling-horizon conflict resolution for repeated warehouse missions.
- [Li et al. — MAPF-LNS2: Fast Repairing via Large Neighborhood Search](https://doi.org/10.1609/aaai.v36i9.21266) — 2022; P1. Repair only colliding subset rather than replanning every robot.
- [Huang et al. — Anytime MAPF via ML-Guided Large Neighborhood Search](https://doi.org/10.1609/aaai.v36i9.21168) — 2022; P3. Use learning for choosing productive conflict-repair subsets.
- [Okumura — LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding](https://doi.org/10.1609/aaai.v37i10.26377) — 2023; P2. High scalability with lazy constraints.
- [Okumura — Improving LaCAM for Scalable Eventually Optimal MAPF](https://ijcai.org/proceedings/2023/28) — 2023; P2. Anytime MAPF search with fast initial routes and later improvement.
- [Kato et al. — Congestion Mitigation Path Planning for Large-Scale Multi-Agent Navigation](https://arxiv.org/abs/2508.05253) — 2025; P1. Spread route demand away from shared congestion hotspots proactively.
- [Amir et al. — ReCoDe: RL-Based Dynamic Constraint Design for Multi-Agent Coordination](https://proceedings.mlr.press/v305/amir25a.html) — 2025; P2. Learn auxiliary coordination constraints while retaining model-based controllers.
- [Okumura et al. — Concrete Multi-Agent Path Planning Enabling Kinodynamically Aggressive Maneuvers](https://www.nature.com/articles/s44182-026-00083-2) — 2026; P0. MAPF with execution constraints (MAPF-X), control-compatible dense robot coordination; 40-robot physical demo.
- [Kim et al. — A Scalable Framework for Lifelong MAPF with Asynchronous Actions](https://doi.org/10.1109/TII.2026.3681327) — 2026; P0. Separate path planning and precedence-based action scheduling under nonuniform durations.
- [Zheng et al. — Learning-guided Prioritized Planning for Lifelong MAPF in Warehouse Automation](https://doi.org/10.1613/jair.1.20611) — 2026; P2. Neural assignment of planning order on top of conventional prioritized search.
- [Klančar et al. — Elevating Priorities for Efficient Lifelong Multi-AGV Pathfinding on Roadmaps](https://doi.org/10.1016/j.robot.2025.105295) — 2026; P1. Safe locations and priority logic in lifelong roadmap coordination.
- [Tang et al. — Greedy Priority Inheritance with Backtracking](https://doi.org/10.1109/TASE.2026.3703194) — 2026; P2. Faster priority backflow for dense interactions.
- [Arita and Okumura — Local Guidance for Configuration-Based Multi-Agent Pathfinding](https://doi.org/10.1609/aaai.v40i35.40169) — 2026; P2. Local congestion-aware spatiotemporal guidance for dense map search.
- [Jain et al. — Graph Attention-Guided Search for Dense Multi-Agent Pathfinding (LaGAT)](https://doi.org/10.1609/aaai.v40i35.40192) — 2026; P3. Learned GNN heuristic guiding robust classical search in dense graphs.
- [Arita and Okumura — Lifelong LaCAM with Local Guidance](https://doi.org/10.1609/socs.v19i1.43067) — 2026; P2. Reuse warm-started spatiotemporal guidance under repeated new tasks.
- [Kumar et al. — Real-Time Multi-Robot Motion Planning with Safe-Interval Search and Learning-Guided Repair](https://ijcai.org/proceedings/2026/842) — 2026; P1. SIPP-PP plus LNS conflict repair and action-order exploration.
- [Huang et al. — SMART: Scalable Multi-Agent Reasoning and Trajectory Planning in Dense Environments](https://doi.org/10.1016/j.trc.2026.105793) — 2026; P2. Hierarchical priority reasoning and corridor-based trajectory refinement for dense vehicles.
- [Guo et al. — Collision Risk Index and Dynamic Priority in Multi-Robot Cooperative Planning](https://doi.org/10.1016/j.robot.2026.105592) — 2026; P3. Dynamic priorities and staged multi-robot conflict resolution.

### Prediction / interaction

- [Salzmann et al. — Trajectron++: Dynamically-Feasible Trajectory Forecasting with Heterogeneous Data](https://arxiv.org/abs/2001.03093) — 2020; P2. Multimodal, map-conditioned, interaction-aware and optionally ego-conditioned trajectories.
- [Sun et al. — M2I: From Factored Marginal to Interactive Trajectory Prediction](https://openaccess.thecvf.com/content/CVPR2022/html/Sun_M2I_From_Factored_Marginal_Trajectory_Prediction_to_Interactive_Prediction_CVPR_2022_paper.html) — 2022; P2. Influencer/reactor modeling for agents whose actions change each other.
- [Alahi et al. — Social LSTM: Human Trajectory Prediction in Crowded Spaces](https://www.cv-foundation.org/openaccess/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf) — 2016; P3. Foundational social-interaction-aware pedestrian forecasting.
- [Gupta et al. — Social GAN: Socially Acceptable Trajectories with Generative Adversarial Networks](https://openaccess.thecvf.com/content_cvpr_2018/papers/Gupta_Social_GAN_Socially_CVPR_2018_paper.pdf) — 2018; P3. Multiple human future modes; diversity as a baseline.
- [Yang et al. — IA-LSTM: Interaction-Aware LSTM for Pedestrian Trajectory Prediction](https://doi.org/10.1109/TCYB.2024.3359237) — 2024; P3. Attention to variable interpersonal interaction importance.
- [Samavi et al. — SICNav: Interactive Crowd Navigation with Bilevel Optimization](https://arxiv.org/abs/2310.10982) — 2024; P0. Predict human response through embedded ORCA model inside MPC, with real tests.
- [Autoware — Map-Based Prediction Implementation (versioned documentation)](https://autowarefoundation.github.io/autoware_universe/refs-tags-v1.0/perception/map_based_prediction/) — 2024; P0. Practical track history, map-branch association, maneuvers, predicted paths.
- [Qiu and Liu — Intent-Aware Spatiotemporal Risk Field for Multi-AGV Planning](https://doi.org/10.1007/s44443-026-00997-9) — 2026; P1. Separate cooperating peer intentions from heterogeneous outside-obstacle forecasting.
- [Meng et al. — Risk-Aware Multi-Modal Pedestrian Prediction with Scene-Attentive Social Attention](https://doi.org/10.1049/itr2.70252) — 2026; P3. Forecast spatial risk beyond average displacement accuracy.
- [Pan et al. — Mixed Crowd Navigation: Perception, Interaction, Planning, and Control (survey)](https://doi.org/10.1146/annurev-control-032024-023929) — 2026; P2. Comprehensive survey organizing perception–interaction–planning–control.

### Safety / occlusion

- [Firoozi et al. — OA-MPC: Occlusion-Aware MPC for Guaranteed Safe Robot Navigation with Unseen Dynamic Obstacles](https://arxiv.org/abs/2211.09156) — 2025; P0. Forward reachable occupancy for unseen agents and terminal stop for recursive feasibility.
- [Haraldsen et al. — Theoretical Analysis of Velocity Obstacle Method for Nonholonomic Vehicles](https://doi.org/10.1109/TCST.2024.3375023) — 2024; P1. Analyzes avoidance feasibility with vehicle dynamics instead of assuming ideal planar velocity.
- [Zheng et al. — Occlusion-Aware Contingency Safety-Critical Planning for Autonomous Driving](https://doi.org/10.1109/TCYB.2025.3632366) — 2026; P1. Phantom-agent reachable sets and contingency speed bounds.
- [Ye et al. — Risk-Aware and Socially Compliant Topological Motion Planning Under Uncertainty](https://doi.org/10.1049/csy2.70051) — 2026; P2. Multiple topology classes and uncertain human motion in chance-constrained local optimization.
- [Safety-Critical Coordination of Legged Robots via Forward Reachable Set CBFs](https://doi.org/10.1109/ICRA57147.2024.10610589) — 2024; P3. Reachability-informed layered safety filter and tracking.
- [Social Robot Navigation through Constrained Optimization: Uncertainty-Based Objectives and Constraints](https://www.sciencedirect.com/science/article/pii/S0921889024002148) — 2024; P2. Simulation and physical evaluation of uncertainty-aware social MPC constraints.

### Benchmark / implementation

- [Shcherbyna et al. — Arena 4.0 ROS2 Navigation Benchmark Platform](https://arxiv.org/abs/2409.12471) — 2024; P1. Generated worlds/scenarios and ROS2 comparison workflow.
- [Shcherbyna et al. — Arena 5.0 ROS2 Social Navigation Benchmark](https://arxiv.org/abs/2610.11220) — 2026; P1. Comprehensive generated scenarios, standardized benchmark metrics, expanded simulator support.
- [Pérez-Higueras et al. — HuNavSim ROS 2 Human Navigation Simulator](https://arxiv.org/abs/2305.01303) — 2023; P1. Configurable pedestrian behaviors and social navigation metrics.
- [Social Robot Navigation: A Review and Benchmarking of Learning-Based Methods](https://doi.org/10.3389/frobt.2025.1658643) — 2025; P2. Taxonomy of social navigators, comparison datasets, tools and sensing modalities.
- [Bridging Requirements, Planning, and Evaluation: A Review of Social Robot Navigation](https://pmc.ncbi.nlm.nih.gov/articles/PMC11086376/) — 2024; P2. Taxonomy of safety/social evaluation metrics and simulator benchmarks.
- [NVIDIA/ROS Nav2 — MPPI Controller Jazzy Documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/) — 2026; P0. Practical controller integration and critic extension points.


## Download helper

```bash
python3 download_open_papers.py --all
# Or omit --all for sensor-only/high-priority local navigation papers
python3 download_open_papers.py
```

Source limits: direct source links were checked where possible; large arXiv PDFs and institutional servers may rate-limit tools. This index is not a claim of a locally downloaded full-text copy for every reference.
