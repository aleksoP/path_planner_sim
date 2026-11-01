# Reproducible checks behind the recommendations

These are our own small analytical examples. They do not reproduce a paper's navigation benchmark. Run `python3 docs/research/deployment_review/analytical_checks.py` from the repository root; it writes [the results](analytical_results.json). All dimensions and assumptions are explicit in [the script](analytical_checks.py).

## Motion models

A robot at 1 m/s turning at 0.5 rad/s for 3 s travels along an arc under CTRV. Its endpoint differs by approximately 2.11 m from constant world-velocity translation. Rotating only its footprint does not repair this positional discrepancy. This motivates testing the current predictor with turning actors before adding learned interaction models.

## Stopping with an unknown opponent

At 1.5 m/s, 0.12 s response delay, and 1.2 m/s² guaranteed deceleration, ego stops in 1.37 s after traveling 1.1175 m. A 1.5 m/s nonbraking opponent travels 2.055 m during that interval. With 0.15 m clearance, the initial **bumper-to-bumper** gap needed to avoid contact up to ego's stop is 3.3225 m, before uncertainty allowances. If both robots brake under the same assumptions, the corresponding gap is 2.385 m. Center separation additionally includes robot extents.

This example neither establishes a protective field for real hardware nor prevents the continuing opponent from hitting us after we stop. It distinguishes cooperative braking, collision-free motion until stopping, and indefinitely collision-free terminal occupancy.

## Ellipse inflation

For an ellipse with semiaxes `a,b`, its support in a unit direction `n` is `sqrt(a²*n_x²+b²*n_y²)`. Adding a disc of radius `rho` adds `rho` to that support. Replacing the Minkowski sum with an ellipse of semiaxes `a+rho,b+rho` can yield a **smaller** support in some directions. The script demonstrates this for `(a,b,rho)=(2,0.2,0.5)` and a 45-degree direction. Therefore that commonly used approximation is not automatically conservative.

Xin's printed exclusion equation uses `a²+r²` and `b²+r²` in the denominators [R04, PDF p.5, Eq.4]. Even the circular special case `(a=b=1,r=0.5)` excludes only a radius of `sqrt(1.25)=1.118...`, whereas nonintersecting discs require 1.5 m. This is a counterexample to treating the printed formula alone as a footprint-separation certificate; it is not a claim that every reported experiment collided. Additional unstated inflation would need separate verification.

## Frenet signs and physical smoothness

The script checks the right-normal heading and curvature formulas in [the design](02_system_design.md#right-normal-frenet-conversion) against finite differences of an independent Cartesian construction. The reference is deliberately not parameterized by arc length and has varying signed curvature. This exercises the curvature derivative term and both signs of curvature. Numerical agreement is evidence for the derivation, not for the current implementation or a future projection algorithm.

For `d(s)=D*(10u³−15u⁴+6u⁵)`, `u=s/S`, constant longitudinal speed `v`, the squared temporal lateral jerk integral is `720*D²/(S/v)^5`. Tripling speed multiplies this cost by 243. A spatial quintic is not a speed-independent comfort or actuator guarantee.

## Risk and trial counts

Fifty time samples, two neighbors, and a 5% failure probability at each pair/time have a union bound of 1 after capping. That bound is uninformative. Correlation does not justify multiplying pointwise confidences as if independent. Continuous-time events require coverage between samples as well.

With zero observed events in 3,000 independent, identically distributed Bernoulli trials, the one-sided 95% upper confidence limit is about 0.001 per trial. This is an illustrative statistical statement about the tested distribution, not a guarantee over new traffic policies, a robot lifetime, or correlated near-identical seeds. Report exposure and the operational domain alongside any confidence interval.
