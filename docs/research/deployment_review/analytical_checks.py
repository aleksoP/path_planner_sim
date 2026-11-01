#!/usr/bin/env python3
"""Reproducible review calculations; no third-party dependencies or robot control.

Run from any directory. Results are written beside this file. These are numerical
examples and geometry checks, not reproductions of the papers' experiments.
"""
import json
import math
from pathlib import Path


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def frenet_check(u, h=1e-4):
    # Reference r(u)=(u,c*u^3), deliberately NOT parameterized by arc length.
    # d is also specified in u; convert both derivatives to arc length below.
    c = 0.11
    speed = math.hypot(1, 3 * c * u * u)
    speed_u = 18 * c * c * u**3 / speed
    kappa = 6 * c * u / speed**3
    kappa_s = (6 * c / speed**3 - 18 * c * u * speed_u / speed**4) / speed
    d = 0.13 + 0.07 * u + 0.025 * u * u
    d_u, d_uu = 0.07 + 0.05 * u, 0.05
    dp = d_u / speed
    dpp = d_uu / speed**2 - d_u * speed_u / speed**3
    A = 1 + kappa * d
    predicted = (kappa * A * A + 2 * kappa * dp * dp
                 + kappa_s * d * dp - A * dpp) / (A * A + dp * dp)**1.5

    def point(z):
        dz = 0.13 + 0.07 * z + 0.025 * z * z
        v = math.hypot(1, 3 * c * z * z)
        # Right normal (ty,-tx).
        return (z + dz * 3 * c * z * z / v, c * z**3 - dz / v)

    lo, mid, hi = point(u - h), point(u), point(u + h)
    p1 = tuple((hi[i] - lo[i]) / (2 * h) for i in range(2))
    p2 = tuple((hi[i] - 2 * mid[i] + lo[i]) / h**2 for i in range(2))
    numerical = cross(p1, p2) / math.hypot(*p1)**3
    reference_heading = math.atan2(3 * c * u * u, 1)
    heading = math.atan2(p1[1], p1[0])
    numerical_delta = math.atan2(math.sin(heading - reference_heading),
                                 math.cos(heading - reference_heading))
    predicted_delta = math.atan2(-dp, A)
    return dict(u=u, curvature_formula=predicted, curvature_finite_difference=numerical,
                curvature_error=abs(predicted - numerical),
                heading_error=abs(predicted_delta - numerical_delta))


def main():
    v, w, T = 1.0, 0.5, 3.0
    ctrv = (v / w * math.sin(w * T), v / w * (1 - math.cos(w * T)))
    cv = (v * T, 0.0)
    ego_v, peer_v, delay, brake, clearance = 1.5, 1.5, 0.12, 1.2, 0.15
    stop_t = delay + ego_v / brake
    ego_stop = ego_v * delay + ego_v**2 / (2 * brake)
    # Support of ellipse+disc vs ellipse with rho added to each semiaxis.
    a, b, rho, n = 2.0, 0.2, 0.5, 1 / math.sqrt(2)
    true_support = math.hypot(a * n, b * n) + rho
    axis_added_support = math.hypot((a + rho) * n, (b + rho) * n)
    frenet = [frenet_check(u) for u in (-1.5, -0.3, 0.0, 0.7, 1.8)]
    D, S = 0.85, 3.2
    speeds = (0.5, 1.0, 1.5)
    # q(tau)=10tau^3-15tau^4+6tau^5; integral_0^1 q'''^2 dtau=720.
    jerk = {str(s): 720 * D * D / (S / s)**5 for s in speeds}
    results = {
        "scope": "Illustrative analytical checks, not navigation experiments or safety certification",
        "cv_ctrv": {"v_mps": v, "yaw_rate_radps": w, "horizon_s": T,
                    "cv_position_m": cv, "ctrv_position_m": ctrv,
                    "position_difference_m": math.dist(cv, ctrv)},
        "head_on_stop": {"ego_speed_mps": ego_v, "peer_speed_mps": peer_v,
                         "response_delay_s": delay, "guaranteed_deceleration_mps2": brake,
                         "time_until_ego_stops_s": stop_t, "ego_center_travel_m": ego_stop,
                         "nonbraking_peer_travel_m": peer_v * stop_t,
                         "bumper_gap_until_ego_stop_with_clearance_m": ego_stop + peer_v * stop_t + clearance,
                         "both_brake_same_delay_gap_with_clearance_m": 2 * ego_stop + clearance,
                         "caveat": "Bumper gaps exclude extra uncertainty. Nonbraking peer can still hit ego after ego stops."},
        "ellipse_plus_disc": {"ellipse_axes_m": [a, b], "disc_radius_m": rho,
                              "true_minkowski_support_m": true_support,
                              "axes_added_ellipse_support_m": axis_added_support,
                              "support_shortfall_m": true_support - axis_added_support},
        "mpcc_squared_radius_formula": {"obstacle_circle_radius_m": 1.0,
                                         "ego_circle_radius_m": 0.5,
                                         "exclusion_radius_in_printed_formula_m": math.sqrt(1.0 + 0.5**2),
                                         "required_circle_sum_radius_m": 1.5},
        "right_normal_frenet": frenet,
        "quintic_temporal_jerk_integral": {"lateral_offset_m": D, "longitudinal_span_m": S,
                                            "integral_m2_per_s5_by_speed_mps": jerk,
                                            "ratio_1_5_to_0_5_mps": jerk['1.5'] / jerk['0.5']},
        "probability_accounting": {"pointwise_failure_probability": 0.05,
                                    "agent_count": 2, "time_sample_count": 50,
                                    "union_bound_episode_failure_upper_bound": min(1.0, 2 * 50 * 0.05),
                                    "zero_events_in_3000_independent_trials_one_sided_95pct_upper": 1 - 0.05**(1 / 3000),
                                    "caveat": "Union bound can be vacuous; trial calculation assumes independent identically distributed Bernoulli trials."}
    }
    assert max(x['curvature_error'] for x in frenet) < 1e-6
    assert max(x['heading_error'] for x in frenet) < 1e-7
    assert true_support > axis_added_support
    assert math.isclose(jerk['1.5'] / jerk['0.5'], 3**5)
    target = Path(__file__).with_name('analytical_results.json')
    target.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
