#!/usr/bin/env python3
"""Reproduce analytical examples in docs/research; not a robot planner benchmark."""
import argparse
import json
import math
from pathlib import Path


def integrate(function, n=20000):
    """Composite Simpson integral over [0,1]."""
    return (function(0)+function(1)+sum((4 if i % 2 else 2)*function(i/n)
                                      for i in range(1, n)))/(3*n)


def analytical_examples():
    amplitude, length = .85, 3.2
    rows = []
    for speed in (.5, 1., 1.5):
        # For a straight reference at constant s_dot, d=D*q(s/L).
        duration = length/speed
        integral = integrate(lambda u: (60-360*u+360*u*u)**2)
        jerk_cost = amplitude**2*integral/duration**5
        exact = 720*amplitude**2/duration**5
        assert math.isclose(jerk_cost, exact, rel_tol=1e-10)
        rows.append(dict(speed=speed, duration=duration, lateral_jerk_squared_integral=jerk_cost))
    kappa, right_offset, delta = 1., .85, math.radians(15)
    tangent_scale = 1+kappa*right_offset
    expected_slope = -tangent_scale*math.tan(delta)
    baseline_slope = -math.tan(delta)
    resulting_angle = math.atan(-baseline_slope/tangent_scale)
    speed, braking, delay = 1.5, 1.2, .12
    stop = speed*delay+speed**2/(2*braking)
    # Nonzero circulation of DTEB's approximate B field in a region avoiding its singularity.
    # The two forward-time curves have identical endpoints and are homotopic in y=1.
    def circulation(a):
        def f(t):
            x = 1+t+a*math.sin(math.pi*t)
            dx = 1+a*math.pi*math.cos(math.pi*t)
            relative = x-.3*t
            return dx/(2*math.pi*(relative*relative+1))
        return integrate(f)
    signatures = [circulation(a) for a in (0., .2)]
    # Sum(dt^2) is not invariant to discretization and is not total time.
    sum_squares = [dict(total_time=2., intervals=n, objective=4/n) for n in (10, 20)]
    cv = (3., 0.)
    ctrv = (math.sin(.5*3)/.5, (1-math.cos(.5*3))/.5)
    return dict(
        note="Analytical examples only; no claims about measured robot performance",
        quintic_constant_speed=rows,
        right_normal_curved_reference=dict(kappa=kappa, offset=right_offset,
            desired_heading_degrees=15, tangent_scale=tangent_scale,
            correct_d_prime=expected_slope, straight_approximation_d_prime=baseline_slope,
            resulting_heading_degrees=math.degrees(resulting_angle)),
        stopping=dict(speed=speed, deceleration=braking, delay=delay,
            one_robot_center_travel=stop, two_robot_gap_consumption=2*stop,
            gap_with_additional_clearance=.15+2*stop),
        teb_discretization=sum_squares,
        dteb_approximate_signature=dict(path_a=signatures[0], path_b=signatures[1],
            difference=signatures[0]-signatures[1], obstacle_velocity=.3,
            geometry="x(t)=1+t+a*sin(pi*t), y(t)=1, obstacle=(0.3*t,0), a=0 or 0.2"),
        prediction=dict(speed=1., yaw_rate=.5, horizon=3., cv_position=cv,
            ctrv_position=ctrv, center_prediction_difference=math.dist(cv, ctrv)))


def plot(destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13.8, 4.2), layout="constrained")
    speeds = [.1+i*.01 for i in range(141)]
    axes[0].plot(speeds, [720*.85**2*(v/3.2)**5 for v in speeds], color="#2871a0")
    axes[0].set(title="Same spatial quintic, different timing", xlabel="Speed [m/s]",
                ylabel="Integral of lateral jerk² [m²/s⁵]")
    offsets = [-1.2+i*.01 for i in range(241)]
    for curvature in (-1., 0., 1.):
        axes[1].plot(offsets, [1+curvature*d for d in offsets], label=f"κ = {curvature:g} 1/m")
    axes[1].axhline(0, color="black", ls="--", lw=.8)
    axes[1].set(title="Right-normal chart tangent scale", xlabel="Right offset d [m]", ylabel="A = 1 + κd")
    axes[1].legend(fontsize=8)
    for delay in (0., .12, .3):
        axes[2].plot(speeds, [v*delay+v*v/(2*1.2) for v in speeds], label=f"Delay {delay:g} s")
    axes[2].set(title="Illustrative bounded-braking distance", xlabel="Initial speed [m/s]", ylabel="Center travel before rest [m]")
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.2)
    fig.suptitle("Analytical examples for planner design — not physical validation", fontsize=12)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    result = analytical_examples()
    content = json.dumps(result, indent=2)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content)
    print(content, end="")
    if args.plot:
        plot(args.plot)


if __name__ == "__main__":
    main()
