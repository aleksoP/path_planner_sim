#pragma once
#include "reference_path.hpp"
namespace cooperative_avoidance {
double quintic(double u);
struct Deformation {
  double offset{0}, start{0}, entry_end{0}, hold_end{0}, exit_end{0};
  double initial_offset{0}, initial_slope{0};
  double lateral(double s) const;
  Vector2 position(const ReferencePath&, double s) const;
};
TimedTrajectory rollout(const RobotState&, const ReferencePath&, const Deformation&,
                        double speed_scale, const PlannerConfig&,
                        double goal_heading=std::numeric_limits<double>::quiet_NaN());
} // namespace cooperative_avoidance
