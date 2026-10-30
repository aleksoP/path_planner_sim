#pragma once
#include "cooperative_avoidance/types.hpp"
namespace avoidance_sim {
// Time-indexed feedforward plus pose-error feedback, independently replaceable.
class TrajectoryFollower {
 public:
  cooperative_avoidance::Twist2D command(const cooperative_avoidance::RobotState&,
    const cooperative_avoidance::TimedTrajectory&,double elapsed,double control_dt) const;
};
} // namespace avoidance_sim
