#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
struct CollisionEvaluation {
  bool feasible{true};
  double minimum_separation{std::numeric_limits<double>::infinity()};
  double conflict_time{-1};
  std::string reason;
  std::string obstacle_id;
  size_t static_index{std::numeric_limits<size_t>::max()};
};
CollisionEvaluation evaluate_trajectory(const TimedTrajectory&, RobotFootprint,
  const std::vector<Polygon>& static_obstacles,
  const std::vector<PredictedObstacleTrajectory>& moving_obstacles,
  double clearance, double spatial_resolution,
  double initial_clearance=std::numeric_limits<double>::quiet_NaN(), double margin_recovery_time=0.5);
} // namespace cooperative_avoidance
