#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
PredictedObstacleTrajectory predict_obstacle(const ObstacleState&, double now, const PlannerConfig&);
} // namespace cooperative_avoidance
