#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
TrajectoryPoint interpolate(const TimedTrajectory&, double relative_time);
bool dynamically_feasible(const TimedTrajectory&, const MotionLimits&, std::string* reason=nullptr);
} // namespace cooperative_avoidance
