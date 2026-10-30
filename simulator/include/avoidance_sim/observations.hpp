#pragma once
#include "cooperative_avoidance/types.hpp"
namespace avoidance_sim {
// Ideal range/field-of-view sensor with static-polygon occlusion. No planning policy.
bool observable(const cooperative_avoidance::RobotState& observer,
                const cooperative_avoidance::RobotState& target,
                const std::vector<cooperative_avoidance::Polygon>& walls,
                double range, double field_of_view, bool line_of_sight);
} // namespace avoidance_sim
