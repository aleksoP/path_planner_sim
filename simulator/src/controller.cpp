#include "avoidance_sim/controller.hpp"
#include "cooperative_avoidance/trajectory.hpp"
#include "cooperative_avoidance/geometry.hpp"
namespace avoidance_sim {
namespace ca=cooperative_avoidance;
ca::Twist2D TrajectoryFollower::command(const ca::RobotState& r,const ca::TimedTrajectory& trajectory,double elapsed,double dt) const {
  if(trajectory.empty()||elapsed>=trajectory.back().time)return {0,0};
  // End-of-control-interval velocity compensates for the actuator model's midpoint integration.
  auto reference=ca::interpolate(trajectory,elapsed),next=ca::interpolate(trajectory,elapsed+dt);
  auto error=reference.pose.position-r.pose.position;
  double along=std::cos(r.pose.heading)*error.x+std::sin(r.pose.heading)*error.y;
  double lateral=-std::sin(r.pose.heading)*error.x+std::cos(r.pose.heading)*error.y;
  return {std::clamp(next.velocity.linear+1.5*along,-r.limits.max_reverse,r.limits.max_forward),
    std::clamp(next.velocity.angular+2*ca::wrap_angle(reference.pose.heading-r.pose.heading)+2*lateral,
               -r.limits.max_angular,r.limits.max_angular)};
}
} // namespace avoidance_sim
