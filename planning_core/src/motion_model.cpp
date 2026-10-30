#include "cooperative_avoidance/motion_model.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include <stdexcept>
namespace cooperative_avoidance {
void validate_limits(const MotionLimits& l) {
  for(double x:{l.max_forward,l.max_angular,l.max_curvature,l.acceleration.acceleration,
                l.acceleration.deceleration,l.acceleration.angular_acceleration})
    if(!std::isfinite(x)||x<=0) throw std::invalid_argument("motion limits must be finite and positive");
  if(!std::isfinite(l.max_reverse)||l.max_reverse<0) throw std::invalid_argument("invalid reverse limit");
}
MotionStep UnicycleModel::step(const RobotState& r,Twist2D c,double dt) const {
  if(!(dt>0)||!std::isfinite(dt)||!std::isfinite(c.linear)||!std::isfinite(c.angular))
    throw std::invalid_argument("invalid motion command or timestep");
  auto out=r; const auto& l=r.limits;
  const double requested_v=c.linear,requested_w=c.angular;
  c.linear=std::clamp(c.linear,-l.max_reverse,l.max_forward);
  c.angular=std::clamp(c.angular,-l.max_angular,l.max_angular);
  // Reversals must brake to zero first; acceleration resumes on a later step.
  if(c.linear*r.velocity.linear<0) c.linear=0;
  double rate=std::abs(c.linear)>std::abs(r.velocity.linear)?l.acceleration.acceleration:l.acceleration.deceleration;
  out.velocity.linear=r.velocity.linear+std::clamp(c.linear-r.velocity.linear,-rate*dt,rate*dt);
  out.velocity.angular=r.velocity.angular+std::clamp(c.angular-r.velocity.angular,
    -l.acceleration.angular_acceleration*dt,l.acceleration.angular_acceleration*dt);
  // Midpoint integration with linearly changing actuator velocities.
  double v=(r.velocity.linear+out.velocity.linear)/2, w=(r.velocity.angular+out.velocity.angular)/2;
  double half=r.pose.heading+w*dt/2;
  out.pose.position=out.pose.position+Vector2{std::cos(half),std::sin(half)}*(v*dt);
  out.pose.heading=wrap_angle(r.pose.heading+w*dt);
  return {out,std::abs(requested_v-out.velocity.linear)>1e-9||std::abs(requested_w-out.velocity.angular)>1e-9};
}
} // namespace cooperative_avoidance
