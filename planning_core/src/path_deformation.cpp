#include "cooperative_avoidance/path_deformation.hpp"
#include "cooperative_avoidance/motion_model.hpp"
#include "cooperative_avoidance/geometry.hpp"
namespace cooperative_avoidance {
double quintic(double u) { u=std::clamp(u,0.,1.); return u*u*u*(10+u*(-15+6*u)); }
double Deformation::lateral(double s) const {
  if(s>=exit_end) return 0;
  if(s<=start) return initial_offset;
  if(s<entry_end) {
    const double u=(s-start)/(entry_end-start);
    const double tangent_basis=u-6*u*u*u+8*u*u*u*u-3*u*u*u*u*u;
    return initial_offset+(offset-initial_offset)*quintic(u)+(entry_end-start)*initial_slope*tangent_basis;
  }
  if(s<=hold_end) return offset;
  return offset*(1-quintic((s-hold_end)/(exit_end-hold_end)));
}
Vector2 Deformation::position(const ReferencePath& p,double s) const {
  auto q=p.sample(s); return q.position+q.right_normal*lateral(s);
}
TimedTrajectory rollout(const RobotState& initial,const ReferencePath& path,const Deformation& deformation,
 double speed_scale,const PlannerConfig& config,double goal_heading) {
  TimedTrajectory out{{0,initial.pose,initial.velocity}};
  RobotState state=initial; UnicycleModel model;
  double progress=path.project(state.pose.position).s;
  const size_t n=static_cast<size_t>(std::ceil(config.horizon/config.rollout_step));
  for(size_t i=1;i<=n;++i) {
    const double time=std::min(config.horizon,i*config.rollout_step), dt=time-out.back().time;
    progress=std::max(progress,path.project(state.pose.position,std::max(0.,progress-0.2)).s);
    double look=std::max(config.lookahead,std::abs(state.velocity.linear)*0.5);
    auto target=deformation.position(path,std::min(path.length(),progress+look));
    auto delta=target-state.pose.position;
    double alpha=wrap_angle(std::atan2(delta.y,delta.x)-state.pose.heading);
    double curvature=2*std::sin(alpha)/std::max(0.15,norm(delta));
    curvature=std::clamp(curvature,-state.limits.max_curvature,state.limits.max_curvature);
    const double remaining=norm(path.sample(path.length()).position-state.pose.position);
    double speed=state.limits.max_forward*speed_scale;
    speed=std::min(speed,state.limits.max_angular/std::max(0.01,std::abs(curvature)));
    // Curvature slows traversal; actuator integration enforces all acceleration bounds.
    speed/=1+0.35*std::abs(curvature);
    speed=std::min(speed,std::sqrt(2*state.limits.acceleration.deceleration*std::max(0.,remaining-0.04)));
    speed=std::min(speed,1.5*remaining);
    Twist2D command{speed,speed*curvature};
    if(remaining<config.goal_tolerance) command={0,std::isfinite(goal_heading)?
      std::clamp(2*wrap_angle(goal_heading-state.pose.heading),-state.limits.max_angular,state.limits.max_angular):0};
    state=model.step(state,command,dt).state;
    out.push_back({time,state.pose,state.velocity});
  }
  return out;
}
} // namespace cooperative_avoidance
