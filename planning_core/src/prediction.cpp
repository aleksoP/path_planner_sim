#include "cooperative_avoidance/prediction.hpp"
#include "cooperative_avoidance/trajectory.hpp"
#include "cooperative_avoidance/geometry.hpp"
namespace cooperative_avoidance {
PredictedObstacleTrajectory predict_obstacle(const ObstacleState& o,double now,const PlannerConfig& c) {
  PredictedObstacleTrajectory p; p.id=o.id; p.footprint=o.footprint;
  const double age=std::max(0.,now-o.timestamp);
  const double intent_age=now-o.intention_timestamp;
  p.used_intention=o.cooperative&&intent_age>=-1e-8&&intent_age<=c.intent_max_age&&
    !o.intention.empty()&&o.intention.back().time>=intent_age+std::min(c.horizon,1.0)&&
    dynamically_feasible(o.intention,MotionLimits{100,100,100,100,{100,100,100}});
  p.initial_uncertainty=std::max(0.,o.uncertainty)+(1-std::clamp(o.confidence,0.,1.))*0.2+c.uncertainty_growth*age;
  p.uncertainty_growth=c.uncertainty_growth;
  p.uncertainty=p.initial_uncertainty+c.uncertainty_growth*c.horizon;
  const size_t steps=static_cast<size_t>(std::ceil(c.horizon/c.prediction_step));
  for(size_t i=0;i<=steps;++i) {
    double t=std::min(c.horizon,i*c.prediction_step);
    TrajectoryPoint point;
    if(p.used_intention) {
      point=interpolate(o.intention,t+intent_age);
      const double tail=std::max(0.,t+intent_age-o.intention.back().time);
      point.pose.position=point.pose.position+Vector2{std::cos(point.pose.heading),std::sin(point.pose.heading)}*(point.velocity.linear*tail);
      point.pose.heading=wrap_angle(point.pose.heading+point.velocity.angular*tail);
      point.time=t;
    }
    else point={t,{o.pose.position+o.velocity*(t+age),wrap_angle(o.pose.heading+o.angular_velocity*(t+age))},
                       {norm(o.velocity),o.angular_velocity}};
    p.trajectory.push_back(point);
  }
  return p;
}
} // namespace cooperative_avoidance
