#include "avoidance_sim/scenario.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include "cooperative_avoidance/motion_model.hpp"
#include <fstream>
#include <set>
#include <stdexcept>
namespace avoidance_sim {
namespace {
Json read(const std::string& file) {std::ifstream in(file);if(!in)throw std::runtime_error("cannot open "+file);Json j;in>>j;return j;}
ca::Vector2 vector(const Json& j) {if(j.size()!=2)throw std::invalid_argument("point must have two coordinates");return {j.at(0),j.at(1)};}
ca::Pose2D pose(const Json& j) {if(j.size()!=3)throw std::invalid_argument("pose must be [x,y,yaw]");return {{j.at(0),j.at(1)},j.at(2)};}
ca::MotionLimits limits(const Json& j) {
  ca::MotionLimits l;
  l.max_forward=j.value("max_forward",l.max_forward);l.max_reverse=j.value("max_reverse",l.max_reverse);
  l.max_angular=j.value("max_angular",l.max_angular);l.max_curvature=j.value("max_curvature",l.max_curvature);
  l.acceleration.acceleration=j.value("acceleration",l.acceleration.acceleration);
  l.acceleration.deceleration=j.value("deceleration",l.acceleration.deceleration);
  l.acceleration.angular_acceleration=j.value("angular_acceleration",l.acceleration.angular_acceleration);
  ca::validate_limits(l);return l;
}
ca::PlannerConfig config(const Json& j) {
  ca::PlannerConfig c;
#define PARAM(name) c.name=j.value(#name,c.name)
  PARAM(horizon);PARAM(prediction_step);PARAM(rollout_step);PARAM(clearance);PARAM(tracking_margin);PARAM(tracking_recovery_time);
  PARAM(intent_max_age);PARAM(uncertainty_growth);PARAM(transition_length);PARAM(passing_length);
  PARAM(return_clearance);PARAM(lookahead);PARAM(goal_tolerance);PARAM(commitment_time);PARAM(recovery_time);
  PARAM(collision_resolution);PARAM(offsets);PARAM(speed_scales);PARAM(transition_scales);
#undef PARAM
  const auto w=j.value("weights",Json::object());
#define WEIGHT(name) c.weights.name=w.value(#name,c.weights.name)
  WEIGHT(progress);WEIGHT(deviation);WEIGHT(speed_reduction);WEIGHT(smoothness);WEIGHT(clearance);WEIGHT(left_side);WEIGHT(switching);
#undef WEIGHT
  ca::validate_config(c);return c;
}
ca::Polygon polygon(const Json& j) {
  ca::Polygon p;
  if(j.contains("polygon")) for(const auto& v:j.at("polygon")) p.push_back(vector(v));
  else {const auto size=vector(j.at("size"));p=ca::rectangle(pose(j.at("pose")),{size.x,size.y});}
  if(!ca::convex_polygon(p)) throw std::invalid_argument("only finite convex obstacle polygons are supported");
  return p;
}
}
Scenario load_scenario(const std::string& file,const std::string& override_file) {
  Scenario s;s.source=read(file);const auto& j=s.source;
  s.name=j.at("name");
  auto sim=j.value("simulation",Json::object());
  s.timestep=sim.value("timestep",s.timestep);s.planner_period=sim.value("planner_period",s.planner_period);
  s.control_period=sim.value("control_period",s.control_period);s.duration=sim.value("duration",s.duration);
  s.record_period=sim.value("record_period",s.record_period);
  for(double x:{s.timestep,s.planner_period,s.control_period,s.duration,s.record_period})
    if(!std::isfinite(x)||x<=0)throw std::invalid_argument("simulation intervals must be positive");
  // Integer tick scheduling avoids floating-point frequency drift. Reject unsupported ratios explicitly.
  for(double x:{s.planner_period,s.control_period,s.record_period})
    if(x<s.timestep||std::abs(x/s.timestep-std::round(x/s.timestep))>1e-8)
      throw std::invalid_argument("planner/control/record periods must be integer multiples of timestep");
  auto obs=j.value("observations",Json::object());
  s.ground_truth_observations=obs.value("ground_truth",true);s.observation_range=obs.value("range",30.);
  s.observation_line_of_sight=obs.value("line_of_sight",true);
  s.observation_field_of_view=obs.value("field_of_view_degrees",360.)*ca::pi/180;
  s.observation_memory=obs.value("memory",.5);
  if(!s.ground_truth_observations)throw std::invalid_argument("V0.1 supports explicit synthetic ground-truth observations only");
  if(!std::isfinite(s.observation_range)||s.observation_range<=0)throw std::invalid_argument("invalid observation range");
  if(!std::isfinite(s.observation_field_of_view)||s.observation_field_of_view<=0||s.observation_field_of_view>2*ca::pi||
     !std::isfinite(s.observation_memory)||s.observation_memory<0)throw std::invalid_argument("invalid observation field of view or memory");
  Json params=j.value("planner",Json::object());if(!override_file.empty())params.merge_patch(read(override_file));
  s.planner=config(params);
  for(const auto& o:j.at("obstacles")) s.obstacles.push_back(polygon(o));
  std::set<std::string> ids;
  for(const auto& r:j.at("robots")) {
    ca::RobotState robot;robot.id=r.at("id");robot.pose=pose(r.at("start"));
    const auto f=r.at("footprint");robot.footprint={f.at("length"),f.at("width")};
    if(robot.footprint.length<=0||robot.footprint.width<=0||!std::isfinite(robot.footprint.length)||!std::isfinite(robot.footprint.width))
      throw std::invalid_argument("invalid robot footprint");
    robot.limits=limits(r.value("limits",Json::object()));
    const auto v=r.value("velocity",Json::array({0.,0.}));robot.velocity={v.at(0),v.at(1)};
    auto goal=pose(r.at("goal"));
    for(double x:{robot.pose.position.x,robot.pose.position.y,robot.pose.heading,goal.position.x,goal.position.y,goal.heading,
                   robot.velocity.linear,robot.velocity.angular})
      if(!std::isfinite(x))throw std::invalid_argument("non-finite robot state");
    if(std::abs(robot.velocity.angular)>robot.limits.max_angular||robot.velocity.linear>robot.limits.max_forward||robot.velocity.linear<-robot.limits.max_reverse)
      throw std::invalid_argument("initial velocity exceeds limits");
    if(robot.id.empty()||!ids.insert(robot.id).second)throw std::invalid_argument("robot IDs must be unique and nonempty");
    std::vector<ca::Vector2> path;for(const auto& p:r.at("path"))path.push_back(vector(p));
    ca::ReferencePath reference(path,r.value("smoothing_passes",3u));
    if(ca::norm(reference.sample(reference.length()).position-goal.position)>0.001)
      throw std::invalid_argument("path endpoint must equal goal position");
    s.robots.push_back({robot,std::move(reference),goal,r.value("cooperative",true)});
  }
  if(s.robots.empty())throw std::invalid_argument("scenario needs at least one robot");
  // Canonical ID order, independent of input ordering; all planning still uses one frozen snapshot.
  std::sort(s.robots.begin(),s.robots.end(),[](const auto& a,const auto& b){return a.initial.id<b.initial.id;});
  s.events=j.value("events",Json::array());
  for(const auto& e:s.events) {
    double t=e.at("time");if(!std::isfinite(t)||t<0)throw std::invalid_argument("invalid event time");
    std::string type=e.at("type");
    if(type=="add_obstacle")polygon(e.at("obstacle"));
    else if(type=="stop_robot"||type=="communication_loss") {
      if(!ids.count(e.at("robot").get<std::string>()))throw std::invalid_argument("event references unknown robot");
      if(type=="communication_loss"&&e.value("duration",0.)<=0)throw std::invalid_argument("communication loss needs duration");
    } else throw std::invalid_argument("unknown event type: "+type);
  }
  return s;
}
Json pose_json(ca::Pose2D p) {return Json::array({p.position.x,p.position.y,p.heading});}
Json trajectory_json(const ca::TimedTrajectory& trajectory,size_t stride) {
  Json j=Json::array();for(size_t i=0;i<trajectory.size();++i)if(i%stride==0||i+1==trajectory.size()) {
    const auto& p=trajectory[i];j.push_back({p.time,p.pose.position.x,p.pose.position.y,p.pose.heading,p.velocity.linear,p.velocity.angular});
  }return j;
}
Json planner_config_json(const ca::PlannerConfig& c) {
  Json j;
#define PARAM(name) j[#name]=c.name
  PARAM(horizon);PARAM(prediction_step);PARAM(rollout_step);PARAM(clearance);PARAM(tracking_margin);PARAM(intent_max_age);PARAM(tracking_recovery_time);
  PARAM(uncertainty_growth);PARAM(transition_length);PARAM(passing_length);PARAM(return_clearance);PARAM(lookahead);
  PARAM(goal_tolerance);PARAM(commitment_time);PARAM(recovery_time);PARAM(collision_resolution);PARAM(offsets);PARAM(speed_scales);PARAM(transition_scales);
#undef PARAM
#define WEIGHT(name) j["weights"][#name]=c.weights.name
  WEIGHT(progress);WEIGHT(deviation);WEIGHT(speed_reduction);WEIGHT(smoothness);WEIGHT(clearance);WEIGHT(left_side);WEIGHT(switching);
#undef WEIGHT
  return j;
}
} // namespace avoidance_sim
