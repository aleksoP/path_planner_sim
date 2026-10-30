#include "avoidance_sim/engine.hpp"
#include "avoidance_sim/controller.hpp"
#include "avoidance_sim/observations.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include "cooperative_avoidance/trajectory.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <optional>
#include <thread>
namespace avoidance_sim {
namespace {
using Clock=std::chrono::steady_clock;
struct RobotMetrics {
  double distance{0},deviation_integral{0},max_deviation{0},speed_integral{0},low_speed_time{0};
  double max_angular{0},max_acceleration{0},max_deceleration{0},max_angular_acceleration{0},max_curvature{0};
  int stops{0},avoidances{0},side_reversals{0},saturations{0},replans{0};
  bool moving{false};ca::PassingSide last_side{ca::PassingSide::None};
  double goal_time{-1},last_progress_time{0};double last_progress{0};
};
Json polygon_json(const ca::Polygon& polygon) {Json j=Json::array();for(auto p:polygon)j.push_back({p.x,p.y});return j;}
ca::Polygon event_polygon(const Json& j) {
  if(j.contains("polygon")){ca::Polygon p;for(auto v:j["polygon"])p.push_back({v[0],v[1]});return p;}
  auto p=j.at("pose"),s=j.at("size");return ca::rectangle({{p[0],p[1]},p[2]},{s[0],s[1]});
}
}
SimulationResult Engine::run(const Scenario& s,const RunOptions& options) const {
  SimulationResult out;TrajectoryFollower follower;
  std::vector<ca::RobotState> robots;std::vector<ca::CooperativePlanner> planners;
  std::vector<ca::PlannerResult> plans(s.robots.size());
  std::vector<double> plan_times(s.robots.size(),0),loss_until(s.robots.size(),-1);
  std::vector<ca::Twist2D> commands(s.robots.size());
  std::vector<RobotMetrics> metrics(s.robots.size());
  std::vector<bool> forced_stop(s.robots.size(),false),event_applied(s.events.size(),false);
  auto statics=s.obstacles;
  std::vector<std::vector<std::optional<ca::ObstacleState>>> tracks(s.robots.size(),
    std::vector<std::optional<ca::ObstacleState>>(s.robots.size()));
  for(const auto& r:s.robots){robots.push_back(r.initial);planners.emplace_back(s.planner);}
  const size_t planner_ticks=std::llround(s.planner_period/s.timestep),control_ticks=std::llround(s.control_period/s.timestep);
  const size_t record_ticks=std::llround(s.record_period/s.timestep),ticks=std::floor(s.duration/s.timestep+1e-8);
  double time=0,min_separation=std::numeric_limits<double>::infinity();
  int collisions=0,violations=0,near_misses=0,predicted=0,no_feasible=0;
  size_t calls=0,evaluated=0;double planning_sum=0,planning_max=0;
  Json rejections=Json::object(),collision_events=Json::array();std::set<std::string> collision_contacts,clearance_contacts,near_contacts;
  auto add_event=[&](Json j){if(options.record)out.events.push_back(std::move(j));};
  auto inspect=[&](double at) {
    std::set<std::string> current_collisions,current_clearance,current_near;
    auto pair=[&](const ca::Polygon& a,const ca::Polygon& b,const std::string& key,ca::Vector2 location) {
      double d=ca::polygon_separation(a,b);min_separation=std::min(min_separation,d);
      if(ca::polygons_collide(a,b)) {
        current_collisions.insert(key);
        if(!collision_contacts.count(key)) {++collisions;
          Json event={{"time",at},{"type","collision"},{"pair",key},{"location",{location.x,location.y}}};
          collision_events.push_back(event);add_event(std::move(event));}
      }
      if(d<s.planner.clearance-1e-8) {
        current_clearance.insert(key);if(!clearance_contacts.count(key)){++violations;add_event({{"time",at},{"type","clearance_violation"},{"pair",key},{"separation",d}});}
      } else if(d<s.planner.clearance+s.planner.tracking_margin) {
        current_near.insert(key);if(!near_contacts.count(key)){++near_misses;add_event({{"time",at},{"type","near_miss"},{"pair",key},{"separation",d}});}
      }
    };
    for(size_t i=0;i<robots.size();++i) {
      const auto a=ca::rectangle(robots[i].pose,robots[i].footprint);
      for(size_t j=0;j<statics.size();++j)pair(a,statics[j],robots[i].id+":static_"+std::to_string(j),robots[i].pose.position);
      for(size_t j=i+1;j<robots.size();++j)pair(a,ca::rectangle(robots[j].pose,robots[j].footprint),robots[i].id+":"+robots[j].id,
                                          (robots[i].pose.position+robots[j].pose.position)/2);
    }
    collision_contacts=std::move(current_collisions);clearance_contacts=std::move(current_clearance);near_contacts=std::move(current_near);
  };
  auto frame=[&](){
    if(!options.record)return;
    Json f={{"time",time},{"minimum_separation_so_far",min_separation},{"collisions",collisions},{"robots",Json::array()}};
    for(size_t i=0;i<robots.size();++i)f["robots"].push_back({{"id",robots[i].id},{"pose",pose_json(robots[i].pose)},
      {"velocity",{robots[i].velocity.linear,robots[i].velocity.angular}},{"command",{commands[i].linear,commands[i].angular}},
      {"state",ca::to_string(plans[i].state)},{"status",ca::to_string(plans[i].status)},
      {"side",ca::to_string(plans[i].side)},{"maneuver",ca::to_string(plans[i].maneuver)},
      {"distance",metrics[i].distance},{"goal_reached",metrics[i].goal_time>=0}});
    out.trace.push_back(std::move(f));
  };
  inspect(0);const auto wall_start=Clock::now();
  for(size_t tick=0;tick<=ticks;++tick) {
    time=tick*s.timestep;
    for(size_t k=0;k<s.events.size();++k) if(!event_applied[k]&&time+1e-9>=s.events[k].at("time").get<double>()) {
      const auto& e=s.events[k];event_applied[k]=true;std::string type=e.at("type");
      if(type=="add_obstacle")statics.push_back(event_polygon(e.at("obstacle")));
      else for(size_t i=0;i<robots.size();++i) if(robots[i].id==e.at("robot")) {
        if(type=="stop_robot")forced_stop[i]=true;
        if(type=="communication_loss")loss_until[i]=time+e.at("duration").get<double>();
      }
      auto logged=e;logged["applied_time"]=time;add_event(logged);inspect(time);
    }
    if(tick%planner_ticks==0) {
      // Freeze state AND intentions. Publish all new plans only after the entire planning round.
      const auto snapshot=robots;const auto previous_plans=plans;const auto previous_times=plan_times;
      std::vector<ca::PlannerResult> next=plans;
      for(size_t i=0;i<robots.size();++i) {
        if(metrics[i].goal_time>=0)continue;
        ca::PlannerRequest request{time,snapshot[i],s.robots[i].path,s.robots[i].goal,statics,{}};
        Json observation_log=Json::array();
        for(size_t j=0;j<robots.size();++j)if(i!=j) {
          bool visible=observable(snapshot[i],snapshot[j],statics,s.observation_range,
                                  s.observation_field_of_view,s.observation_line_of_sight);
          if(visible) {
          ca::ObstacleState o;o.id=snapshot[j].id;o.timestamp=time;o.pose=snapshot[j].pose;o.footprint=snapshot[j].footprint;
          o.velocity=ca::Vector2{std::cos(o.pose.heading),std::sin(o.pose.heading)}*snapshot[j].velocity.linear;
          o.angular_velocity=snapshot[j].velocity.angular;o.cooperative=s.robots[j].cooperative;
          if(time>=loss_until[j]&&o.cooperative) {o.intention=previous_plans[j].trajectory;o.intention_timestamp=previous_times[j];}
          tracks[i][j]=std::move(o);
          }
          const auto& tracked=tracks[i][j];
          if(tracked&&time-tracked->timestamp<=s.observation_memory+1e-9) {
            request.obstacles.push_back(*tracked);
            if(options.record)observation_log.push_back({{"id",tracked->id},{"timestamp",tracked->timestamp},{"visible",visible},
              {"pose",pose_json(tracked->pose)},{"velocity",{tracked->velocity.x,tracked->velocity.y}}});
          }
        }
        auto before=Clock::now();next[i]=planners[i].plan(request);
        double elapsed=std::chrono::duration<double>(Clock::now()-before).count();
        planning_sum+=elapsed;planning_max=std::max(planning_max,elapsed);++calls;
        auto& p=next[i];
        if(forced_stop[i]) {
          p.trajectory=ca::rollout(robots[i],s.robots[i].path,{},0,s.planner);
          p.status=ca::PlanningStatus::Yielding;p.maneuver=ca::Maneuver::Stop;p.state=ca::EncounterState::Yielding;p.reason="Scenario event: commanded bounded stop";
        }
        if(p.predicted_conflict)++predicted;
        if(p.status==ca::PlanningStatus::NoFeasibleManeuver)++no_feasible;
        if(p.maneuver_replanned)++metrics[i].replans;
        evaluated+=p.candidates.size();
        for(const auto& c:p.candidates)if(!c.feasible){if(!rejections.contains(c.reason))rejections[c.reason]=0;rejections[c.reason]=rejections[c.reason].get<int>()+1;}
        for(auto state:p.transitions)if(state==ca::EncounterState::ManeuverSelected)++metrics[i].avoidances;
        if(p.side!=ca::PassingSide::None){
          if(metrics[i].last_side!=ca::PassingSide::None&&metrics[i].last_side!=p.side)++metrics[i].side_reversals;
          metrics[i].last_side=p.side;
        }
        if(options.record) {
          Json e={{"time",time},{"type","plan"},{"robot",robots[i].id},{"state",ca::to_string(p.state)},
            {"status",ca::to_string(p.status)},{"side",ca::to_string(p.side)},{"maneuver",ca::to_string(p.maneuver)},
            {"reason",p.reason},{"predicted_conflict",p.predicted_conflict},{"minimum_predicted_separation",p.min_predicted_separation},
            {"trajectory",trajectory_json(p.trajectory,3)},{"transitions",Json::array()},{"candidates",Json::array()},{"predictions",Json::array()}};
          e["observations"]=std::move(observation_log);
          e["robot_pose"]=pose_json(request.robot.pose);
          e["maneuver_replanned"]=p.maneuver_replanned;e["rejoin_s"]=p.rejoin_s;e["maneuver_path"]=Json::array();
          for(auto point:p.maneuver_path)e["maneuver_path"].push_back({point.x,point.y});
          for(auto state:p.transitions)e["transitions"].push_back(ca::to_string(state));
          for(const auto& c:p.candidates) {
            Json diagnostic={{"maneuver",ca::to_string(c.maneuver)},{"side",ca::to_string(c.side)},
              {"offset",c.offset},{"speed_scale",c.speed_scale},{"feasible",c.feasible},{"reason",c.reason},
              {"cost",c.cost},{"minimum_separation",c.min_separation}};
            if(options.debug)diagnostic["trajectory"]=trajectory_json(c.trajectory,4);
            e["candidates"].push_back(std::move(diagnostic));
          }
          for(const auto& prediction:p.predictions)e["predictions"].push_back({{"id",prediction.id},
            {"used_intention",prediction.used_intention},{"uncertainty",prediction.uncertainty},
            {"initial_uncertainty",prediction.initial_uncertainty},{"uncertainty_growth",prediction.uncertainty_growth},
            {"trajectory",trajectory_json(prediction.trajectory,2)}});
          add_event(std::move(e));
        }
      }
      plans=std::move(next);for(size_t i=0;i<robots.size();++i)if(metrics[i].goal_time<0)plan_times[i]=time;
    }
    if(tick%control_ticks==0)for(size_t i=0;i<robots.size();++i)commands[i]=metrics[i].goal_time>=0||forced_stop[i]?
      ca::Twist2D{}:follower.command(robots[i],plans[i].trajectory,time-plan_times[i],s.control_period);
    if(tick%record_ticks==0)frame();
    bool complete=true;for(const auto& m:metrics)if(m.goal_time<0)complete=false;
    if(complete||tick==ticks)break;
    // Bound every footprint point's travel per substep, including rotational motion.
    double point_speed=0;for(const auto& r:robots)point_speed=std::max(point_speed,r.limits.max_forward+
      r.limits.max_angular*std::hypot(r.footprint.length,r.footprint.width)/2);
    const size_t substeps=std::max<size_t>(1,std::ceil(point_speed*s.timestep/0.01));
    const double dt=s.timestep/substeps;
    for(size_t sub=0;sub<substeps;++sub) {
      for(size_t i=0;i<robots.size();++i) {
        auto previous=robots[i];auto step=model_->step(previous,commands[i],dt);robots[i]=step.state;
        auto& m=metrics[i];const auto& r=robots[i];
        if(step.saturated)++m.saturations;
        double distance=ca::norm(r.pose.position-previous.pose.position);m.distance+=distance;
        auto projection=s.robots[i].path.project(r.pose.position);
        m.deviation_integral+=projection.distance*dt;m.max_deviation=std::max(m.max_deviation,projection.distance);
        m.speed_integral+=std::abs(r.velocity.linear)*dt;
        if(std::abs(r.velocity.linear)<0.1)m.low_speed_time+=dt;
        if(std::abs(r.velocity.linear)>0.15)m.moving=true;
        if(m.moving&&std::abs(r.velocity.linear)<0.03){++m.stops;m.moving=false;}
        m.max_angular=std::max(m.max_angular,std::abs(r.velocity.angular));
        const double a=(r.velocity.linear-previous.velocity.linear)/dt;
        m.max_acceleration=std::max(m.max_acceleration,a);m.max_deceleration=std::max(m.max_deceleration,-a);
        m.max_angular_acceleration=std::max(m.max_angular_acceleration,std::abs(r.velocity.angular-previous.velocity.angular)/dt);
        if(std::abs(r.velocity.linear)>0.05)m.max_curvature=std::max(m.max_curvature,std::abs(r.velocity.angular/r.velocity.linear));
        if(projection.s>m.last_progress+0.05){m.last_progress=projection.s;m.last_progress_time=time+(sub+1)*dt;}
        if(m.goal_time<0&&ca::norm(r.pose.position-s.robots[i].goal.position)<=s.planner.goal_tolerance&&
           std::abs(r.velocity.linear)<0.05&&std::abs(ca::wrap_angle(r.pose.heading-s.robots[i].goal.heading))<0.12) {
          m.goal_time=time+(sub+1)*dt;add_event({{"time",m.goal_time},{"type","goal"},{"robot",r.id}});
        }
      }
      inspect(time+(sub+1)*dt);
    }
    if(options.realtime)std::this_thread::sleep_until(wall_start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((tick+1)*s.timestep)));
  }
  if(options.record&&(out.trace.empty()||std::abs(out.trace.back()["time"].get<double>()-time)>1e-8))frame();
  int goals=0;Json robot_metrics=Json::array();
  for(size_t i=0;i<robots.size();++i) {
    const auto& m=metrics[i];if(m.goal_time>=0)++goals;
    robot_metrics.push_back({{"id",robots[i].id},{"goal_reached",m.goal_time>=0},{"goal_time",m.goal_time<0?Json(nullptr):Json(m.goal_time)},
      {"distance",m.distance},{"average_speed",m.speed_integral/std::max(time,1e-9)},
      {"mean_path_deviation",m.deviation_integral/std::max(time,1e-9)},{"max_path_deviation",m.max_deviation},
      {"low_speed_time",m.low_speed_time},{"full_stops",m.stops},{"avoidance_maneuvers",m.avoidances},
      {"passing_side_reversals",m.side_reversals},{"max_angular_velocity",m.max_angular},
      {"maneuver_replans",m.replans},
      {"max_linear_acceleration",m.max_acceleration},{"max_linear_deceleration",m.max_deceleration},
      {"max_angular_acceleration",m.max_angular_acceleration},{"max_curvature",m.max_curvature},
      {"command_saturation_substeps",m.saturations},{"final_pose",pose_json(robots[i].pose)},
      {"stalled",m.goal_time<0&&time-m.last_progress_time>5}});
  }
  std::string outcome=collisions?"collision":goals==static_cast<int>(robots.size())?"completed":"unresolved_timeout";
  out.metrics={{"schema","cooperative-avoidance/metrics/v1"},{"scenario",s.name},{"seed",options.seed},{"outcome",outcome},
    {"simulation_time",time},{"goal_completion",goals},{"robot_count",robots.size()},
    {"collisions",collisions},{"collision_events",collision_events},{"minimum_geometric_separation",min_separation},{"clearance_violations",violations},
    {"near_misses",near_misses},{"predicted_conflict_updates",predicted},{"no_feasible_updates",no_feasible},
    {"timeout_events",outcome=="unresolved_timeout"?1:0},{"robots",robot_metrics},
    {"planner",{{"calls",calls},{"candidates_evaluated",evaluated},{"rejection_reasons",rejections},
      {"configured_frequency_hz",1/s.planner_period},{"effective_calls_per_robot_second",calls/std::max(1e-9,time*robots.size())},
      {"average_wall_seconds",calls?planning_sum/calls:0},
      {"maximum_wall_seconds",planning_max},{"total_wall_seconds",planning_sum}}},
    {"execution_wall_seconds",std::chrono::duration<double>(Clock::now()-wall_start).count()}};
  add_event({{"time",time},{"type","outcome"},{"outcome",outcome}});
  return out;
}
void write_run(const std::string& directory,const Scenario& s,const RunOptions& options,const SimulationResult& result) {
  std::filesystem::create_directories(directory);
  // Refuse to replace a prior recording accidentally; select a fresh run directory.
  for(const auto* file:{"run_config.json","metrics.json","trace.jsonl","events.jsonl"})
    if(std::filesystem::exists(std::filesystem::path(directory)/file))throw std::runtime_error("output already contains a run: "+directory);
  auto config=s.source;config["schema"]="cooperative-avoidance/run/v1";config["seed"]=options.seed;
  config["planner"]=planner_config_json(s.planner);config["simulation"]={{"timestep",s.timestep},{"planner_period",s.planner_period},
    {"control_period",s.control_period},{"duration",s.duration},{"record_period",s.record_period}};
  config["recording"]=options.record;config["debug"]=options.debug;config["world_polygons"]=Json::array();
  for(const auto& polygon:s.obstacles)config["world_polygons"].push_back(polygon_json(polygon));
  config["effective_paths"]=Json::object();
  for(const auto& robot:s.robots){Json points=Json::array();for(auto p:robot.path.points())points.push_back({p.x,p.y});config["effective_paths"][robot.initial.id]=points;}
  for(auto& definition:config["robots"]) for(const auto& r:s.robots) if(definition["id"]==r.initial.id) {
    const auto& l=r.initial.limits;
    definition["limits"]={{"max_forward",l.max_forward},{"max_reverse",l.max_reverse},{"max_angular",l.max_angular},
      {"max_curvature",l.max_curvature},{"acceleration",l.acceleration.acceleration},{"deceleration",l.acceleration.deceleration},
      {"angular_acceleration",l.acceleration.angular_acceleration}};
    definition["cooperative"]=r.cooperative;definition["velocity"]={r.initial.velocity.linear,r.initial.velocity.angular};
  }
  config["observations"]={{"ground_truth",s.ground_truth_observations},{"range",s.observation_range},
    {"line_of_sight",s.observation_line_of_sight},{"field_of_view_degrees",s.observation_field_of_view*180/ca::pi},
    {"memory",s.observation_memory}};
  auto save=[&](const std::string& name,const Json& j){std::ofstream f(std::filesystem::path(directory)/name);f.exceptions(std::ios::failbit|std::ios::badbit);f<<j.dump(2)<<'\n';};
  save("run_config.json",config);save("metrics.json",result.metrics);
  for(const auto* name:{"trace.jsonl","events.jsonl"}){
    std::ofstream f(std::filesystem::path(directory)/name);f.exceptions(std::ios::failbit|std::ios::badbit);
    for(const auto& record:std::string(name)=="trace.jsonl"?result.trace:result.events)f<<record.dump()<<'\n';
  }
}
} // namespace avoidance_sim
