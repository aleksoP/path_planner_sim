#define BOOST_TEST_MODULE cooperative_avoidance_integration
#include <boost/test/included/unit_test.hpp>
#include "avoidance_sim/engine.hpp"
#include "avoidance_sim/observations.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include <map>
using namespace avoidance_sim;
namespace {
SimulationResult run(const std::string& name,bool record=false) {
  return Engine{}.run(load_scenario(std::string(SCENARIO_DIR)+"/"+name+".json"),{42,record,false,false});
}
void safe_complete(const SimulationResult& r,int goals) {
  BOOST_CHECK_EQUAL(r.metrics["collisions"].get<int>(),0);
  BOOST_CHECK_EQUAL(r.metrics["clearance_violations"].get<int>(),0);
  BOOST_CHECK_EQUAL(r.metrics["goal_completion"].get<int>(),goals);
  BOOST_CHECK_EQUAL(r.metrics["outcome"].get<std::string>(),"completed");
  for(const auto& robot:r.metrics["robots"]) {
    BOOST_CHECK(robot["max_linear_acceleration"].get<double>()<=.8+1e-7);
    BOOST_CHECK(robot["max_linear_deceleration"].get<double>()<=1.2+1e-7);
    BOOST_CHECK(robot["max_angular_acceleration"].get<double>()<=2+1e-7);
    BOOST_CHECK(robot["max_angular_velocity"].get<double>()<=1.5+1e-7);
    BOOST_CHECK_EQUAL(robot["passing_side_reversals"].get<int>(),0);
  }
}
}
BOOST_AUTO_TEST_CASE(empty_corridor) {
  auto r=run("empty_corridor");safe_complete(r,1);
  BOOST_CHECK_SMALL(r.metrics["robots"][0]["max_path_deviation"].get<double>(),1e-6);
  BOOST_CHECK_EQUAL(r.metrics["robots"][0]["avoidance_maneuvers"].get<int>(),0);
}
BOOST_AUTO_TEST_CASE(completed_head_on_passing_and_reproducibility) {
  auto a=run("head_on_wide",true);safe_complete(a,2);
  double amin=0,bmax=0;bool swapped=false;
  for(const auto& frame:a.trace) {
    amin=std::min(amin,frame["robots"][0]["pose"][1].get<double>());
    bmax=std::max(bmax,frame["robots"][1]["pose"][1].get<double>());
    if(frame["robots"][0]["pose"][0].get<double>()>frame["robots"][1]["pose"][0].get<double>())swapped=true;
  }
  BOOST_CHECK(amin<-.7);BOOST_CHECK(bmax>.7);BOOST_CHECK(swapped);
  for(const auto& r:a.metrics["robots"]) {BOOST_CHECK(std::abs(r["final_pose"][1].get<double>())<.1);BOOST_CHECK(r["avoidance_maneuvers"].get<int>()>=1);}
  auto b=run("head_on_wide",true);BOOST_CHECK(a.trace==b.trace);BOOST_CHECK(a.events==b.events);
  auto scenario=load_scenario(std::string(SCENARIO_DIR)+"/head_on_wide.json");
  std::reverse(scenario.robots.begin(),scenario.robots.end());
  auto reverse=Engine{}.run(scenario,{42,false,false,false});safe_complete(reverse,2);
  BOOST_CHECK_SMALL(reverse.metrics["simulation_time"].get<double>()-a.metrics["simulation_time"].get<double>(),1e-9);
}
BOOST_AUTO_TEST_CASE(narrow_is_safely_unresolved) {
  auto r=run("head_on_narrow",true);BOOST_CHECK_EQUAL(r.metrics["collisions"].get<int>(),0);
  BOOST_CHECK_EQUAL(r.metrics["clearance_violations"].get<int>(),0);
  BOOST_CHECK_EQUAL(r.metrics["outcome"].get<std::string>(),"unresolved_timeout");
  BOOST_CHECK_EQUAL(r.metrics["goal_completion"].get<int>(),0);
  for(const auto& robot:r.trace.back()["robots"])BOOST_CHECK_SMALL(robot["velocity"][0].get<double>(),.05);
}
BOOST_AUTO_TEST_CASE(static_both_sides) {safe_complete(run("static_obstacle_left"),1);safe_complete(run("static_obstacle_right"),1);}
BOOST_AUTO_TEST_CASE(crossing) {safe_complete(run("crossing"),2);}
BOOST_AUTO_TEST_CASE(l_shaped_corridor_completes_turn) {
  auto r=run("l_shaped_corridor",true);safe_complete(r,1);
  bool horizontal=false,vertical=false;
  for(const auto& frame:r.trace) {
    const auto& pose=frame["robots"][0]["pose"];
    const double x=pose[0].get<double>(),y=pose[1].get<double>(),yaw=pose[2].get<double>();
    if(x<-3&&std::abs(y)<.1&&std::abs(yaw)<.1)horizontal=true;
    if(y>3&&std::abs(x)<.1&&std::abs(yaw-ca::pi/2)<.1)vertical=true;
  }
  BOOST_CHECK(horizontal);BOOST_CHECK(vertical);
  const auto& robot=r.metrics["robots"][0];
  BOOST_CHECK_SMALL(robot["final_pose"][0].get<double>(),.1);
  BOOST_CHECK_SMALL(robot["final_pose"][1].get<double>()-8,.1);
  BOOST_CHECK_SMALL(robot["final_pose"][2].get<double>()-ca::pi/2,.12);
  BOOST_CHECK(robot["max_path_deviation"].get<double>()<.1);
  BOOST_CHECK(robot["max_curvature"].get<double>()<=2.5+1e-7);
  BOOST_CHECK_EQUAL(robot["avoidance_maneuvers"].get<int>(),0);
}
BOOST_AUTO_TEST_CASE(opposing_l_shaped_robots_replan_after_observation_and_rejoin) {
  auto r=run("l_shaped_head_on",true);safe_complete(r,2);
  BOOST_CHECK_EQUAL(r.metrics["timeout_events"].get<int>(),0);
  BOOST_CHECK(r.metrics["predicted_conflict_updates"].get<int>()>0);
  BOOST_CHECK(r.metrics["robots"][0]["maneuver_replans"].get<int>()>0);
  BOOST_CHECK_SMALL(r.metrics["robots"][0]["final_pose"][0].get<double>(),.1);
  BOOST_CHECK_SMALL(r.metrics["robots"][1]["final_pose"][1].get<double>(),.1);
  auto scenario=load_scenario(std::string(SCENARIO_DIR)+"/l_shaped_head_on.json");
  std::map<std::string,bool> observed;bool returned=false;
  for(const auto& event:r.events)if(event.at("type")=="plan") {
    const std::string id=event.at("robot");
    if(!event["observations"].empty())observed[id]=true;
    if(event["maneuver"]=="Offset")BOOST_CHECK(observed[id]);
    if(event["time"].get<double>()<3) {
      BOOST_CHECK(event["observations"].empty());
      BOOST_CHECK(event["predictions"].empty());
      BOOST_CHECK_EQUAL(event["maneuver"].get<std::string>(),"Nominal");
    }
    if(!event["maneuver_path"].empty()) {
      const auto& last=event["maneuver_path"].back();
      const auto& definition=scenario.robots[id=="A"?0:1];
      BOOST_CHECK_SMALL(definition.path.project({last[0],last[1]}).distance,1e-6);
    }
    if(event["state"]=="Returning") {
      if(!returned) {
        const auto& definition=scenario.robots[id=="A"?0:1];
        const auto& raw=event["robot_pose"];
        ca::Pose2D own{{raw[0],raw[1]},raw[2]};
        double rear=definition.path.length();
        for(auto corner:ca::rectangle(own,definition.initial.footprint))rear=std::min(rear,definition.path.project(corner).s);
        BOOST_REQUIRE(!event["observations"].empty());
        for(const auto& observation:event["observations"]) {
          const auto& p=observation["pose"];ca::Pose2D other{{p[0],p[1]},p[2]};
          double front=0;
          for(auto corner:ca::rectangle(other,scenario.robots[id=="A"?1:0].initial.footprint))
            front=std::max(front,definition.path.project(corner).s);
          BOOST_CHECK(rear>front+scenario.planner.return_clearance);
        }
      }
      returned=true;
    }
  }
  BOOST_CHECK(returned);
  for(const auto& robot:r.metrics["robots"])BOOST_CHECK(robot["max_curvature"].get<double>()<=2.5+1e-7);
}
BOOST_AUTO_TEST_CASE(observations_obey_occlusion_field_of_view_and_range) {
  ca::RobotState observer,target;target.pose.position={4,0};
  auto wall=ca::rectangle({{2,0},0},{.2,2});
  BOOST_CHECK(!observable(observer,target,{wall},10,2*ca::pi,true));
  BOOST_CHECK(observable(observer,target,{wall},10,2*ca::pi,false));
  BOOST_CHECK(!observable(observer,target,{},2,2*ca::pi,true));
  target.pose.position={-4,0};
  BOOST_CHECK(!observable(observer,target,{},10,ca::pi/2,true));
  BOOST_CHECK(observable(observer,target,{},10,2*ca::pi,true));
  target.pose.position={4,.9};wall=ca::rectangle({{2,0},0},{.2,1});
  BOOST_CHECK(observable(observer,target,{wall},10,2*ca::pi,true)); // partially visible footprint
  auto scenario=load_scenario(std::string(SCENARIO_DIR)+"/l_shaped_head_on.json");
  BOOST_CHECK(!observable(scenario.robots[0].initial,scenario.robots[1].initial,scenario.obstacles,30,2*ca::pi,true));
  std::reverse(wall.begin(),wall.end());target.pose.position={4,0};
  BOOST_CHECK(!observable(observer,target,{wall},10,2*ca::pi,true));
}
