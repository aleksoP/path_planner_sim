#include <cooperative_avoidance/cooperative_planner.hpp>
#include <iostream>
int main() {
  namespace ca=cooperative_avoidance;
  ca::RobotState robot;robot.id="external_client";
  ca::ReferencePath path({{0,0},{10,0}});
  ca::CooperativePlanner planner;
  ca::PlannerRequest request{0,robot,path,{{10,0},0},{},{}};
  const auto result=planner.plan(request);
  std::cout<<ca::to_string(result.status)<<": "<<result.trajectory.size()<<" timed points\n";
  return result.status==ca::PlanningStatus::Nominal?0:1;
}
