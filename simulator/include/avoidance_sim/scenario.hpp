#pragma once
#include "cooperative_avoidance/cooperative_planner.hpp"
#include <nlohmann/json.hpp>
#include <string>
namespace avoidance_sim {
namespace ca=cooperative_avoidance;
using Json=nlohmann::json;
struct RobotDefinition {
  ca::RobotState initial;
  ca::ReferencePath path;
  ca::Pose2D goal;
  bool cooperative{true};
};
struct Scenario {
  std::string name;
  double timestep{0.02}, planner_period{0.1}, control_period{0.02}, duration{35}, record_period{0.1};
  bool ground_truth_observations{true};
  double observation_range{30};
  bool observation_line_of_sight{true};
  double observation_field_of_view{2*ca::pi}, observation_memory{0.5};
  ca::PlannerConfig planner;
  std::vector<ca::Polygon> obstacles;
  std::vector<RobotDefinition> robots;
  Json events=Json::array();
  Json source;
};
Scenario load_scenario(const std::string& filename,const std::string& planner_override="");
Json planner_config_json(const ca::PlannerConfig&);
Json pose_json(ca::Pose2D);
Json trajectory_json(const ca::TimedTrajectory&,size_t stride=1);
} // namespace avoidance_sim
