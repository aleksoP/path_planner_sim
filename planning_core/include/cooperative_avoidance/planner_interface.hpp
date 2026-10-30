#pragma once
#include "reference_path.hpp"
namespace cooperative_avoidance {
struct PlannerRequest {
  double timestamp{0};
  RobotState robot;
  const ReferencePath& reference_path;
  Pose2D goal;
  std::vector<Polygon> static_obstacles;
  std::vector<ObstacleState> obstacles;
};
class PlannerInterface {
 public:
  virtual ~PlannerInterface()=default;
  virtual PlannerResult plan(const PlannerRequest&)=0;
};
} // namespace cooperative_avoidance
