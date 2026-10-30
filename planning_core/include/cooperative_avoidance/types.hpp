#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace cooperative_avoidance {
constexpr double pi = 3.14159265358979323846;
// World frame: x/y right handed, yaw CCW. All quantities are SI.
struct Vector2 {
  double x{0}, y{0};
  Vector2 operator+(Vector2 b) const { return {x+b.x,y+b.y}; }
  Vector2 operator-(Vector2 b) const { return {x-b.x,y-b.y}; }
  Vector2 operator*(double k) const { return {x*k,y*k}; }
  Vector2 operator/(double k) const { return {x/k,y/k}; }
};
inline double dot(Vector2 a, Vector2 b) { return a.x*b.x+a.y*b.y; }
inline double cross(Vector2 a, Vector2 b) { return a.x*b.y-a.y*b.x; }
inline double norm(Vector2 a) { return std::hypot(a.x,a.y); }
inline Vector2 unit(Vector2 a) { const auto n=norm(a); return n>1e-12?a/n:Vector2{1,0}; }
struct Pose2D { Vector2 position; double heading{0}; };
struct Twist2D { double linear{0}, angular{0}; };
struct AccelerationLimits { double acceleration{0.8}, deceleration{1.2}, angular_acceleration{2.0}; };
struct MotionLimits {
  double max_forward{1.5}, max_reverse{0.3}, max_angular{1.5};
  double max_curvature{2.5};
  AccelerationLimits acceleration;
};
struct RobotFootprint { double length{0.8}, width{0.6}; };
struct RobotState {
  std::string id;
  Pose2D pose;
  Twist2D velocity;
  RobotFootprint footprint;
  MotionLimits limits;
};
using Polygon = std::vector<Vector2>;
struct TrajectoryPoint { double time{0}; Pose2D pose; Twist2D velocity; };
// Times relative to this trajectory's epoch, never wall-clock timestamps.
using TimedTrajectory = std::vector<TrajectoryPoint>;
struct ObstacleState {
  std::string id;
  double timestamp{0};
  Pose2D pose;
  Vector2 velocity;
  double angular_velocity{0};
  RobotFootprint footprint;
  double confidence{1}, uncertainty{0}; // uncertainty radius in metres
  bool cooperative{false};
  double intention_timestamp{0};
  TimedTrajectory intention;
};
struct PredictedObstacleTrajectory {
  std::string id;
  RobotFootprint footprint;
  double uncertainty{0};
  double initial_uncertainty{0}, uncertainty_growth{0};
  bool used_intention{false};
  TimedTrajectory trajectory;
};
struct CostWeights {
  double progress{4}, deviation{0.4}, speed_reduction{1}, smoothness{0.1};
  double clearance{0.05}, left_side{0.8}, switching{5};
};
struct PlannerConfig {
  double horizon{5}, prediction_step{0.1}, rollout_step{0.04};
  double clearance{0.15}, tracking_margin{0.04}, intent_max_age{0.3};
  double tracking_recovery_time{0.5};
  double uncertainty_growth{0.02}, transition_length{3.2}, passing_length{1.0};
  double return_clearance{0.8}, lookahead{0.6}, goal_tolerance{0.10};
  double commitment_time{1.0}, recovery_time{0.5};
  double collision_resolution{0.025};
  std::vector<double> transition_scales{1.0,0.5};
  std::vector<double> offsets{0.85,0.95,1.1};
  std::vector<double> speed_scales{1.0,0.65,0.3};
  CostWeights weights;
};
enum class PassingSide { None, Right, Left };
enum class Maneuver { Nominal, Offset, Slow, Stop };
enum class PlanningStatus { Nominal, Avoidance, Yielding, NoFeasibleManeuver };
enum class EncounterState { Normal, ConflictDetected, ManeuverSelected, Shifting, Passing, Returning, Yielding, Recovery };
const char* to_string(PassingSide);
const char* to_string(Maneuver);
const char* to_string(PlanningStatus);
const char* to_string(EncounterState);
struct CandidateDiagnostic {
  PassingSide side{PassingSide::None};
  Maneuver maneuver{Maneuver::Nominal};
  double offset{0}, speed_scale{1}, cost{0};
  double min_separation{std::numeric_limits<double>::infinity()};
  bool feasible{false};
  std::string reason;
  TimedTrajectory trajectory;
};
struct PlannerResult {
  PlanningStatus status{PlanningStatus::NoFeasibleManeuver};
  TimedTrajectory trajectory;
  PassingSide side{PassingSide::None};
  Maneuver maneuver{Maneuver::Stop};
  EncounterState state{EncounterState::Normal};
  double min_predicted_separation{std::numeric_limits<double>::infinity()};
  bool predicted_conflict{false};
  std::string reason;
  std::vector<CandidateDiagnostic> candidates;
  std::vector<PredictedObstacleTrajectory> predictions;
  std::vector<EncounterState> transitions;
  std::vector<Vector2> maneuver_path; // spatial intent including rejoin, beyond the timed horizon if necessary
  double rejoin_s{-1};
  bool maneuver_replanned{false};
};
} // namespace cooperative_avoidance
