#pragma once
#include "path_deformation.hpp"
#include "planner_interface.hpp"
#include <map>
namespace cooperative_avoidance {
// One instance per robot. No external world state or mutable globals.
class CooperativePlanner final : public PlannerInterface {
 public:
  explicit CooperativePlanner(PlannerConfig config={});
  PlannerResult plan(const PlannerRequest&) override;
  void reset();
 private:
  PlannerConfig config_;
  Deformation committed_;
  PassingSide side_{PassingSide::None};
  EncounterState state_{EncounterState::Normal};
  double selected_at_{0}, last_conflict_{0}, progress_{0};
  std::string blocked_encounter_;
  std::map<std::string,ObstacleState> encounter_agents_;
  double static_clear_s_{-1};
};
void validate_config(const PlannerConfig&);
} // namespace cooperative_avoidance
