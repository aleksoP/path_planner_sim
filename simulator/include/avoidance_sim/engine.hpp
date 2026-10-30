#pragma once
#include "scenario.hpp"
#include "cooperative_avoidance/motion_model.hpp"
#include <memory>
namespace avoidance_sim {
struct RunOptions { unsigned seed{42}; bool record{true}, debug{false}, realtime{false}; };
struct SimulationResult { Json metrics; std::vector<Json> trace,events; };
class Engine {
 public:
  explicit Engine(std::shared_ptr<const ca::MotionModel> model=std::make_shared<ca::UnicycleModel>()):model_(std::move(model)){}
  SimulationResult run(const Scenario&,const RunOptions& options={}) const;
 private:
  std::shared_ptr<const ca::MotionModel> model_;
};
void write_run(const std::string& directory,const Scenario&,const RunOptions&,const SimulationResult&);
} // namespace avoidance_sim
