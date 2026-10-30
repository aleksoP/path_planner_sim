#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
struct MotionStep { RobotState state; bool saturated{false}; };
class MotionModel {
 public:
  virtual ~MotionModel()=default;
  virtual MotionStep step(const RobotState&, Twist2D command, double dt) const=0;
};
class UnicycleModel final : public MotionModel {
 public:
  MotionStep step(const RobotState&, Twist2D command, double dt) const override;
};
void validate_limits(const MotionLimits& limits);
} // namespace cooperative_avoidance
