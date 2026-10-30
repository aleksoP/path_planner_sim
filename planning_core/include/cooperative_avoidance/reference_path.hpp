#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
struct PathSample { Vector2 position, tangent, right_normal; double heading{0}; };
struct PathProjection { double s{0}, distance{0}, lateral{0}; };
class ReferencePath {
 public:
  explicit ReferencePath(std::vector<Vector2> points, unsigned smoothing_passes=3);
  PathSample sample(double s) const;
  PathProjection project(Vector2 p, double minimum_s=0) const;
  double length() const { return lengths_.back(); }
  const std::vector<Vector2>& points() const { return points_; }
 private:
  std::vector<Vector2> points_;
  std::vector<double> lengths_;
};
} // namespace cooperative_avoidance
