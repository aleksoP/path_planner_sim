#pragma once
#include "types.hpp"
namespace cooperative_avoidance {
double wrap_angle(double angle);
double point_segment_distance(Vector2 p, Vector2 a, Vector2 b, double* fraction=nullptr);
Polygon rectangle(Pose2D pose, RobotFootprint footprint, double margin=0);
bool polygons_collide(const Polygon& a, const Polygon& b);
double polygon_separation(const Polygon& a, const Polygon& b);
bool convex_polygon(const Polygon& polygon);
} // namespace cooperative_avoidance
