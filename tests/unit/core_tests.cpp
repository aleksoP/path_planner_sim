#define BOOST_TEST_MODULE cooperative_avoidance_core
#include <boost/test/included/unit_test.hpp>
#include "cooperative_avoidance/cooperative_planner.hpp"
#include "cooperative_avoidance/collision_checker.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include "cooperative_avoidance/motion_model.hpp"
#include "cooperative_avoidance/prediction.hpp"
#include "cooperative_avoidance/trajectory.hpp"
using namespace cooperative_avoidance;
BOOST_AUTO_TEST_CASE(geometry_and_clearance) {
  BOOST_CHECK_SMALL(point_segment_distance({1,1},{0,0},{2,0})-1,1e-10);
  BOOST_CHECK_SMALL(point_segment_distance({3,0},{0,0},{2,0})-1,1e-10);
  BOOST_CHECK_SMALL(point_segment_distance({1,0},{0,0},{0,0})-1,1e-10);
  BOOST_CHECK_SMALL(wrap_angle(3*pi)+pi,1e-10);
  auto a=rectangle({{0,0},pi/4},{2,1}),b=rectangle({{0.5,0},-pi/4},{2,1});
  BOOST_CHECK(polygons_collide(a,b));
  auto c=rectangle({{3,0},0},{2,1}),d=rectangle({{0,0},0},{2,1});
  BOOST_CHECK(!polygons_collide(c,d));BOOST_CHECK_SMALL(polygon_separation(c,d)-1,1e-10);
  auto wall=rectangle({{0,1.1},0},{10,.2});
  BOOST_CHECK(polygons_collide(rectangle({{0,.8},0},{.8,.6}),wall));
  BOOST_CHECK(!convex_polygon({{0,0},{1,0},{.2,.2},{1,1},{0,1}}));
  Polygon star;for(int i:{0,2,4,1,3})star.push_back({std::cos(i*2*pi/5),std::sin(i*2*pi/5)});
  BOOST_CHECK(!convex_polygon(star));
}
BOOST_AUTO_TEST_CASE(arc_length_and_smoothing) {
  ReferencePath p({{0,0},{0,0},{10,0}});
  auto q=p.project({3,2});BOOST_CHECK_SMALL(q.s-3,1e-9);BOOST_CHECK_SMALL(q.lateral+2,1e-9);
  BOOST_CHECK_SMALL(p.sample(4).position.x-4,1e-9);
  ReferencePath reversed({{10,0},{0,0}});
  BOOST_CHECK(p.sample(3).right_normal.y<0);BOOST_CHECK(reversed.sample(3).right_normal.y>0);
  ReferencePath corner({{0,0},{2,0},{2,2}});
  BOOST_CHECK(corner.points().size()>3);
  BOOST_CHECK_THROW(ReferencePath({{0,0},{0,0}}),std::invalid_argument);
}
BOOST_AUTO_TEST_CASE(quintic_endpoint_smoothness_and_sign) {
  BOOST_CHECK_SMALL(quintic(0),1e-12);BOOST_CHECK_SMALL(quintic(1)-1,1e-12);
  const double h=1e-4;
  BOOST_CHECK_SMALL((quintic(h)-quintic(0))/h,1e-5);
  BOOST_CHECK_SMALL((quintic(1)-quintic(1-h))/h,1e-5);
  BOOST_CHECK_SMALL((quintic(2*h)-2*quintic(h)+quintic(0))/(h*h),.02);
  BOOST_CHECK_SMALL((quintic(1)-2*quintic(1-h)+quintic(1-2*h))/(h*h),.02);
  ReferencePath p({{0,0},{20,0}});Deformation d{1,1,4,7,10};
  BOOST_CHECK_SMALL(d.position(p,5).y+1,1e-9);BOOST_CHECK_SMALL(d.lateral(12),1e-9);
}
BOOST_AUTO_TEST_CASE(replanned_envelope_starts_from_current_lateral_pose_and_closes) {
  Deformation d{.95,2,4,6,8,.4,.2};
  const double h=1e-5;
  BOOST_CHECK_SMALL(d.lateral(2)-.4,1e-10);
  BOOST_CHECK_SMALL((d.lateral(2+h)-d.lateral(2))/h-.2,1e-5);
  BOOST_CHECK_SMALL(d.lateral(4)-.95,1e-10);
  BOOST_CHECK_SMALL((d.lateral(4)-d.lateral(4-h))/h,1e-5);
  BOOST_CHECK_SMALL(d.lateral(8),1e-10);
  Deformation returning{0,2,4,4,4.01,.7,-.1};
  BOOST_CHECK_SMALL(returning.lateral(2)-.7,1e-10);
  BOOST_CHECK_SMALL(returning.lateral(4),1e-10);
}
BOOST_AUTO_TEST_CASE(interpolation_and_limited_reversal) {
  TimedTrajectory t{{0,{{0,0},pi-.1},{0,0}},{1,{{1,0},-pi+.1},{1,.2}}};
  auto mid=interpolate(t,.5);BOOST_CHECK_SMALL(mid.pose.position.x-.5,1e-10);
  BOOST_CHECK_SMALL(std::abs(mid.pose.heading)-pi,1e-10);BOOST_CHECK_SMALL(mid.velocity.linear-.5,1e-10);
  RobotState r;r.velocity.linear=1.5;auto m=UnicycleModel{}.step(r,{-1,10},.1);
  BOOST_CHECK_SMALL(m.state.velocity.linear-1.38,1e-9);BOOST_CHECK_SMALL(m.state.velocity.angular-.2,1e-9);
  BOOST_CHECK(m.saturated);
}
BOOST_AUTO_TEST_CASE(nominal_and_dynamic_feasibility) {
  ReferencePath p({{0,0},{20,0}});RobotState r;r.id="A";
  PlannerConfig c;CooperativePlanner planner(c);
  auto result=planner.plan({0,r,p,{{20,0},0},{},{}});
  BOOST_CHECK(result.status==PlanningStatus::Nominal);BOOST_CHECK(result.side==PassingSide::None);
  BOOST_CHECK(result.trajectory.back().pose.position.x>4);
  for(const auto& candidate:result.candidates)BOOST_CHECK(dynamically_feasible(candidate.trajectory,r.limits));
  auto curved=rollout(r,p,{1.1,0,3.2,6,9.2},1,c);BOOST_CHECK(dynamically_feasible(curved,r.limits));
  auto invalid=curved;invalid[1].velocity.linear=20;BOOST_CHECK(!dynamically_feasible(invalid,r.limits));
  TimedTrajectory sideways{{0,{{0,0},0},{1,0}},{1,{{0,1},0},{1,0}}};
  BOOST_CHECK(!dynamically_feasible(sideways,r.limits));
  auto turn=rollout(r,ReferencePath({{0,0},{.05,0}}),{},1,c,pi/2);
  BOOST_CHECK(std::abs(wrap_angle(turn.back().pose.heading-pi/2))<.02);
}
BOOST_AUTO_TEST_CASE(stale_intentions_fall_back_to_observed_velocity) {
  ObstacleState o;o.id="B";o.cooperative=true;o.velocity={1,0};o.timestamp=2;o.intention_timestamp=0;
  o.intention={{0,{{0,0},0},{0,0}},{10,{{0,0},0},{0,0}}};
  auto stale=predict_obstacle(o,2,{});BOOST_CHECK(!stale.used_intention);
  BOOST_CHECK_SMALL(stale.trajectory.back().pose.position.x-5,1e-9);
  o.intention_timestamp=2;auto fresh=predict_obstacle(o,2,{});BOOST_CHECK(fresh.used_intention);
  BOOST_CHECK_SMALL(fresh.trajectory.back().pose.position.x,1e-9);
}
BOOST_AUTO_TEST_CASE(impossible_geometry_rejected_and_side_commitment) {
  ReferencePath p({{0,0},{20,0}});RobotState r;r.id="A";r.pose.position={1,0};r.velocity.linear=1.5;
  ObstacleState o;o.id="B";o.pose={{10,0},pi};o.velocity={-1.5,0};
  auto walls=std::vector<Polygon>{rectangle({{10,2.6},0},{25,.2}),rectangle({{10,-2.6},0},{25,.2})};
  CooperativePlanner planner;auto first=planner.plan({0,r,p,{{20,0},0},walls,{o}});
  BOOST_REQUIRE(first.maneuver==Maneuver::Offset);BOOST_CHECK(first.side==PassingSide::Right);
  auto next=interpolate(first.trajectory,.1);r.pose=next.pose;r.velocity=next.velocity;o.pose.position.x-=.15;o.pose.position.y+=.002;o.timestamp=.1;
  auto second=planner.plan({.1,r,p,{{20,0},0},walls,{o}});
  BOOST_CHECK(second.side==PassingSide::Right);
  walls={rectangle({{10,.775},0},{25,.2}),rectangle({{10,-.775},0},{25,.2})};
  CooperativePlanner narrow;auto blocked=narrow.plan({.1,r,p,{{20,0},0},walls,{o}});
  BOOST_CHECK(blocked.maneuver!=Maneuver::Offset);
  for(const auto& c:blocked.candidates)if(c.maneuver==Maneuver::Offset)BOOST_CHECK(!c.feasible);
}
BOOST_AUTO_TEST_CASE(early_return_and_between_sample_collision_rejected) {
  TimedTrajectory returning{{0,{{0,-1},0},{1,0}},{1,{{1,0},0},{1,0}}};
  PredictedObstacleTrajectory obstacle;obstacle.id="B";obstacle.footprint={.8,.6};
  obstacle.trajectory={{0,{{1,0},pi},{0,0}},{1,{{1,0},pi},{0,0}}};
  auto unsafe=evaluate_trajectory(returning,{.8,.6},{},{obstacle},.15,.025);BOOST_CHECK(!unsafe.feasible);
  TimedTrajectory fast{{0,{{-1,0},0},{10,0}},{.2,{{1,0},0},{10,0}}};
  auto thin=rectangle({{0,0},0},{.01,2});
  BOOST_CHECK(!evaluate_trajectory(fast,{.1,.1},{thin},{},0,.025).feasible);
}
BOOST_AUTO_TEST_CASE(buffer_recovery_preserves_required_clearance) {
  TimedTrajectory away{{0,{{0,0},-pi/2},{.2,0}},{1,{{0,-.2},-pi/2},{.2,0}}};
  auto wall=rectangle({{0,.66},0},{10,.2});
  BOOST_CHECK(!evaluate_trajectory(away,{.8,.6},{wall},{},.19,.025).feasible);
  BOOST_CHECK(evaluate_trajectory(away,{.8,.6},{wall},{},.19,.025,.15).feasible);
  auto too_close=rectangle({{0,.64},0},{10,.2});
  BOOST_CHECK(!evaluate_trajectory(away,{.8,.6},{too_close},{},.19,.025,.15).feasible);
}
