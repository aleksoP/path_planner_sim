#include "cooperative_avoidance/cooperative_planner.hpp"
#include "cooperative_avoidance/collision_checker.hpp"
#include "cooperative_avoidance/prediction.hpp"
#include "cooperative_avoidance/trajectory.hpp"
#include "cooperative_avoidance/motion_model.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include <stdexcept>
#include <array>
namespace cooperative_avoidance {
namespace {
double longitudinal_extent(RobotFootprint f,double relative_heading) {
  return .5*(f.length*std::abs(std::cos(relative_heading))+f.width*std::abs(std::sin(relative_heading)));
}
std::array<double,7> shape_key(const Deformation& d) {
  return {d.offset,d.start,d.entry_end,d.hold_end,d.exit_end,d.initial_offset,d.initial_slope};
}
}
void validate_config(const PlannerConfig& c) {
  for(double x:{c.horizon,c.prediction_step,c.rollout_step,c.transition_length,c.lookahead,c.collision_resolution,
                c.goal_tolerance,c.intent_max_age,c.commitment_time,c.recovery_time,c.tracking_recovery_time})
    if(!std::isfinite(x)||x<=0) throw std::invalid_argument("planner time/length parameters must be finite and positive");
  if(c.horizon>60||c.horizon/std::min(c.rollout_step,c.prediction_step)>10000)
    throw std::invalid_argument("unbounded prediction configuration");
  for(double x:{c.clearance,c.tracking_margin,c.return_clearance,c.passing_length,c.uncertainty_growth,
    c.weights.progress,c.weights.deviation,c.weights.speed_reduction,c.weights.smoothness,c.weights.clearance,
    c.weights.left_side,c.weights.switching})
    if(!std::isfinite(x)||x<0) throw std::invalid_argument("invalid planner margin or weight");
  if(c.offsets.empty()||c.speed_scales.empty()) throw std::invalid_argument("candidate sets cannot be empty");
  for(double x:c.offsets) if(!std::isfinite(x)||x<=0) throw std::invalid_argument("offsets must be positive");
  for(double x:c.speed_scales) if(!std::isfinite(x)||x<=0||x>1) throw std::invalid_argument("speed scales must be in (0,1]");
  if(c.transition_scales.empty())throw std::invalid_argument("transition scales cannot be empty");
  for(double x:c.transition_scales)if(!std::isfinite(x)||x<=0||x>1)throw std::invalid_argument("transition scales must be in (0,1]");
}
CooperativePlanner::CooperativePlanner(PlannerConfig config):config_(std::move(config)) {validate_config(config_);}
void CooperativePlanner::reset() {side_=PassingSide::None;state_=EncounterState::Normal;committed_={};selected_at_=last_conflict_=progress_=0;blocked_encounter_.clear();encounter_agents_.clear();static_clear_s_=-1;}
PlannerResult CooperativePlanner::plan(const PlannerRequest& r) {
  validate_limits(r.robot.limits);
  auto finite_pose=[](Pose2D p){return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.heading);};
  auto valid_footprint=[](RobotFootprint f){return std::isfinite(f.length)&&std::isfinite(f.width)&&f.length>0&&f.width>0;};
  if(!std::isfinite(r.timestamp)||r.timestamp<0||!finite_pose(r.robot.pose)||!finite_pose(r.goal)||
     !valid_footprint(r.robot.footprint)||!std::isfinite(r.robot.velocity.linear)||!std::isfinite(r.robot.velocity.angular)||
     r.robot.velocity.linear>r.robot.limits.max_forward||r.robot.velocity.linear<-r.robot.limits.max_reverse||
     std::abs(r.robot.velocity.angular)>r.robot.limits.max_angular)
    throw std::invalid_argument("invalid planner robot state or timestamp");
  for(const auto& polygon:r.static_obstacles)if(!convex_polygon(polygon))throw std::invalid_argument("planner requires convex static polygons");
  for(const auto& o:r.obstacles)if(!finite_pose(o.pose)||!valid_footprint(o.footprint)||!std::isfinite(o.timestamp)||
    o.timestamp>r.timestamp+1e-8||!std::isfinite(o.velocity.x)||!std::isfinite(o.velocity.y)||!std::isfinite(o.angular_velocity)||
    !std::isfinite(o.uncertainty)||o.uncertainty<0||!std::isfinite(o.confidence)||o.confidence<0||o.confidence>1)
    throw std::invalid_argument("invalid obstacle observation");
  PlannerResult result;
  auto transition=[&](EncounterState next){if(next!=state_){state_=next;result.transitions.push_back(next);}};
  for(const auto& o:r.obstacles) result.predictions.push_back(predict_obstacle(o,r.timestamp,config_));
  progress_=std::max(progress_,r.reference_path.project(r.robot.pose.position,std::max(0.,progress_-0.2)).s);
  auto nominal=rollout(r.robot,r.reference_path,{},1,config_,r.goal.heading);
  auto nominal_safety=evaluate_trajectory(nominal,r.robot.footprint,r.static_obstacles,result.predictions,
                                         config_.clearance+config_.tracking_margin,config_.collision_resolution,config_.clearance,config_.tracking_recovery_time);
  result.predicted_conflict=!nominal_safety.feasible;
  for(const auto& o:r.obstacles)if(o.id==nominal_safety.obstacle_id||encounter_agents_.count(o.id))encounter_agents_[o.id]=o;
  if(side_==PassingSide::None&&nominal_safety.static_index<r.static_obstacles.size())for(auto vertex:r.static_obstacles[nominal_safety.static_index])
    static_clear_s_=std::max(static_clear_s_,r.reference_path.project(vertex).s);
  if(result.predicted_conflict) {
    last_conflict_=r.timestamp;
    if(state_==EncounterState::Normal||state_==EncounterState::Recovery) transition(EncounterState::ConflictDetected);
  }
  // A spatial envelope stays anchored across replans; a moving planning window would never finish shifting.
  if(side_!=PassingSide::None && progress_>committed_.exit_end &&
      r.timestamp-selected_at_>=config_.commitment_time && std::abs(r.reference_path.project(r.robot.pose.position).lateral)<0.12) {
    side_=PassingSide::None;committed_={};encounter_agents_.clear();static_clear_s_=-1;transition(EncounterState::Recovery);
  }
  if(state_==EncounterState::Recovery && r.timestamp-last_conflict_>config_.recovery_time)
    transition(EncounterState::Normal);

  if(!blocked_encounter_.empty()) for(const auto& o:r.obstacles) if(o.id==blocked_encounter_) {
    const auto tangent=r.reference_path.sample(progress_).tangent;
    if(dot(o.pose.position-r.robot.pose.position,tangent)<-(r.robot.footprint.length+o.footprint.length)/2-config_.return_clearance)
      blocked_encounter_.clear();
  }

  auto footprint_cleared=[&](Pose2D pose,double time) {
    const auto projection=r.reference_path.project(pose.position);
    const auto sample=r.reference_path.sample(projection.s);
    const double rear=projection.s-longitudinal_extent(r.robot.footprint,wrap_angle(pose.heading-sample.heading));
    if(static_clear_s_>=0&&rear<static_clear_s_+config_.return_clearance)return false;
    for(const auto& [id,observation]:encounter_agents_) {
      Pose2D obstacle_pose=observation.pose;double uncertainty=observation.uncertainty;
      bool predicted=false;
      for(const auto& prediction:result.predictions)if(prediction.id==id) {
        obstacle_pose=interpolate(prediction.trajectory,time).pose;uncertainty=prediction.uncertainty;predicted=true;break;
      }
      if(!predicted) {
        // A lost track is not an implicit clearance acknowledgment: include its last
        // measured occupancy as well as extrapolated occupancy before rejoining.
        const double age=std::max(0.,r.timestamp-observation.timestamp)+time;
        auto extrapolated=observation.pose;extrapolated.position=extrapolated.position+observation.velocity*age;
        if(r.reference_path.project(extrapolated.position).s>r.reference_path.project(obstacle_pose.position).s)obstacle_pose=extrapolated;
        uncertainty+=config_.uncertainty_growth*age;
      }
      const auto other=r.reference_path.project(obstacle_pose.position);
      const auto other_sample=r.reference_path.sample(other.s);
      const double front=other.s+longitudinal_extent(observation.footprint,wrap_angle(obstacle_pose.heading-other_sample.heading));
      if(rear<front+config_.return_clearance+uncertainty)return false;
    }
    return true;
  };
  std::vector<Deformation> candidate_shapes;
  std::map<std::array<double,7>,bool> geometry_cache;
  auto evaluate=[&](Deformation d,double scale,Maneuver maneuver,PassingSide side) {
    CandidateDiagnostic candidate;candidate.side=side;candidate.maneuver=maneuver;
    candidate.offset=d.offset;candidate.speed_scale=scale;
    candidate.trajectory=rollout(r.robot,r.reference_path,d,scale,config_,r.goal.heading);
    candidate.feasible=dynamically_feasible(candidate.trajectory,r.robot.limits,&candidate.reason);
    // Check the ENTIRE spatial maneuver against static geometry before committing. A finite
    // time horizon can end before an inadequate offset actually reaches the obstructing object.
    if(candidate.feasible&&maneuver==Maneuver::Offset) {
      const auto key=shape_key(d);
      auto cached=geometry_cache.find(key);
      bool geometry_ok=cached==geometry_cache.end()||cached->second;
      const double begin=std::max(progress_,d.start),end=std::min(d.exit_end,r.reference_path.length());
      for(double s=begin;cached==geometry_cache.end()&&s<=end+1e-8&&geometry_ok;s+=config_.collision_resolution) {
        const auto position=d.position(r.reference_path,s);
        const auto tangent=d.position(r.reference_path,s+0.01)-d.position(r.reference_path,s-0.01);
        const auto body=rectangle({position,std::atan2(tangent.y,tangent.x)},r.robot.footprint);
        const double recovery=std::clamp((s-begin)/config_.lookahead,0.,1.);
        for(const auto& obstacle:r.static_obstacles)if(polygon_separation(body,obstacle)<=
              config_.clearance+(config_.tracking_margin+config_.collision_resolution)*recovery) {
          geometry_ok=false;break;
        }
      }
      geometry_cache[key]=geometry_ok;
      if(!geometry_ok){candidate.feasible=false;candidate.reason="offset geometry static clearance";}
      if(candidate.feasible)for(const auto& point:candidate.trajectory) {
        const double s=r.reference_path.project(point.pose.position).s;
        const bool returning=(std::abs(d.offset)<1e-9&&std::abs(d.initial_offset)>1e-6)||s>d.hold_end;
        if(returning&&!footprint_cleared(point.pose,point.time)) {
          candidate.feasible=false;candidate.reason="return before footprint clearance";break;
        }
      }
    }
    if(candidate.feasible) {
      auto safe=evaluate_trajectory(candidate.trajectory,r.robot.footprint,r.static_obstacles,result.predictions,
                    config_.clearance+config_.tracking_margin,config_.collision_resolution,config_.clearance,config_.tracking_recovery_time);
      candidate.feasible=safe.feasible;candidate.reason=safe.reason;candidate.min_separation=safe.minimum_separation;
    }
    const auto& w=config_.weights;
    const double progress=r.reference_path.project(candidate.trajectory.back().pose.position).s-progress_;
    double smooth=0;for(size_t i=1;i<candidate.trajectory.size();++i)
      smooth+=std::abs(candidate.trajectory[i].velocity.angular-candidate.trajectory[i-1].velocity.angular);
    candidate.cost=-w.progress*progress+w.deviation*std::abs(d.offset)+w.speed_reduction*(1-scale)+w.smoothness*smooth+
      w.clearance/(0.1+candidate.min_separation)+(side==PassingSide::Left?w.left_side:0)+
      (side_!=PassingSide::None&&side!=side_?w.switching:0);
    if(side_!=PassingSide::None&&maneuver==Maneuver::Offset&&shape_key(d)!=shape_key(committed_))
      candidate.cost+=config_.weights.switching*.1;
    result.candidates.push_back(std::move(candidate));
    candidate_shapes.push_back(d);
  };

  if(side_==PassingSide::None) {
    evaluate({},1,Maneuver::Nominal,PassingSide::None);
    for(double scale:config_.speed_scales) if(scale<0.999) evaluate({},scale,Maneuver::Slow,PassingSide::None);
  } else {
    for(double scale:config_.speed_scales) evaluate(committed_,scale,Maneuver::Offset,side_);
  }
  bool fast_committed_feasible=false;
  for(const auto& candidate:result.candidates)if(candidate.feasible&&candidate.maneuver==Maneuver::Offset&&candidate.speed_scale>=.65)
    fast_committed_feasible=true;
  // Only generate offsets in response to a predicted trajectory conflict, never proximity alone.
  if((side_==PassingSide::None&&(result.predicted_conflict||!blocked_encounter_.empty()))||
     (side_!=PassingSide::None&&!fast_committed_feasible)) {
    double conflict_s=r.reference_path.project(interpolate(nominal,std::max(0.,nominal_safety.conflict_time)).pose.position).s;
    // Full-length footprint plus downstream buffer before exit. All return candidates are also time-checked.
    const auto projection=r.reference_path.project(r.robot.pose.position);
    const auto sample=r.reference_path.sample(progress_);
    const Vector2 heading{std::cos(r.robot.pose.heading),std::sin(r.robot.pose.heading)};
    const double slope=std::clamp(dot(heading,sample.right_normal)/std::max(.2,dot(heading,sample.tangent)),-2.,2.);
    for(double sign:{1.,-1.}) for(double offset:config_.offsets) for(double transition:config_.transition_scales) {
      if(side_!=PassingSide::None&&sign!=(side_==PassingSide::Right?1.:-1.))continue;
      const double length=config_.transition_length*transition;
      double hold=std::max(progress_+length+config_.passing_length,
                          conflict_s+r.robot.footprint.length+config_.return_clearance+config_.passing_length);
      Deformation d{sign*offset,progress_,progress_+length,hold,hold+length,projection.lateral,slope};
      if(d.exit_end>r.reference_path.length()-config_.goal_tolerance) continue;
      for(double scale:config_.speed_scales) evaluate(d,scale,Maneuver::Offset,sign>0?PassingSide::Right:PassingSide::Left);
    }
  }
  if(side_!=PassingSide::None) {
    // Extend an otherwise fixed passing section while an obstacle still occupies
    // the return region. The side stays committed; the merge station does not.
    if(!fast_committed_feasible)for(double extension:config_.transition_scales) {
      auto extended=committed_;extended.hold_end+=config_.transition_length*extension;
      extended.exit_end+=config_.transition_length*extension;
      if(extended.exit_end<=r.reference_path.length()-config_.goal_tolerance)
        for(double scale:config_.speed_scales)evaluate(extended,scale,Maneuver::Offset,side_);
    }
    if(footprint_cleared(r.robot.pose,0)) {
      const auto projection=r.reference_path.project(r.robot.pose.position);
      const auto sample=r.reference_path.sample(progress_);
      const Vector2 heading{std::cos(r.robot.pose.heading),std::sin(r.robot.pose.heading)};
      const double slope=std::clamp(dot(heading,sample.right_normal)/std::max(.2,dot(heading,sample.tangent)),-2.,2.);
      for(double transition:config_.transition_scales) {
        const double end=progress_+config_.transition_length*transition;
        if(end>r.reference_path.length()-config_.goal_tolerance)continue;
        Deformation rejoin{0,progress_,end,end,end+.01,projection.lateral,slope};
        for(double scale:config_.speed_scales)evaluate(rejoin,scale,Maneuver::Offset,side_);
      }
    }
  }

  // No backtracking or passing bays in V0.1: a head-on encounter with no geometrically
  // usable lateral envelope is a latched unresolved stop, not repeated forward creeping.
  bool static_offset_possible=false;
  for(const auto& c:result.candidates) if(c.maneuver==Maneuver::Offset&&c.reason!="offset geometry static clearance")
    static_offset_possible=true;
  if(side_==PassingSide::None&&result.predicted_conflict&&!static_offset_possible) for(const auto& o:r.obstacles) {
    const auto tangent=r.reference_path.sample(progress_).tangent;
    const auto heading=Vector2{std::cos(o.pose.heading),std::sin(o.pose.heading)};
    if(dot(tangent,heading)<-0.7&&dot(o.pose.position-r.robot.pose.position,tangent)>0&&
       r.reference_path.project(o.pose.position).distance<r.robot.footprint.width+o.footprint.width)
      blocked_encounter_=o.id;
  }
  if(!blocked_encounter_.empty())for(auto& c:result.candidates){
    c.feasible=false;c.reason="unresolved head-on geometry: holding for "+blocked_encounter_;
  }

  // At intersections, stable IDs impose a right-of-way preference. Safety is still a hard constraint.
  bool yield_priority=false;
  for(const auto& o:r.obstacles) {
    if(!o.cooperative||r.robot.id<o.id) continue;
    auto a=r.reference_path.sample(progress_).tangent,b=Vector2{std::cos(o.pose.heading),std::sin(o.pose.heading)};
    if(std::abs(dot(a,b))<0.7 && result.predicted_conflict) yield_priority=true;
  }
  if(yield_priority) for(auto& c:result.candidates) if(c.maneuver==Maneuver::Offset||c.speed_scale>0.4) {
    c.feasible=false;c.reason="crossing priority: larger ID yields";
  }
  // Braking is a real acceleration-limited rollout, not a zero-velocity teleport.
  evaluate(side_==PassingSide::None?Deformation{}:committed_,0,Maneuver::Stop,side_);
  size_t best=result.candidates.size(); double cost=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<result.candidates.size();++i) if(result.candidates[i].feasible&&result.candidates[i].cost<cost) {
    best=i;cost=result.candidates[i].cost;
  }
  if(best==result.candidates.size()) {
    const auto& braking=result.candidates.back();result.trajectory=braking.trajectory;
    result.min_predicted_separation=braking.min_separation;
    result.status=PlanningStatus::NoFeasibleManeuver;result.maneuver=Maneuver::Stop;result.side=side_;
    result.reason="No collision-free candidate under current predictions; bounded emergency braking: "+braking.reason;
    transition(EncounterState::Yielding);
  } else {
    const auto& selected=result.candidates[best];result.trajectory=selected.trajectory;result.side=selected.side;
    result.maneuver=selected.maneuver;result.min_predicted_separation=selected.min_separation;
    if(selected.maneuver==Maneuver::Offset) {
      if(side_==PassingSide::None) {
        side_=selected.side;selected_at_=r.timestamp;
        transition(EncounterState::ManeuverSelected);
      }
      else result.maneuver_replanned=shape_key(committed_)!=shape_key(candidate_shapes[best]);
      committed_=candidate_shapes[best];
      transition(std::abs(committed_.offset)<1e-9?EncounterState::Returning:progress_<committed_.entry_end?EncounterState::Shifting:
                 progress_<committed_.hold_end?EncounterState::Passing:EncounterState::Returning);
      result.status=PlanningStatus::Avoidance;
    } else if(selected.maneuver==Maneuver::Stop||selected.maneuver==Maneuver::Slow) {
      result.status=PlanningStatus::Yielding;transition(EncounterState::Yielding);
    } else {
      result.status=PlanningStatus::Nominal;
      if(!result.predicted_conflict&&side_==PassingSide::None&&state_!=EncounterState::Recovery) transition(EncounterState::Normal);
    }
    result.reason=selected.maneuver==Maneuver::Stop?"Safe bounded braking; passage remains unresolved":"Lowest-cost feasible candidate";
  }
  if(side_!=PassingSide::None) {
    result.rejoin_s=committed_.exit_end;
    const double end=std::min(r.reference_path.length(),committed_.exit_end+config_.lookahead);
    if(result.maneuver==Maneuver::Offset) {
      for(double s=progress_;s<end;s+=.1)result.maneuver_path.push_back(committed_.position(r.reference_path,s));
      result.maneuver_path.push_back(r.reference_path.sample(end).position);
    }
  }
  result.state=state_;
  return result;
}
} // namespace cooperative_avoidance
