#include "cooperative_avoidance/collision_checker.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include "cooperative_avoidance/trajectory.hpp"
namespace cooperative_avoidance {
CollisionEvaluation evaluate_trajectory(const TimedTrajectory& t,RobotFootprint f,
 const std::vector<Polygon>& statics,const std::vector<PredictedObstacleTrajectory>& moving,
 double clearance,double resolution,double initial_clearance,double recovery_time) {
  CollisionEvaluation out;
  if(t.empty()) {out.feasible=false;out.minimum_separation=0;out.conflict_time=0;out.reason="empty trajectory";return out;}
  double max_speed=0;
  const double radius=std::hypot(f.length,f.width)/2;
  for(const auto& p:t) max_speed=std::max(max_speed,std::abs(p.velocity.linear)+std::abs(p.velocity.angular)*radius);
  double other_speed=0;
  for(const auto& o:moving) for(const auto& p:o.trajectory) other_speed=std::max(other_speed,
    std::abs(p.velocity.linear)+std::abs(p.velocity.angular)*std::hypot(o.footprint.length,o.footprint.width)/2);
  const double max_dt=std::min(0.05,resolution/std::max(0.01,max_speed+other_speed));
  for(size_t k=0;k<t.size();++k) {
    const double begin=k?t[k-1].time:t[0].time, end=t[k].time;
    const size_t n=std::max<size_t>(1,std::ceil((end-begin)/max_dt));
    // Bound unobserved separation between samples by relative footprint point speed.
    const double bound=(max_speed+other_speed)*(end-begin)/n/2;
    for(size_t j=0;j<=n;++j) {
      double time=begin+(end-begin)*j/n; const auto a=rectangle(interpolate(t,time).pose,f);
      const double required=std::isfinite(initial_clearance)?initial_clearance+(clearance-initial_clearance)*
        std::clamp(time/std::max(1e-6,recovery_time),0.,1.):clearance;
      for(size_t index=0;index<statics.size();++index) {
        const auto& b=statics[index];
        double d=polygon_separation(a,b); out.minimum_separation=std::min(out.minimum_separation,d);
        if(d<=required+bound) {out.feasible=false;out.conflict_time=time;out.reason="static clearance";out.static_index=index;return out;}
      }
      for(const auto& o:moving) {
        double d=polygon_separation(a,rectangle(interpolate(o.trajectory,time).pose,o.footprint));
        out.minimum_separation=std::min(out.minimum_separation,d);
        const double uncertainty=o.uncertainty_growth>0?o.initial_uncertainty+o.uncertainty_growth*time:o.uncertainty;
        if(d<=required+uncertainty+bound) {out.feasible=false;out.conflict_time=time;out.reason="dynamic clearance: "+o.id;out.obstacle_id=o.id;return out;}
      }
    }
  }
  return out;
}
} // namespace cooperative_avoidance
