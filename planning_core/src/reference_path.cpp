#include "cooperative_avoidance/reference_path.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include <stdexcept>
namespace cooperative_avoidance {
ReferencePath::ReferencePath(std::vector<Vector2> points,unsigned passes) {
  for(auto p:points) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)) throw std::invalid_argument("non-finite path point");
    if(points_.empty()||norm(p-points_.back())>1e-8) points_.push_back(p);
  }
  if(points_.size()<2) throw std::invalid_argument("reference path needs two distinct points");
  // Endpoint-preserving Chaikin corner cutting. Never claim an arbitrary polyline is C2.
  for(unsigned k=0;k<passes&&points_.size()>2;++k) {
    std::vector<Vector2> smooth{points_.front()};
    for(size_t i=1;i<points_.size();++i) {
      smooth.push_back(points_[i-1]*0.75+points_[i]*0.25);
      smooth.push_back(points_[i-1]*0.25+points_[i]*0.75);
    }
    smooth.push_back(points_.back()); points_=std::move(smooth);
  }
  lengths_.push_back(0);
  for(size_t i=1;i<points_.size();++i) lengths_.push_back(lengths_.back()+norm(points_[i]-points_[i-1]));
}
PathSample ReferencePath::sample(double s) const {
  auto position=[&](double arc) {
    arc=std::clamp(arc,0.,length());
    size_t i=std::min<size_t>(std::upper_bound(lengths_.begin(),lengths_.end(),arc)-lengths_.begin(),lengths_.size()-1);
    i=std::max<size_t>(1,i);
    double u=(arc-lengths_[i-1])/(lengths_[i]-lengths_[i-1]);
    return points_[i-1]*(1-u)+points_[i]*u;
  };
  auto tangent=unit(position(s+0.15)-position(s-0.15));
  return {position(s),tangent,{tangent.y,-tangent.x},std::atan2(tangent.y,tangent.x)};
}
PathProjection ReferencePath::project(Vector2 p,double minimum_s) const {
  PathProjection best; best.distance=std::numeric_limits<double>::infinity();
  for(size_t i=1;i<points_.size();++i) {
    if(lengths_[i]<minimum_s) continue;
    double u=0; point_segment_distance(p,points_[i-1],points_[i],&u);
    u=std::max(u,std::clamp((minimum_s-lengths_[i-1])/(lengths_[i]-lengths_[i-1]),0.,1.));
    auto q=points_[i-1]+(points_[i]-points_[i-1])*u; double d=norm(p-q);
    if(d<best.distance) {
      best.s=lengths_[i-1]+u*(lengths_[i]-lengths_[i-1]); best.distance=d;
      best.lateral=dot(p-q,sample(best.s).right_normal);
    }
  }
  if(!std::isfinite(best.distance)) return project(p,length()-1e-8);
  return best;
}
} // namespace cooperative_avoidance
