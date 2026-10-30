#include "avoidance_sim/observations.hpp"
#include "cooperative_avoidance/geometry.hpp"
namespace avoidance_sim {
namespace ca=cooperative_avoidance;
namespace {
bool segment_blocked(ca::Vector2 a,ca::Vector2 b,const ca::Polygon& p) {
  // Clip the sight segment to a convex polygon's inward half-planes.
  double area=0;for(size_t i=0;i<p.size();++i)area+=ca::cross(p[i],p[(i+1)%p.size()]);
  const double sign=area>=0?1.:-1.;double lower=0,upper=1;
  for(size_t i=0;i<p.size();++i) {
    const auto edge=p[(i+1)%p.size()]-p[i];
    const double value=sign*ca::cross(edge,a-p[i]),slope=sign*ca::cross(edge,b-a);
    if(std::abs(slope)<1e-12) {if(value<0)return false;}
    else if(slope>0)lower=std::max(lower,-value/slope);
    else upper=std::min(upper,-value/slope);
    if(lower>upper+1e-10)return false;
  }
  return lower<=1&&upper>=0;
}
}
bool observable(const ca::RobotState& observer,const ca::RobotState& target,
 const std::vector<ca::Polygon>& walls,double range,double fov,bool line_of_sight) {
  if(ca::norm(target.pose.position-observer.pose.position)>range)return false;
  auto samples=ca::rectangle(target.pose,target.footprint);samples.push_back(target.pose.position);
  for(auto point:samples) {
    auto delta=point-observer.pose.position;
    if(std::abs(ca::wrap_angle(std::atan2(delta.y,delta.x)-observer.pose.heading))>fov/2+1e-10)continue;
    bool blocked=false;
    if(line_of_sight)for(const auto& wall:walls)if(segment_blocked(observer.pose.position,point,wall)){blocked=true;break;}
    if(!blocked)return true;
  }
  return false;
}
} // namespace avoidance_sim
