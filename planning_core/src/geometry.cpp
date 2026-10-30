#include "cooperative_avoidance/geometry.hpp"
#include <stdexcept>
namespace cooperative_avoidance {
// Canonical half-open interval [-pi,pi), including the ambiguous exact antipode.
double wrap_angle(double a) { const double w=std::remainder(a,2*pi);return w>=pi?w-2*pi:w; }
double point_segment_distance(Vector2 p, Vector2 a, Vector2 b, double* fraction) {
  auto d=b-a; double t=dot(d,d)>1e-20?std::clamp(dot(p-a,d)/dot(d,d),0.,1.):0;
  if(fraction) *fraction=t;
  return norm(p-(a+d*t));
}
Polygon rectangle(Pose2D p, RobotFootprint f, double margin) {
  const double c=std::cos(p.heading),s=std::sin(p.heading);
  Polygon result;
  for(auto v:Polygon{{-f.length/2-margin,-f.width/2-margin},{f.length/2+margin,-f.width/2-margin},
                    {f.length/2+margin,f.width/2+margin},{-f.length/2-margin,f.width/2+margin}})
    result.push_back(p.position+Vector2{c*v.x-s*v.y,s*v.x+c*v.y});
  return result;
}
bool convex_polygon(const Polygon& p) {
  if(p.size()<3) return false;
  double sign=0, area=0;
  for(size_t i=0;i<p.size();++i) {
    if(!std::isfinite(p[i].x)||!std::isfinite(p[i].y)) return false;
    const auto a=p[(i+1)%p.size()]-p[i], b=p[(i+2)%p.size()]-p[(i+1)%p.size()];
    if(norm(a)<1e-10) return false;
    double turn=cross(a,b); area+=cross(p[i],p[(i+1)%p.size()]);
    if(std::abs(turn)>1e-10) { if(sign*turn<0) return false; sign=turn; }
  }
  if(std::abs(area)<=1e-10||sign==0) return false;
  // Local turn signs alone also admit star polygons; every vertex must lie on
  // the same inward side of every edge for this SAT contract to hold.
  const double orientation=std::copysign(1.,sign);
  for(size_t i=0;i<p.size();++i) for(auto v:p)
    if(orientation*cross(p[(i+1)%p.size()]-p[i],v-p[i])<-1e-10)return false;
  return true;
}
bool polygons_collide(const Polygon& a,const Polygon& b) {
  if(a.size()<3||b.size()<3) throw std::invalid_argument("collision polygons need >=3 vertices");
  for(const auto* p:{&a,&b}) for(size_t i=0;i<p->size();++i) {
    auto e=(*p)[(i+1)%p->size()]-(*p)[i]; Vector2 axis{-e.y,e.x};
    double amin=dot(a[0],axis),amax=amin,bmin=dot(b[0],axis),bmax=bmin;
    for(auto x:a) { amin=std::min(amin,dot(x,axis)); amax=std::max(amax,dot(x,axis)); }
    for(auto x:b) { bmin=std::min(bmin,dot(x,axis)); bmax=std::max(bmax,dot(x,axis)); }
    if(amax<bmin-1e-12 || bmax<amin-1e-12) return false;
  }
  return true;
}
double polygon_separation(const Polygon& a,const Polygon& b) {
  if(polygons_collide(a,b)) return 0;
  double d=std::numeric_limits<double>::infinity();
  for(const auto& v:a) for(size_t j=0;j<b.size();++j) d=std::min(d,point_segment_distance(v,b[j],b[(j+1)%b.size()]));
  for(const auto& v:b) for(size_t j=0;j<a.size();++j) d=std::min(d,point_segment_distance(v,a[j],a[(j+1)%a.size()]));
  return d;
}
} // namespace cooperative_avoidance
