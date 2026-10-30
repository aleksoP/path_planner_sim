#include "cooperative_avoidance/trajectory.hpp"
#include "cooperative_avoidance/geometry.hpp"
#include <stdexcept>
namespace cooperative_avoidance {
TrajectoryPoint interpolate(const TimedTrajectory& t,double time) {
  if(t.empty()) throw std::invalid_argument("cannot interpolate empty trajectory");
  if(time<=t.front().time) return t.front();
  if(time>=t.back().time) return t.back();
  auto it=std::upper_bound(t.begin(),t.end(),time,[](double x,const auto& p){return x<p.time;});
  const auto& b=*it; const auto& a=*(it-1); double u=(time-a.time)/(b.time-a.time);
  return {time,{a.pose.position*(1-u)+b.pose.position*u,wrap_angle(a.pose.heading+u*wrap_angle(b.pose.heading-a.pose.heading))},
    {a.velocity.linear*(1-u)+b.velocity.linear*u,a.velocity.angular*(1-u)+b.velocity.angular*u}};
}
bool dynamically_feasible(const TimedTrajectory& t,const MotionLimits& l,std::string* reason) {
  auto fail=[&](const char* s){if(reason)*reason=s;return false;};
  if(t.empty()) return fail("empty trajectory");
  for(size_t i=0;i<t.size();++i) {
    const auto& p=t[i]; const auto v=p.velocity;
    for(double x:{p.time,p.pose.position.x,p.pose.position.y,p.pose.heading,v.linear,v.angular})
      if(!std::isfinite(x)) return fail("non-finite trajectory");
    if(v.linear>l.max_forward+1e-7||v.linear<-l.max_reverse-1e-7||std::abs(v.angular)>l.max_angular+1e-7)
      return fail("velocity limit");
    if(std::abs(v.linear)>0.05&&std::abs(v.angular/v.linear)>l.max_curvature+1e-7) return fail("curvature limit");
    if(i) {
      double dt=p.time-t[i-1].time; if(dt<=0) return fail("non-increasing trajectory time");
      double rate=std::abs(v.linear)>std::abs(t[i-1].velocity.linear)?l.acceleration.acceleration:l.acceleration.deceleration;
      if(std::abs(v.linear-t[i-1].velocity.linear)>rate*dt+1e-7) return fail("linear acceleration limit");
      if(std::abs(v.angular-t[i-1].velocity.angular)>l.acceleration.angular_acceleration*dt+1e-7)
        return fail("angular acceleration limit");
      const auto& prev=t[i-1];
      const double speed_bound=std::max(std::abs(v.linear),std::abs(prev.velocity.linear));
      if(norm(p.pose.position-prev.pose.position)>speed_bound*dt+1e-6) return fail("nonholonomic position jump");
      const double yaw_bound=std::max(std::abs(v.angular),std::abs(prev.velocity.angular));
      if(std::abs(wrap_angle(p.pose.heading-prev.pose.heading))>yaw_bound*dt+1e-6) return fail("heading jump");
      const double yaw_delta=(v.angular+prev.velocity.angular)*dt/2;
      const double yaw_mid=prev.pose.heading+yaw_delta/2;
      const auto expected=prev.pose.position+Vector2{std::cos(yaw_mid),std::sin(yaw_mid)}*((v.linear+prev.velocity.linear)*dt/2);
      if(norm(p.pose.position-expected)>0.001+speed_bound*dt*dt*dt) return fail("nonholonomic integration mismatch");
      if(std::abs(wrap_angle(p.pose.heading-prev.pose.heading-yaw_delta))>0.001+dt*dt*dt)
        return fail("angular integration mismatch");
    }
  }
  return true;
}
#define CA_STR(Enum,Value) case Enum::Value: return #Value
const char* to_string(PassingSide x) {switch(x){CA_STR(PassingSide,None);CA_STR(PassingSide,Right);CA_STR(PassingSide,Left);}return "Unknown";}
const char* to_string(Maneuver x) {switch(x){CA_STR(Maneuver,Nominal);CA_STR(Maneuver,Offset);CA_STR(Maneuver,Slow);CA_STR(Maneuver,Stop);}return "Unknown";}
const char* to_string(PlanningStatus x) {switch(x){CA_STR(PlanningStatus,Nominal);CA_STR(PlanningStatus,Avoidance);CA_STR(PlanningStatus,Yielding);CA_STR(PlanningStatus,NoFeasibleManeuver);}return "Unknown";}
const char* to_string(EncounterState x) {switch(x){CA_STR(EncounterState,Normal);CA_STR(EncounterState,ConflictDetected);CA_STR(EncounterState,ManeuverSelected);CA_STR(EncounterState,Shifting);CA_STR(EncounterState,Passing);CA_STR(EncounterState,Returning);CA_STR(EncounterState,Yielding);CA_STR(EncounterState,Recovery);}return "Unknown";}
#undef CA_STR
} // namespace cooperative_avoidance
