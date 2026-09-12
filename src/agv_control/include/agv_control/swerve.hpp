#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <optional>

namespace agv {
constexpr double pi = 3.14159265358979323846;
using Four = std::array<double, 4>;
struct Twist { double x{}, y{}, yaw{}; };
struct Config {
  double wheelbase{1.3}, track{0.94}, radius{0.2};
  // The steering limits are asymmetric about straight ahead. A wheel direction is
  // periodic modulo pi, so the span alone fixes the guaranteed margin -- every
  // direction has a branch with margin m iff span >= pi + 2m -- while where the
  // span sits decides which direction is left owning the near-limit branch.
  // [-275, 95] deg keeps both straight-ahead branches at 95 deg and leaves the
  // poor branch on lateral +pi/2, which is only held during a transfer.
  double soft_lower{-275*pi/180}, soft_upper{95*pi/180};
  double rate{0.65}, steer_accel{1.5}, max_speed{15/3.6}, max_yaw{0.35}, accel{0.8}, decel{1.0};
  double reorient{0.30}, aligned{0.035}, stopped{0.025}, hysteresis{0.08};
  double alignment_motion_confirm_s{0.06};
  double lateral_mismatch{0.12};
  double max_lateral_speed{1.0};
  double limit_reserve{3*pi/180}, segment_margin{20*pi/180}, wheel_deadband{0.005};
  double span() const {return soft_upper-soft_lower;}
  double margin(double a) const {return std::min(soft_upper-a,a-soft_lower);}
  // Margin a branch must keep once a latched limit approach forces recentring.
  // This is the old |a| <= pi/2+hysteresis test restated as a margin, and being
  // hysteresis below the guaranteed margin (span-pi)/2 it always leaves a branch.
  double recentre_margin() const {return span()/2-pi/2-hysteresis;}
  void validate() const {
    for (double v : {wheelbase,track,radius,rate,steer_accel,max_speed,max_yaw,accel,decel,reorient,aligned,stopped,hysteresis,lateral_mismatch,max_lateral_speed,limit_reserve,segment_margin,wheel_deadband,alignment_motion_confirm_s})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid controller parameter");
    // Straight ahead is the mechanical zero and must stay directly commandable.
    if (!std::isfinite(soft_lower) || !std::isfinite(soft_upper) || soft_lower >= 0 || soft_upper <= 0)
      throw std::invalid_argument("invalid steering limits");
    // The span relation subsumes the per-endpoint checks a symmetric limit needed:
    // it already guarantees a legal lateral +-pi/2 branch outside the margin.
    if (alignment_motion_confirm_s > .1 || aligned >= reorient
        || span() < pi+2*segment_margin || segment_margin <= limit_reserve
        || recentre_margin() <= limit_reserve || wheel_deadband>=stopped)
      throw std::invalid_argument("invalid steering thresholds");
  }
};
struct Target { Four angle{}, speed{}; };
enum class Mode { Hold, Drive, Brake, Align };
inline const char * name(Mode s) {
  switch(s) {case Mode::Hold:return "HOLD";case Mode::Drive:return "DRIVE";
    case Mode::Brake:return "BRAKE";case Mode::Align:return "ALIGN";} return "UNKNOWN";
}
inline double approach(double a, double b, double delta) {return a+std::clamp(b-a,-delta,delta);}
class Controller {
 public:
  explicit Controller(Config c={}) : c_(c) { c_.validate(); }
  void reset() {out_={};previous_={};steer_velocity_={};initialized_=false;mode_=Mode::Hold;settled_=0;alignment_motion_time_=0;reason_="NONE";unwinding_=false;}
  Target allocate(Twist t, const Four &actual, const Four &previous, bool reconfigure=false,
                  bool stationary=false) const {
    if (!std::isfinite(t.x)||!std::isfinite(t.y)||!std::isfinite(t.yaw))
      throw std::invalid_argument("nonfinite twist");
    const double linear=std::hypot(t.x,t.y);
    const double scale=std::max({1.0,linear/c_.max_speed,std::abs(t.yaw)/c_.max_yaw,
      std::abs(t.y)/c_.max_lateral_speed});
    t.x/=scale;t.y/=scale;t.yaw/=scale;
    Target out;
    double peak=0;
    for (size_t i=0;i<4;++i) {
      const double x=(i<2?1:-1)*c_.wheelbase/2;
      const double y=(i%2==0?1:-1)*c_.track/2;
      const double vx=t.x-t.yaw*y, vy=t.y+t.yaw*x;
      const double speed=std::hypot(vx,vy);
      out.angle[i]=actual[i]; out.speed[i]=0;
      // At a wheel's instantaneous centre of rotation, heading is undefined.
      // A finite deadband prevents encoder / command noise from swinging it.
      if (speed<c_.wheel_deadband) continue;
      const double base=std::atan2(vy,vx);
      double best=1e100;
      for (int k=-3;k<=3;++k) {
        const double a=base+k*pi;
        if (a < c_.soft_lower || a > c_.soft_upper) continue;
        const double margin=c_.margin(a);
        // A stopped vehicle re-steers before it drives, so it must not begin a
        // segment on a branch that leaves less travel than the trajectory may ask
        // for. Both branches of a direction are kinematically identical; only the
        // interior one survives a pass of cross-track and heading corrections.
        if (stationary && margin<c_.segment_margin) continue;
        // Only a latched limit-reconfiguration event restricts choices to the
        // branch furthest from either limit. Ordinary stops/reversals do not.
        if (reconfigure && margin<c_.recentre_margin()) continue;
        // Mechanical travel, not wrapped angle distance. Prefer continuity at ties.
        const double cost=std::abs(a-actual[i])+0.02/(0.1+margin)
          +(std::abs(a-previous[i])>pi/2?c_.hysteresis:0);
        if (cost<best) {best=cost;out.angle[i]=a;out.speed[i]=(k%2==0?speed:-speed);}
      }
      peak=std::max(peak,std::abs(out.speed[i]));
    }
    if (peak>c_.max_speed) for(auto &s:out.speed)s*=c_.max_speed/peak;
    return out;
  }
  Target update(Twist request, const Four &angles, const Four &speeds, double dt,
                std::optional<Four> measured_steer_rates=std::nullopt) {
    if (!(dt>0 && dt<=0.1)) throw std::invalid_argument("invalid timestep");
    Four actual_steer_rates{};
    for(size_t i=0;i<4;++i) actual_steer_rates[i]=measured_steer_rates?
      (*measured_steer_rates)[i]:(initialized_?(angles[i]-last_angles_[i])/dt:0);
    for(size_t i=0;i<4;++i) if(!std::isfinite(angles[i])||!std::isfinite(speeds[i])||!std::isfinite(actual_steer_rates[i]))
      throw std::invalid_argument("nonfinite feedback");
    if(!initialized_) {out_.angle=angles;out_.speed=speeds;previous_=angles; initialized_=true;}
    last_angles_=angles;
    // Select a different branch only after the limit approach is latched and
    // the rolling motion has stopped. All other cases prefer shortest travel.
    const bool stationary=mode_!=Mode::Drive && max_abs(speeds)<c_.stopped
      && max_abs(out_.speed)<c_.stopped;
    const bool reconfigure=unwinding_ && stationary;
    auto target=allocate(request,angles,previous_,reconfigure,stationary);
    previous_=target.angle;
    double err=0,moving=0,desired=0;
    bool reverse=false, excessive_mismatch=false, limit_approach=false;
    for(size_t i=0;i<4;++i) {
      err=std::max(err,std::abs(target.angle[i]-angles[i]));
      moving=std::max(moving,std::abs(speeds[i]));
      desired=std::max(desired,std::abs(target.speed[i]));
      reverse |= speeds[i]*target.speed[i]<0 && std::abs(speeds[i])>c_.stopped;
      excessive_mismatch |= std::abs(speeds[i]*std::sin(target.angle[i]-angles[i]))>c_.lateral_mismatch;
      const double steering_speed=std::max(std::abs(steer_velocity_[i]),std::abs(actual_steer_rates[i]));
      const double reserve=c_.limit_reserve+steering_speed*steering_speed/(2*c_.steer_accel)
        +steering_speed*dt;
      // With the limits asymmetric about straight ahead, "moving outward" is no
      // longer "moving away from zero": what decides is the sign of the motion and
      // the headroom left on that side, measured at whichever of the feedback and
      // the internal command leads in that direction.
      const double ahead=std::max(angles[i],out_.angle[i]);
      const double behind=std::min(angles[i],out_.angle[i]);
      const auto nearing=[&](double direction){
        return std::abs(direction)>1e-8
          && (direction>0?c_.soft_upper-ahead:behind-c_.soft_lower)<=reserve;};
      limit_approach |= nearing(target.angle[i]-angles[i]) || nearing(actual_steer_rates[i])
        || nearing(steer_velocity_[i]);
    }
    const bool zero=desired<1e-5;
    if(mode_==Mode::Drive && (err>c_.reorient||reverse||zero||excessive_mismatch||limit_approach)) {
      mode_=Mode::Brake;
      unwinding_=limit_approach;
      reason_=limit_approach?"LIMIT_RECONFIGURE":zero?"STOP_REQUEST":reverse?"DRIVE_REVERSAL":
        excessive_mismatch?"LATERAL_MISMATCH":"LARGE_STEER_CHANGE";
    }
    if(mode_==Mode::Hold && moving>=c_.stopped) mode_=Mode::Brake;
    // Steering under contact can produce a brief passive wheel-velocity pulse.
    // Keep drive output zero and reset the drive-ready dwell during such pulses.
    // Significant or sustained rolling still stops steering immediately/after confirmation.
    alignment_motion_time_=(mode_==Mode::Align && moving>=c_.stopped)?alignment_motion_time_+dt:0;
    if(mode_==Mode::Align && (moving>=2*c_.stopped || alignment_motion_time_>=c_.alignment_motion_confirm_s)) {
      mode_=Mode::Brake;reason_="ALIGN_WHEEL_MOTION";
    }
    if(mode_==Mode::Hold && !zero) mode_=Mode::Align;
    if(mode_==Mode::Brake && moving<c_.stopped && max_abs(out_.speed)<c_.stopped
       && max_abs(steer_velocity_)<0.01 && max_abs(actual_steer_rates)<0.08)
      mode_=zero?Mode::Hold:Mode::Align;
    if(mode_==Mode::Align && zero) mode_=Mode::Brake;
    double steering_tracking_error=0;
    for(size_t i=0;i<4;++i) steering_tracking_error=std::max(steering_tracking_error,
      std::abs(actual_steer_rates[i]-steer_velocity_[i]));
    // A continuously changing request may already be tracked accurately while
    // the steering joints are turning. It need not wait for zero steering rate.
    if(mode_==Mode::Align && !zero && moving<c_.stopped && err<c_.aligned && steering_tracking_error<0.08
       && max_abs(actual_steer_rates)<=c_.rate+0.05) {
      settled_+=dt;
      if(settled_>=0.10) {mode_=Mode::Drive;settled_=0;unwinding_=false;}
    } else settled_=0;
    double common_scale=1;
    for(size_t i=0;i<4;++i) common_scale=std::min(common_scale,
      std::max(0.0,std::cos(target.angle[i]-angles[i])));
    for(size_t i=0;i<4;++i) {
      if(mode_==Mode::Align||mode_==Mode::Drive) {
        // Acceleration-limited steering, with stopping-distance speed profile.
        const double e=target.angle[i]-out_.angle[i];
        // Reserve one sample of travel as well as braking distance. This avoids
        // snapping the final position step and violating the acceleration limit.
        const double adt=c_.steer_accel*dt;
        double velocity=std::copysign(std::min(c_.rate,
          std::sqrt(adt*adt+2*c_.steer_accel*std::abs(e))-adt),e);
        steer_velocity_[i]=approach(steer_velocity_[i],velocity,c_.steer_accel*dt);
        const double step=steer_velocity_[i]*dt;
        out_.angle[i]=std::clamp(out_.angle[i]+step,
          std::min(c_.soft_lower,out_.angle[i]),std::max(c_.soft_upper,out_.angle[i]));
      } else {
        // Stop existing steering motion smoothly before holding the wheel axes.
        // Do not pursue the newly requested angle while the chassis is braking.
        steer_velocity_[i]=approach(steer_velocity_[i],0,c_.steer_accel*dt);
        out_.angle[i]=std::clamp(out_.angle[i]+steer_velocity_[i]*dt,
          std::min(c_.soft_lower,out_.angle[i]),std::max(c_.soft_upper,out_.angle[i]));
      }
    }
    // One interpolation fraction preserves coordinated wheel-speed proportions.
    Four desired_speed{}; double f=1;
    for(size_t i=0;i<4;++i) {
      desired_speed[i]=mode_==Mode::Drive?target.speed[i]*common_scale:0;
      const double old=out_.speed[i], next=desired_speed[i];
      const double delta=std::abs(next-old);
      if(delta==0) continue;
      const bool crossing=old*next<0;
      const double limit=(crossing||std::abs(next)<std::abs(old))?c_.decel:c_.accel;
      f=std::min(f,limit*dt/delta);
      // A sign reversal must reach zero before accelerating in the new direction.
      if(crossing)f=std::min(f,std::abs(old)/delta);
    }
    for(size_t i=0;i<4;++i)out_.speed[i]+=f*(desired_speed[i]-out_.speed[i]);
    return out_;
  }
  Mode mode()const{return mode_;}
  const std::string &reason()const{return reason_;}
  static double max_abs(const Four &a){double m=0;for(double x:a)m=std::max(m,std::abs(x));return m;}
 private:
  Config c_; Target out_; Four previous_{},steer_velocity_{},last_angles_{};
  bool initialized_{false}; Mode mode_{Mode::Hold}; double settled_{},alignment_motion_time_{};
  std::string reason_{"NONE"};
  bool unwinding_{false};
};
} // namespace agv
