#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <gz/math/Pose3.hh>

namespace agv_linescan {
// Absolute floor preserves low-speed gating; relative tolerance scales with travel speed.
inline double WheelSpeedSpreadLimit(double meanSpeed,double absolute,double relative) {
  if(!std::isfinite(meanSpeed)||!std::isfinite(absolute)||!std::isfinite(relative)||absolute<0||relative<0||relative>.05)
    throw std::invalid_argument("invalid wheel speed consistency tolerance");
  return std::max(absolute,relative*std::abs(meanSpeed));
}
// Resampled AB phase: fixed lines per shaft revolution. No electrical waveform
// or integer-count timing jitter is invented; fractional phase is kept by Trigger.
struct WheelEncoderScale {
  double linesPerTurn, spacing;
  WheelEncoderScale(int ppr,int decode,int multiplier,int divider,double calibratedRadius) {
    if(ppr<=0 || ppr>1000000 || decode!=4 || multiplier<=0 || multiplier>128 ||
       (multiplier & (multiplier-1)) || divider<=0 || divider>255 ||
       !std::isfinite(calibratedRadius) || calibratedRadius<=0)
      throw std::invalid_argument("invalid wheel encoder rescaler");
    linesPerTurn=double(ppr)*decode*multiplier/divider;
    spacing=2*std::acos(-1.)*calibratedRadius/linesPerTurn;
  }
};
struct Event { double time, distance; int direction; };
class Trigger {
 public:
  explicit Trigger(double spacing,bool positionMode=false): spacing_(spacing),positionMode_(positionMode) {
    if (!(spacing > 0)) throw std::invalid_argument("invalid line spacing");
  }
  void Reset() { valid_ = false; direction_ = 0; maxRetrace_=0; retraceEpisodes_=0; retracing_=false; }
  double MaxRetrace() const {return maxRetrace_;}
  size_t RetraceEpisodes() const {return retraceEpisodes_;}
  std::vector<Event> Update(double time, double distance) {
    if (!std::isfinite(time) || !std::isfinite(distance)) throw std::runtime_error("nonfinite encoder");
    if (!valid_) { time_=time; distance_=origin_=extreme_=distance; valid_=true; return {}; }
    if (time <= time_) throw std::runtime_error("encoder time reset");
    std::vector<Event> result;
    double delta=distance-distance_;
    if (!direction_ && std::abs(distance-origin_)>=spacing_-1e-12) {
      direction_=distance>=origin_ ? 1 : -1;
      nextIndex_=1;
      next_=origin_+direction_*spacing_;
    }
    if (direction_) {
      // PositionUp/Down semantics: count signed displacement during retrace,
      // but emit only at new positions beyond the original next-line frontier.
      const double retrace=std::max(0.,direction_*(extreme_-distance));
      maxRetrace_=std::max(maxRetrace_,retrace);
      if(retrace>=spacing_ && !retracing_){++retraceEpisodes_;retracing_=true;}
      if(retrace<=1e-12)retracing_=false;
      if (!positionMode_ && retrace>=spacing_) throw std::runtime_error("direction change");
      if (direction_*(distance-extreme_)>0) extreme_=distance;
    }
    if (direction_ && direction_*delta>1e-12) {
      int direction=direction_;
      while (direction*(distance-next_)>=-1e-12) {
        result.push_back({time_+(next_-distance_)/delta*(time-time_), next_, direction});
        next_=origin_+double(direction)*(++nextIndex_)*spacing_;
        if (result.size()>10000) throw std::runtime_error("excessive triggers per step");
      }
    }
    time_=time; distance_=distance;
    return result;
  }
 private:
  double spacing_, time_=0, distance_=0, next_=0, origin_=0, extreme_=0;
  size_t nextIndex_=1;
  int direction_=0;
  bool valid_=false,positionMode_=false,retracing_=false;
  double maxRetrace_=0;size_t retraceEpisodes_=0;
};

inline double Polynomial(const std::vector<double> &c, double x) {
  double y=0; for (auto it=c.rbegin(); it!=c.rend(); ++it) y=y*x+*it;
  return y;
}

struct Optics {
  size_t width, renderWidth;
  double scale, height, span;
  std::vector<double> source;
  Optics(size_t w, size_t rw, double pitch, double focal, double groundWidth,
         const std::vector<double> &polynomial): width(w), renderWidth(rw) {
    if (w<2 || rw<w || pitch<=0 || focal<=0 || groundWidth<=0 || polynomial.size()<2)
      throw std::invalid_argument("invalid optics");
    scale=w*pitch/(2*focal); height=groundWidth/(2*scale);
    // Validate monotonicity over the entire physical line including both edges.
    double previous=Polynomial(polynomial,-1);
    for (int i=1;i<=32768;++i) {
      double next=Polynomial(polynomial,-1+2.*i/32768);
      if (!std::isfinite(next) || next<=previous) throw std::invalid_argument("nonmonotonic optics");
      previous=next;
    }
    span=scale*std::max(std::abs(Polynomial(polynomial,-1)),std::abs(Polynomial(polynomial,1)));
    for (size_t u=0;u<w;++u) {
      double q=(double(u)-(w-1)/2.)/(w/2.);
      double ray=scale*Polynomial(polynomial,q);
      double pixel=(ray/span+1)*rw/2.-.5;
      if (pixel<0 || pixel>rw-1) throw std::invalid_argument("insufficient render guard pixels");
      source.push_back(pixel);
    }
  }
};

inline gz::math::Pose3d Interpolate(const gz::math::Pose3d &a,
                                  const gz::math::Pose3d &b, double fraction) {
  if (fraction<0 || fraction>1) throw std::runtime_error("pose extrapolation");
  return {a.Pos()+(b.Pos()-a.Pos())*fraction,
          gz::math::Quaterniond::Slerp(fraction,a.Rot(),b.Rot(),true)};
}
}  // namespace agv_linescan
