#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace agv_linescan {
// Equal-weight rigid-body fit from rolling wheel velocity vectors. No pose truth.
struct ScanVelocity { double vx=0,vy=0,wz=0,residual=0; };
inline ScanVelocity FitScanVelocity(const std::array<double,4>& speed,
                                    const std::array<double,4>& angle,double wheelbase,double track) {
  if(!(wheelbase>0 && track>0))throw std::invalid_argument("invalid wheel geometry");
  ScanVelocity v;std::array<double,4> ux{},uy{};
  const std::array<double,4> x={wheelbase/2,wheelbase/2,-wheelbase/2,-wheelbase/2};
  const std::array<double,4> y={track/2,-track/2,track/2,-track/2};
  for(size_t i=0;i<4;++i){
    if(!std::isfinite(speed[i])||!std::isfinite(angle[i]))throw std::invalid_argument("nonfinite encoder");
    ux[i]=speed[i]*std::cos(angle[i]);uy[i]=speed[i]*std::sin(angle[i]);
    v.vx+=ux[i]/4;v.vy+=uy[i]/4;v.wz+=-y[i]*ux[i]+x[i]*uy[i];
  }
  v.wz/=wheelbase*wheelbase+track*track;
  for(size_t i=0;i<4;++i)v.residual=std::max(v.residual,std::hypot(ux[i]-v.vx+v.wz*y[i],uy[i]-v.vy-v.wz*x[i]));
  return v;
}
class ProjectedEncoder {
 public:
  double Update(const std::array<double,4>& wheelDistance,const std::array<double,4>& angle){
    for(size_t i=0;i<4;++i){
      if(!std::isfinite(wheelDistance[i])||!std::isfinite(angle[i]))throw std::invalid_argument("nonfinite encoder");
      if(valid_)distance_+=(wheelDistance[i]-previous_[i])*std::cos((angle[i]+angle_[i])/2)/4;
    }
    previous_=wheelDistance;angle_=angle;valid_=true;return distance_;
  }
  void Reset(){valid_=false;distance_=0;}
 private:
  std::array<double,4> previous_{},angle_{};bool valid_=false;double distance_=0;
};
}
