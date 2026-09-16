#pragma once
#include <algorithm>
#include <optional>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <nlohmann/json.hpp>
#include <gz/math/Vector3.hh>
namespace agv_linescan {
// Same piecewise triangular reference surface as heightfield.py. This is a
// bounded reference for sampling diagnostics, not a replacement for OptiX hits.
class GroundGeometry {
 public:
  explicit GroundGeometry(const nlohmann::json &j):x_(j.at("x").get<std::vector<double>>()),y_(j.at("y").get<std::vector<double>>()),z_(j.at("z").get<std::vector<std::vector<double>>>()) {
    if(x_.size()<2||y_.size()<2||z_.size()!=y_.size())throw std::runtime_error("invalid geometry heightfield");
    for(const auto *a:{&x_,&y_})for(size_t i=0;i<a->size();++i)
      if(!std::isfinite((*a)[i])||(i&&(*a)[i]<=(*a)[i-1]))throw std::runtime_error("invalid geometry axes");
    for(const auto &r:z_) {if(r.size()!=x_.size())throw std::runtime_error("invalid geometry grid");for(double v:r)if(!std::isfinite(v))throw std::runtime_error("invalid geometry height");}
  }
  std::optional<double> Height(double x,double y) const {
    if(!std::isfinite(x)||!std::isfinite(y)||x<x_.front()||x>x_.back()||y<y_.front()||y>y_.back())return {};
    auto i=std::min(size_t(std::upper_bound(x_.begin(),x_.end(),x)-x_.begin()-1),x_.size()-2);
    auto j=std::min(size_t(std::upper_bound(y_.begin(),y_.end(),y)-y_.begin()-1),y_.size()-2);
    double u=(x-x_[i])/(x_[i+1]-x_[i]),v=(y-y_[j])/(y_[j+1]-y_[j]);
    double a=z_[j][i],b=z_[j][i+1],c=z_[j+1][i+1],d=z_[j+1][i];
    return v<=u?a*(1-u)+b*(u-v)+c*v:a*(1-v)+c*u+d*(v-u);
  }
  std::optional<gz::math::Vector3d> Hit(const nlohmann::json &tag) const {
    auto p=tag.at("camera_position_world_m").get<std::vector<double>>();
    auto r=tag.at("camera_rotation_world").get<std::vector<std::vector<double>>>();
    if(p.size()!=3||r.size()!=3||r[0].size()!=3||r[1].size()!=3||r[2].size()!=3)return {};
    gz::math::Vector3d origin(p[0],p[1],p[2]),axis(r[0][2],r[1][2],r[2][2]);
    if(!origin.IsFinite()||!axis.IsFinite()||axis.Length()<1e-12)return {};
    axis.Normalize();if(axis.Z()>=-1e-6)return {};
    auto h=Height(origin.X(),origin.Y());if(!h)return {};
    for(int i=0;i<32;++i) {
      double t=(*h-origin.Z())/axis.Z();if(!std::isfinite(t)||t<=0)return {};
      auto hit=origin+t*axis;auto next=Height(hit.X(),hit.Y());if(!next)return {};
      if(std::abs(hit.Z()-*next)<=1e-9)return hit;h=next;
    }return {};
  }
 private:std::vector<double>x_,y_;std::vector<std::vector<double>>z_;
};
inline const char *GroundVerdict(double encoder,std::optional<double> travel,double floor=.95,double minimum=.30) {
  if(!std::isfinite(encoder)||encoder<=0||!travel||!std::isfinite(*travel))return "unmeasured";
  if(encoder<minimum)return "short_interval";
  double ratio=*travel/encoder;return ratio<floor||ratio>1/floor?"alert":"pass";
}
}
