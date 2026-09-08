#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace agv_linescan {
// Display transfer only: apply AFTER linear area integration, never to raw data.
inline void DisplaySrgb(std::vector<uint8_t>& pixels,double blackLevel=0){
 if(!std::isfinite(blackLevel)||blackLevel<0||blackLevel>=255)throw std::invalid_argument("invalid preview black level");
 std::array<uint8_t,256> lut{};
 for(size_t i=0;i<lut.size();++i){
  const double linear=std::clamp((i-blackLevel)/(255-blackLevel),0.,1.);
  const double display=linear<=.0031308?12.92*linear:1.055*std::pow(linear,1./2.4)-.055;
  lut[i]=static_cast<uint8_t>(std::lround(255*display));
 }
 for(auto& pixel:pixels)pixel=lut[pixel];
}
// Area integration for an overview, never used by calibration/inspection algorithms.
inline std::vector<uint8_t> AreaPreview(const std::vector<uint8_t>& pixels,size_t width,size_t rows,size_t bin=32){
 if(!width||!rows||!bin||pixels.size()!=width*rows)throw std::invalid_argument("invalid preview image");
 const size_t w=(width+bin-1)/bin,h=(rows+bin-1)/bin;std::vector<uint8_t> out(w*h);
 for(size_t y=0;y<h;++y)for(size_t x=0;x<w;++x){
  uint64_t sum=0;size_t count=0;
  for(size_t j=y*bin;j<std::min(rows,(y+1)*bin);++j)for(size_t i=x*bin;i<std::min(width,(x+1)*bin);++i){sum+=pixels[j*width+i];++count;}
  out[y*w+x]=(sum+count/2)/count;
 }return out;
}
}
