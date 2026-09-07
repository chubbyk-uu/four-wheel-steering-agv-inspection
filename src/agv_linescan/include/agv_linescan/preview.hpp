#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace agv_linescan {
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
