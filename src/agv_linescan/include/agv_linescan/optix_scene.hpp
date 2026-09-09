#pragma once
#include "agv_linescan/cuda_grid.hpp"
#include "agv_linescan/radiometry.hpp"
#include <array>
namespace agv_linescan {
// Row-major world-to-link rigid transform, ephemeral per exposure/link.
using LinkTransform=std::array<float,12>;
class OptixScene {
 public:
  OptixScene(const std::vector<float>& rays,const std::string& scene,
             const std::string& robot,const std::string& ptx,size_t capacity,Radiometry sensor,unsigned lampSamples=4);
  ~OptixScene();
  OptixScene(const OptixScene&)=delete;
  OptixScene& operator=(const OptixScene&)=delete;
  const std::vector<std::string>& Links() const;
  // Owned cudaMalloc bytes + logical CUDA array payload; excludes array padding and driver/context.
  size_t AllocatedDeviceBytes() const;
  std::string MaterialStatistics() const;
  GridBatch Sample(const std::vector<GridExposure>& poses,
                   const std::vector<LinkTransform>& transforms,uint64_t firstGlobalLine,bool prewarm=false);
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
