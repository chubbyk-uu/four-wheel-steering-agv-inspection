#pragma once
#include "agv_linescan/cuda_grid.hpp"
#include <array>
#include "agv_linescan/radiometry.hpp"

namespace agv_linescan {
// Disk-backed, bilinear Mono8 plane. Resident slots are pinned during sampling;
// a separate worker reads and uploads future tiles. Missing required data throws.
class CudaTiles {
 public:
  CudaTiles(const std::vector<float>& rays, const std::string& manifest,
            size_t capacity=512, size_t slots=24, Radiometry radiometry={});
  ~CudaTiles();
  CudaTiles(const CudaTiles&)=delete;
  CudaTiles& operator=(const CudaTiles&)=delete;
  std::array<double,2> Anchor(double x,double y) const;
  void Prefetch(const std::vector<GridExposure>& localPoses,
                std::array<double,2> anchor, std::array<double,2> ahead);
  // Only before capture / between explicitly separate scan segments.
  void Warm(const std::vector<GridExposure>& localPoses, std::array<double,2> anchor);
  GridBatch Sample(const std::vector<GridExposure>& localPoses, std::array<double,2> anchor, uint64_t firstGlobalLine);
  std::string Statistics() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
