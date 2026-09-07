#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace agv_linescan {
// World-space camera origin and optical +X/+Z axes; three exposure samples.
// Temporary sampling inputs, never a per-line pose archive.
struct GridPose { float origin[3], across[3], down[3]; };
struct GridExposure { GridPose samples[3]; };
struct GridParameters {
  float spacing=.1f, lineWidth=.002f, extent=20;
  uint8_t dark=25, light=210;
};
// Successful batches contain only fully valid pixels. Any missing exposure sample throws.
struct GridBatch {
  std::vector<uint8_t> pixels;
  std::vector<uint32_t> invalidPerLine;
};
// Explicit unobstructed z=0 grid reference backend. No implicit GZ geometry,
// material, shadow or irradiance support. No CUDA/OpenGL interoperability.
class CudaGrid {
 public:
  CudaGrid(const std::vector<float> &rays, GridParameters parameters, size_t capacity=512);
  ~CudaGrid();
  CudaGrid(const CudaGrid&)=delete;
  CudaGrid& operator=(const CudaGrid&)=delete;
  GridBatch Sample(const std::vector<GridExposure> &poses);
  const std::string &Device() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace agv_linescan
