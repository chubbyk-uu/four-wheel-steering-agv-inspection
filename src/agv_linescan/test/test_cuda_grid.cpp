#include "agv_linescan/cuda_grid.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace agv_linescan;

TEST(CudaGrid, MetricGridAndDistinctRows) {
  // Independent ideal optics: y = pixel offset * 1.2/4096, height exactly 1 m.
  std::vector<float> rays;
  for (int u=0;u<4096;++u) rays.push_back((u-2047.5)*1.2/4096);
  CudaGrid grid(rays,{});
  std::vector<GridExposure> poses(512);
  for (size_t r=0;r<poses.size();++r) for (auto &p:poses[r].samples)
    p={{float(.025+r*1.2/4096),0,1},{0,1,0},{0,0,-1}};
  auto result=grid.Sample(poses);
  size_t mismatch=0;
  for (size_t r=0;r<poses.size();++r) {
    EXPECT_EQ(result.invalidPerLine[r],0u);
    for (size_t u=0;u<4096;++u) {
      double x=.025+r*1.2/4096, y=(double(u)-2047.5)*1.2/4096;
      bool dark=std::abs(std::remainder(x,.1))<=.001 || std::abs(std::remainder(y,.1))<=.001;
      mismatch+=result.pixels[r*4096+u]!=(dark ? 25 : 210);
    }
  }
  EXPECT_EQ(mismatch,0u);
  // Crossing the transverse grid changes rows: no repeated-row shortcut.
  EXPECT_NE(result.pixels[100],result.pixels[256*4096+100]);
}
TEST(CudaGrid, ExposureAndInvalidGeometry) {
  std::vector<float> rays(4096,.05f);
  CudaGrid grid(rays,{});
  GridExposure e;
  e.samples[0]={{.098f,0,1},{0,1,0},{0,0,-1}};
  e.samples[1]={{.100f,0,1},{0,1,0},{0,0,-1}};
  e.samples[2]={{.102f,0,1},{0,1,0},{0,0,-1}};
  auto result=grid.Sample({e});
  for (auto p:result.pixels) EXPECT_EQ(p,148); // round((210+25+210)/3)
  e.samples[2].down[2]=1; // a single lost exposure sample must not darken valid samples
  EXPECT_THROW(grid.Sample({e}),std::runtime_error);
  for (auto &p:e.samples) p.down[2]=1;
  EXPECT_THROW(grid.Sample({e}),std::runtime_error);
  EXPECT_THROW(grid.Sample(std::vector<GridExposure>(513)),std::invalid_argument);
  EXPECT_TRUE(grid.Sample({}).pixels.empty());
}
TEST(CudaGrid, RotatedRaysAndPartialTail) {
  std::vector<float> rays(4096,.05f);
  CudaGrid grid(rays,{});
  // 90 degree yaw: optical across points -world X; downward remains -Z.
  // Camera y=.03 stays between grid marks, camera x=.05 crosses world x=0.
  GridExposure e;
  for (auto &p:e.samples) p={{.05f,.03f,1},{-1,0,0},{0,0,-1}};
  auto result=grid.Sample(std::vector<GridExposure>(7,e));
  ASSERT_EQ(result.pixels.size(),7*4096u);
  for (auto p:result.pixels) EXPECT_EQ(p,25);
  for (auto &p:e.samples) p.origin[0]=.08;
  result=grid.Sample({e});
  for (auto p:result.pixels) EXPECT_EQ(p,210);
}
