#include "agv_linescan/cuda_tiles.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <cmath>
using namespace agv_linescan;
class TilesTest:public ::testing::Test {
 protected:
  std::filesystem::path dir;
  std::vector<float> rays;
  void SetUp() override {
    dir=std::filesystem::temp_directory_path()/("agv_tile_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(dir);
    for(int u=0;u<64;++u) rays.push_back((u-31.5)*.002);
    nlohmann::json c={{"schema","agv.terrain.tiles.v1"},{"encoding","mono8"},{"layout","row_y_column_x"},
      {"gutter_pixels",1},{"plane_z_m",0},{"origin_x_m",0},{"origin_y_m",0},{"length_m",1.6},
      {"width_m",.32},{"texel_m",.01},{"core_pixels",32},{"tiles_x",5},{"tiles_y",1}};
    std::ofstream(dir/"manifest.json")<<c.dump();
    for(int tile=0;tile<5;++tile) {
      std::ofstream f(dir/("tile_"+std::to_string(tile)+"_0.pgm"),std::ios::binary);
      f<<"P5\n34 34\n255\n";
      for(int y=-1;y<=32;++y) for(int x=-1;x<=32;++x) f.put(char(std::clamp(tile*32+x+2*y,0,255)));
    }
  }
  void TearDown() override {std::filesystem::remove_all(dir);}
  std::vector<GridExposure> Poses(double x,std::array<double,2> anchor) {
    std::vector<GridExposure> p(2);
    for(int r=0;r<2;++r) for(auto& s:p[r].samples)
      s={{float(x+r*.002-anchor[0]),float(.16-anchor[1]),1},{0,1,0},{0,0,-1}};
    return p;
  }
};
TEST_F(TilesTest, BilinearGutterContinuityAndEviction) {
  CudaTiles gpu(rays,(dir/"manifest.json").string(),512,4);
  auto a=gpu.Anchor(.319,.16);auto poses=Poses(.319,a);
  gpu.Prefetch(poses,a,{.7,0});gpu.Warm(poses,a);
  auto out=gpu.Sample(poses,a,0);
  ASSERT_EQ(out.pixels.size(),128u);
  for(int r=0;r<2;++r) {
    EXPECT_EQ(out.invalidPerLine[r],0u);
    for(int u=0;u<64;++u) {
      double expected=(.319+r*.002)/.01-.5+2*((.16+rays[u])/.01-.5);
      EXPECT_NEAR(out.pixels[r*64+u],std::nearbyint(expected),1);
    }
  }
  a=gpu.Anchor(1.4,.16);poses=Poses(1.4,a);
  EXPECT_THROW(gpu.Sample(poses,a,0),std::runtime_error); // cannot fabricate a missing tile
  gpu.Prefetch(poses,a,{0,0});gpu.Warm(poses,a);
  out=gpu.Sample(poses,a,0);
  EXPECT_EQ(out.invalidPerLine[0],0u);
  auto stats=nlohmann::json::parse(gpu.Statistics());
  EXPECT_EQ(stats["required_tile_misses"],1);
  EXPECT_GT(stats["tile_loads"].get<int>(),4);
}
TEST_F(TilesTest, MissingFileIsExplicitFailure) {
  std::filesystem::remove(dir/"tile_4_0.pgm");
  CudaTiles gpu(rays,(dir/"manifest.json").string(),512,4);
  auto a=gpu.Anchor(1.4,.16);auto poses=Poses(1.4,a);
  gpu.Prefetch(poses,a,{0,0});
  EXPECT_THROW(gpu.Warm(poses,a),std::runtime_error);
}

TEST_F(TilesTest, RadiometryExposureShadowSaturationAndNoise) {
  Radiometry c;c.enabled=true;c.noise=false;c.prnu=0;c.vignette=0;c.cameraHeight=1;
  c.shadowStart=0;c.shadowEnd=2;c.shadowTransmission=1;
  auto capture=[&](Radiometry settings) {
    CudaTiles gpu(rays,(dir/"manifest.json").string(),512,4,settings);
    auto a=gpu.Anchor(.319,.16);auto poses=Poses(.319,a);
    gpu.Prefetch(poses,a,{0,0});gpu.Warm(poses,a);
    auto result=gpu.Sample(poses,a,0);
    EXPECT_EQ(result.invalidPerLine[0],0u);return result.pixels;
  };
  auto sun=capture(c);
  c.shadowTransmission=0;auto shade=capture(c);
  for(size_t i=0;i<sun.size();++i) {
    EXPECT_GE(sun[i],shade[i]);EXPECT_LE(sun[i]-shade[i],2);
  }
  c.shadowTransmission=1;c.exposure=5e-6f;auto shortExposure=capture(c);
  for(size_t i=0;i<sun.size();++i) EXPECT_NEAR((sun[i]-c.black)*.25f,shortExposure[i]-c.black,1);
  c.exposure=20e-6f;c.ledPeak=0;c.ambient=0;
  auto dark=capture(c);for(auto pixel:dark) EXPECT_EQ(pixel,4);
  c.ledPeak=40000;auto saturated=capture(c);for(auto pixel:saturated) EXPECT_EQ(pixel,255);
  c.ledPeak=40;c.ambient=1;c.noise=true;
  auto noisy=capture(c);EXPECT_EQ(noisy,capture(c));EXPECT_NE(noisy,sun);
}

TEST_F(TilesTest, NoiseIsIndependentOfBatchPartition) {
  Radiometry c;c.enabled=true;c.cameraHeight=1;
  CudaTiles a(rays,(dir/"manifest.json").string(),512,4,c),b(rays,(dir/"manifest.json").string(),512,4,c);
  auto anchor=a.Anchor(.319,.16);auto poses=Poses(.319,anchor);
  for(auto* gpu:{&a,&b}) {gpu->Prefetch(poses,anchor,{0,0});gpu->Warm(poses,anchor);}
  b.Sample(poses,anchor,777); // kernel warmup must not advance acquisition noise
  auto whole=a.Sample(poses,anchor,100).pixels;
  auto first=b.Sample({poses[0]},anchor,100).pixels,second=b.Sample({poses[1]},anchor,101).pixels;
  first.insert(first.end(),second.begin(),second.end());EXPECT_EQ(whole,first);
}

TEST(Radiometry, FullWidthDominanceFiniteBandAndSensorStatistics) {
  Radiometry c;c.enabled=true;c.Validate();
  float minimum=100;
  for(int u=0;u<4096;++u) {
    float q=(u-2047.5f)/2048.f;
    float across=.6f*(q+.04f*q*q*q);
    float led=LedIrradiance(c,0,across,c.cameraHeight,1);
    minimum=std::min(minimum,led);EXPECT_GE(led,20);
  }
  EXPECT_GT(minimum,33);
  EXPECT_LT(LedIrradiance(c,.16f,0,c.cameraHeight,1),.001);
  EXPECT_GT(LedIrradiance(c,.04f,0,c.cameraHeight,1),20);
  c.prnu=0;c.vignette=0;
  double sum=0,squared=0;const int count=100000;
  for(int i=0;i<count;++i) {double value=SensorCode(c,6000,i,100);sum+=value;squared+=value*value;}
  double mean=sum/count,variance=squared/count-mean*mean;
  EXPECT_NEAR(mean,129.5,.03);
  // Shot + read noise propagated to DN, plus rounding variance.
  double expected=(6000+9)*std::pow(251./12000,2)+1./12;
  EXPECT_NEAR(variance,expected,.06);
  c.exposure=-1;EXPECT_THROW(c.Validate(),std::invalid_argument);
}

TEST_F(TilesTest, PartialExposureOutsideTerrainRejectsBatchAndCanRetry) {
 CudaTiles gpu(rays,(dir/"manifest.json").string(),512,4);
 auto a=gpu.Anchor(.319,.16);auto poses=Poses(.319,a);
 gpu.Prefetch(poses,a,{0,0});gpu.Warm(poses,a);
 auto broken=poses;broken[0].samples[2].origin[1]=2;
 EXPECT_THROW(gpu.Sample(broken,a,123),std::runtime_error);
 EXPECT_EQ(gpu.Sample(poses,a,123).invalidPerLine[0],0u);
}
TEST(Radiometry, FixedReferenceDoesNotCancelPowerChanges) {
 Radiometry c;c.prnu=0;c.vignette=0;
 auto baseline=MeanElectrons(c,.5f,40,1,0,0);
 EXPECT_NEAR(MeanElectrons(c,.5f,81,1,0,0),baseline*2,.01);
 c.referenceIrradiance=82;
 EXPECT_NEAR(MeanElectrons(c,.5f,81,1,0,0),baseline,.01);
 c.referenceIrradiance=0;EXPECT_THROW(c.Validate(),std::invalid_argument);
}
