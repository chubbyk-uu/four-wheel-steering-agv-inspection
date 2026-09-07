#include "agv_linescan/optix_scene.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <openssl/sha.h>
#include <iomanip>
using namespace agv_linescan;
using Json=nlohmann::json;
std::string hashFile(const std::filesystem::path& p){std::ifstream f(p);std::string s((std::istreambuf_iterator<char>(f)),{});unsigned char h[32];SHA256((const unsigned char*)s.data(),s.size(),h);std::ostringstream o;for(auto c:h)o<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c);return o.str();}
class OptixTest:public ::testing::Test {
 protected:
 std::filesystem::path dir;std::unique_ptr<OptixScene> gpu;Radiometry sensor;
 std::vector<GridExposure> poses;
 LinkTransform away={1,0,0,-5,0,1,0,0,0,0,1,0};
 LinkTransform cover={1,0,0,-.05,0,1,0,-.025,0,0,1,-.4};
 void SetUp()override{
  dir=std::filesystem::temp_directory_path()/("agv_optix_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::filesystem::create_directory(dir);
  std::ofstream(dir/"ground.obj")<<"v -10 -10 0\nv 10 -10 0\nv 10 10 0\nv -10 10 0\nf 1 2 3\nf 1 3 4\n";
  Json m={{"schema","agv.shared.static_scene.v1"},{"units","m"},{"frame","world"},{"transform","identity_world_baked"},{"assets",Json::array({{{"name","terrain"},{"mesh","ground.obj"},{"sha256",hashFile(dir/"ground.obj")},{"triangles",2}}})}};std::ofstream(dir/"scene.json")<<m;
  // A black square, moved independently at every exposure. Exact plane oracle below.
  std::ofstream(dir/"robot.urdf")<<"<robot name=\"fixture\"/>";
  Json group={{"name","occluder"},{"vertices",{{-.08,-.08,0},{.08,-.08,0},{.08,.08,0},{-.08,-.08,0},{.08,.08,0},{-.08,.08,0}}},{"reflectance",{0,0}}};
  Json robot={{"schema","agv.robot.ray_scene.v1"},{"source_urdf","robot.urdf"},{"source_sha256",hashFile(dir/"robot.urdf")},{"groups",Json::array({group})}};std::ofstream(dir/"robot.json")<<robot;
  sensor.noise=false;sensor.prnu=0;sensor.vignette=0;sensor.ledPeak=0;
  gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
  GridExposure p;for(auto& s:p.samples)s={{.05f,.025f,sensor.cameraHeight},{0,1,0},{0,0,-1}};poses={p};
 }
 void TearDown()override{gpu.reset();std::filesystem::remove_all(dir);}
};
TEST_F(OptixTest, ExposureTransformsAndRotatedOccluder){
 auto clear=gpu->Sample(poses,{away,away,away},100).pixels[0];
 auto black=gpu->Sample(poses,{cover,cover,cover},100).pixels[0];
 EXPECT_GT(clear,black);EXPECT_EQ(black,sensor.black);
 float ground=MeanElectrons(sensor,210.f/255,0,1,0,0);
 EXPECT_EQ(clear,SensorCode(sensor,ground,100,0));
 auto mixed=gpu->Sample(poses,{away,cover,cover},100).pixels[0];
 EXPECT_EQ(mixed,SensorCode(sensor,ground/3,100,0));
 // Rotate the square vertical about its center: it no longer intersects the downward ray.
 LinkTransform rotated={0,0,-1,.4f,0,1,0,-.025f,1,0,0,-.08f};
 EXPECT_EQ(gpu->Sample(poses,{rotated,rotated,rotated},100).pixels[0],clear);
}
TEST_F(OptixTest, MissingOneExposureRejectsBatch){
 poses[0].samples[2].origin[0]=20;
 EXPECT_THROW(gpu->Sample(poses,{away,away,away},0),std::runtime_error);
 poses[0].samples[2]=poses[0].samples[0];EXPECT_NO_THROW(gpu->Sample(poses,{away,away,away},0));
}
TEST_F(OptixTest, ExplicitStaticDiffuseReflectance){
 std::ifstream file(dir/"scene.json");Json m;file>>m;
 m["assets"][0]["linear_reflectance"]=.25;std::ofstream(dir/"scene.json")<<m;
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 EXPECT_EQ(gpu->Sample(poses,{away,away,away},0).pixels[0],SensorCode(sensor,MeanElectrons(sensor,.25,0,1,0,0),0,0));
 m["assets"][0]["linear_reflectance"]=-.1;std::ofstream(dir/"scene.json")<<m;
 EXPECT_THROW(OptixScene(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor),std::runtime_error);
}
TEST_F(OptixTest, NoiseIndependentOfWarmupAndPartition){
 sensor.noise=true;sensor.ledPeak=40;
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 std::vector<GridExposure> batch(16,poses[0]);std::vector<LinkTransform> transforms(48,away);
 auto expected=gpu->Sample(batch,transforms,300).pixels;
 gpu->Sample(poses,{cover,cover,cover},0);
 for(size_t i=0;i<16;++i)EXPECT_EQ(gpu->Sample(poses,{away,away,away},300+i).pixels[0],expected[i]);
}
TEST_F(OptixTest, FixtureShadowChangesGroundWithoutPrimaryOcclusion){
 sensor.ledPeak=40;
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 auto clear=gpu->Sample(poses,{away,away,away},0).pixels[0];
 // Square blocks a subset of LED rays at z=.15 but is clear of camera ray at x=.05.
 LinkTransform shade={1,0,0,-.17f,0,1,0,-.025f,0,0,1,-.15f};
 auto dark=gpu->Sample(poses,{shade,shade,shade},0).pixels[0];
 EXPECT_GT(clear,dark+10);EXPECT_GT(dark,sensor.black);
}
TEST_F(OptixTest, EmittingPointsFollowTheirPhysicalLink){
 sensor.ledPeak=40;
 std::ifstream file(dir/"robot.json");Json robot;file>>robot;
 robot["led_emitters"]={{"link","occluder"},{"positions_m",{{-4.70,-.21,.30},{-4.70,-.07,.30},{-4.70,.07,.30},{-4.70,.21,.30}}}};
 std::ofstream(dir/"robot.json")<<robot;
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 auto lit=gpu->Sample(poses,{away,away,away},0).pixels[0];
 auto below=away;below[11]=.5f; // Move the fixture below ground; primary occluder stays out of view.
 auto dark=gpu->Sample(poses,{below,below,below},0).pixels[0];
 EXPECT_GT(lit,dark+50);
 EXPECT_EQ(dark,SensorCode(sensor,MeanElectrons(sensor,210.f/255,0,1,0,0),0,0));
}

TEST_F(OptixTest, RejectsScaleShearAndReflectionAtEveryExposure){
 for(int exposure=0;exposure<3;++exposure){
  for(int variant=0;variant<3;++variant){
   std::vector<LinkTransform> transforms(3,away);
   if(variant==0)transforms[exposure][0]=1.01f;
   if(variant==1)transforms[exposure][1]=.01f;
   if(variant==2)transforms[exposure][0]=-1.f;
   EXPECT_THROW(gpu->Sample(poses,transforms,0),std::invalid_argument);
  }
 }
 EXPECT_NO_THROW(gpu->Sample(poses,{away,away,away},0));
}

TEST_F(OptixTest, LampQuadratureBoundsAndUnshadowedInvariance){
 sensor.ledPeak=40;
 unsigned char expected=0;
 for(unsigned count:{4,8,16,32,64,128,256}){
  OptixScene sampler({0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor,count);
  auto value=sampler.Sample(poses,{away,away,away},0).pixels[0];
  if(count==4)expected=value;
  EXPECT_EQ(value,expected);
 }
 for(unsigned count:{0,257})
  EXPECT_THROW(OptixScene({0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor,count),std::invalid_argument);
}
