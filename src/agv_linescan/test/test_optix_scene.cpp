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

// Hardware texture interpolation oracle under ambient-only illumination, independent of BRDF.
TEST_F(OptixTest, GroundTextureSamplingIntegrityAndCoverage){
 gpu.reset();sensor.ambient=41;
 auto writeMap=[&](const std::string& name,const std::vector<unsigned char>& bytes){
  std::ofstream f(dir/name,std::ios::binary);f.write((const char*)bytes.data(),bytes.size());f.close();
  return Json{{"file",name},{"sha256",hashFile(dir/name)}};
 };
 auto color=writeMap("color.raw",{0,64,128,255});
 std::ifstream input(dir/"scene.json");Json m;input>>m;
 m["assets"][0]["material"]="ground";
 m["ground_material"]={{"schema","agv.ground_material.xy.v1"},{"width",2},{"height",2},
   {"origin_xy_m",{0,0}},{"span_xy_m",{1,1}},{"color",color},{"roughness",.8}};
 auto makeSampler=[&](){std::ofstream(dir/"scene.json")<<m;
  return std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);};
 gpu=makeSampler();
 for(auto& s:poses[0].samples){s.origin[0]=.5f;s.origin[1]=.5f;}
 auto code=gpu->Sample(poses,{away,away,away},0).pixels[0];
 EXPECT_NEAR(code,SensorCode(sensor,MeanElectrons(sensor,(0+64+128+255)/(4.f*255),0,41,0,0),0,0),1);
 for(auto& s:poses[0].samples){s.origin[0]=.75f;s.origin[1]=.75f;}
 EXPECT_EQ(gpu->Sample(poses,{away,away,away},0).pixels[0],SensorCode(sensor,MeanElectrons(sensor,1,0,41,0,0),0,0));
 poses[0].samples[1].origin[0]=1.1f;
 EXPECT_THROW(gpu->Sample(poses,{away,away,away},0),std::runtime_error);
 m["ground_material"]["color"]["sha256"]="incorrect";
 EXPECT_THROW(makeSampler(),std::runtime_error);
 m["ground_material"]["color"]=writeMap("color.raw",{0,64,128});
 EXPECT_THROW(makeSampler(),std::runtime_error);
}

TEST_F(OptixTest, GroundNormalAndRoughnessAffectDirectLight){
 gpu.reset();sensor.ledPeak=40;
 auto writeMap=[&](const std::string& name,const std::vector<unsigned char>& bytes){
  std::ofstream f(dir/name,std::ios::binary);f.write((const char*)bytes.data(),bytes.size());f.close();
  return Json{{"file",name},{"sha256",hashFile(dir/name)}};
 };
 std::ifstream input(dir/"scene.json");Json m;input>>m;m["assets"][0]["material"]="ground";
 m["ground_material"]={{"schema","agv.ground_material.xy.v1"},{"width",1},{"height",1},
  {"origin_xy_m",{-1,-1}},{"span_xy_m",{2,2}},{"color",writeMap("color.raw",{128})},{"roughness",.8}};
 auto sample=[&](){std::ofstream(dir/"scene.json")<<m;
  OptixScene sampler({0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor,16);
  return sampler.Sample(poses,{away,away,away},0).pixels[0];};
 auto base=sample();
 m["ground_material"]["normal"]=writeMap("normal.raw",{128,128});
 EXPECT_NEAR(sample(),base,1);
 m["ground_material"]["normal"]=writeMap("normal.raw",{204,128});
 EXPECT_NE(sample(),base);
 m["ground_material"].erase("normal");
 m["ground_material"]["roughness_map"]=writeMap("rough.raw",{204});
 EXPECT_EQ(sample(),base); // .8 constant map and .8 uniform parameter agree.
 m["ground_material"]["roughness_map"]=writeMap("rough.raw",{26});
 EXPECT_NE(sample(),base);
 m["ground_material"]["span_xy_m"]={0,1};
 EXPECT_THROW(sample(),std::runtime_error);
}

TEST_F(OptixTest, StreamedMaterialSeamsEvictionReverseAndCorruption){
 gpu.reset();sensor.ambient=41;
 auto writeMap=[&](const std::string& name,const std::vector<unsigned char>& bytes){
  std::ofstream f(dir/name,std::ios::binary);f.write((const char*)bytes.data(),bytes.size());f.close();
  return Json{{"file",name},{"sha256",hashFile(dir/name)}};
 };
 std::ifstream input(dir/"scene.json");Json m;input>>m;m["assets"][0]["material"]="ground";
 Json material={{"schema","agv.ground_material.tiles.v1"},{"tiles_x",4},{"tiles_y",1},
  {"core_pixels",4},{"gutter_pixels",1},{"texel_m",.1},{"origin_xy_m",{0,0}},
  {"height_bounds_m",{0,0}},{"roughness",.6},{"cache_slots",2},
  {"prefetch_ahead_m",0},{"prefetch_behind_m",0},{"required_wait_timeout_s",1},
  {"tiles",Json::array()}};
 for(int tile=0;tile<4;++tile){
  std::vector<unsigned char> color,normal;
  for(int y=-1;y<5;++y)for(int x=-1;x<5;++x){
   color.push_back(32+8*(tile*4+x));normal.push_back(128);normal.push_back(128);
  }
  material["tiles"].push_back({{"ix",tile},{"iy",0},
   {"color",writeMap("tile"+std::to_string(tile)+".raw",color)},
   {"normal",writeMap("normal"+std::to_string(tile)+".raw",normal)}});
 }
 m["ground_material"]=material;std::ofstream(dir/"scene.json")<<m;
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 for(float x:{.2f,.399f,.4f,.401f,.8f,1.2f,1.4f,1.2f,.8f,.4f,.2f}){
  for(auto& p:poses[0].samples){p.origin[0]=x;p.origin[1]=.2f;}
  float reflectance=(32+8*(x/.1f-.5f))/255.f;
  EXPECT_NEAR(gpu->Sample(poses,{away,away,away},33).pixels[0],
   SensorCode(sensor,MeanElectrons(sensor,reflectance,0,41,0,0),33,0),1);
  EXPECT_EQ(Json::parse(gpu->MaterialStatistics())["slot_pins"],0);
 }
 auto stats=Json::parse(gpu->MaterialStatistics());EXPECT_GT(stats["evictions"].get<int>(),0);EXPECT_EQ(stats["cache_slots"],2);
 // First tile is resident; corrupt a future tile and require it after recreation.
 gpu.reset();std::ofstream(dir/"tile3.raw",std::ios::binary)<<"bad";
 gpu=std::make_unique<OptixScene>(std::vector<float>{0},(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor);
 EXPECT_NO_THROW(gpu->Sample(poses,{away,away,away},33));
 for(auto& p:poses[0].samples)p.origin[0]=1.4f;
 EXPECT_THROW(gpu->Sample(poses,{away,away,away},33),std::runtime_error);
 EXPECT_EQ(Json::parse(gpu->MaterialStatistics())["slot_pins"],0);
}

TEST_F(OptixTest, ConfiguredFallbackLampMatchesExplicitEmitters) {
 // Same finite lamp and shadow caster, represented once through the fixture
 // fallback and once through explicit robot-local emitter coordinates.
 sensor.ledPeak=40;
 const float tilt=std::atan2(sensor.ledForward,sensor.ledHeight);
 std::vector<float> rays;for(int i=0;i<128;++i)rays.push_back((i-63.5f)/128);
 Json robot;{std::ifstream f(dir/"robot.json");f>>robot;}
 for(auto& v:robot["groups"][0]["vertices"]){v[0]=v[0].get<float>()+.20f;v[2]=.15f;}
 LinkTransform identity={1,0,0,0,0,1,0,0,0,0,1,0};
 std::vector<uint8_t> previous;
 for(float length:{.60f,1.20f}) {
  sensor.ledLength=length;robot.erase("led_emitters");
  std::ofstream(dir/"robot.json")<<robot;
  gpu=std::make_unique<OptixScene>(rays,(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor,8);
  auto fallback=gpu->Sample(poses,{identity,identity,identity},0).pixels;
  Json points=Json::array();
  for(int i=0;i<4;++i)points.push_back({.05f+sensor.ledForward-.022f*std::sin(tilt),
    .025f+(length-.04f)*((i+.5f)/4-.5f),sensor.ledHeight-.022f*std::cos(tilt)});
  robot["led_emitters"]={{"link","occluder"},{"positions_m",points}};
  std::ofstream(dir/"robot.json")<<robot;
  gpu=std::make_unique<OptixScene>(rays,(dir/"scene.json").string(),(dir/"robot.json").string(),AGV_TEST_OPTIX_PTX,16,sensor,8);
  auto explicitLamp=gpu->Sample(poses,{identity,identity,identity},0).pixels;
  ASSERT_EQ(fallback.size(),explicitLamp.size());
  for(size_t i=0;i<fallback.size();++i)EXPECT_LE(std::abs(int(fallback[i])-int(explicitLamp[i])),1);
  if(!previous.empty()){EXPECT_NE(previous,fallback);}
  previous=fallback;
 }
}
