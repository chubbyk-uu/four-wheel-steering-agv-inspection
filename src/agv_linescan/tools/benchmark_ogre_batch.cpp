// Narrow-line multi-view experiment: shared STATIC scene, distinct pose per exposure.
// This is deliberately not a replacement for the encoder/GZ system plugin.
#include "agv_linescan/sampling.hpp"
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Light.hh>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
using Clock=std::chrono::steady_clock;
using Json=nlohmann::json;
double Elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
int main(int argc,char** argv) {
 try {
  if(argc!=8) throw std::runtime_error("CONFIG OUTPUT BATCH_ROWS TOTAL_ROWS FLUSH_PASSES LED_COUNT READBACK_0_OR_1");
  auto config=YAML::LoadFile(argv[1]);const std::filesystem::path out(argv[2]);
  const size_t batch=std::stoul(argv[3]),total=std::stoul(argv[4]);
  const int flush=std::stoi(argv[5]),lightCount=std::stoi(argv[6]);const bool copy=std::stoi(argv[7]);
  if(!batch || batch>16 || !total || total%batch || flush<1 || flush>255 || lightCount<0 || lightCount>21)
    throw std::invalid_argument("invalid experiment arguments");
  if(!std::filesystem::create_directory(out)) throw std::runtime_error("output must be new");
  agv_linescan::Optics optics(config["width"].as<size_t>(),config["render_width"].as<size_t>(),
    config["pixel_pitch_m"].as<double>(),config["focal_length_m"].as<double>(),
    config["nominal_width_m"].as<double>(),config["ray_polynomial"].as<std::vector<double>>());
  const double spacing=config["line_spacing_m"].as<double>(),exposure=config["exposure_s"].as<double>();
  const double speed=20./3.6,startX=.1,centerX=startX+total*spacing/2;
  auto init=Clock::now();
  auto* engine=gz::rendering::engine("ogre2");
  if(!engine) throw std::runtime_error("Ogre2 initialization failed");
  auto scene=engine->CreateScene("agv_batch_experiment");
  scene->SetAmbientLight(.5,.5,.5);scene->SetBackgroundColor(.8,.85,.9);
  scene->SetCameraPassCountPerGpuFlush(flush);
  auto box=[&](const std::string& name,gz::math::Vector3d position,gz::math::Vector3d scale,double gray,double yaw) {
    auto v=scene->CreateVisual(name);auto material=scene->CreateMaterial(name+"_mat");
    material->SetAmbient(gray,gray,gray);material->SetDiffuse(gray,gray,gray);material->SetCastShadows(false);
    v->AddGeometry(scene->CreateBox());v->SetLocalScale(scale);v->SetLocalPosition(position);
    v->SetLocalRotation(0,0,yaw);v->SetMaterial(material);scene->RootVisual()->AddChild(v);
  };
  box("floor",{0,0,-.005},{4,4,.01},.65,0);
  // A diagonal fiducial moves across the line by about one pixel per encoder event.
  // It exposes stale/repeated camera poses even when axis-aligned grid rows repeat.
  box("diagonal",{0,0,.00015},{3,.0015,.0003},.015,M_PI/4);
  auto sun=scene->CreateDirectionalLight();sun->SetDirection(-.5,.2,-1);sun->SetDiffuseColor(.8,.8,.8);
  sun->SetCastShadows(true);scene->RootVisual()->AddChild(sun);
  const double forward=config["led_forward_offset_m"].as<double>(),height=config["led_height_m"].as<double>();
  const double tilt=std::atan(forward/height),lx=centerX+forward-.022*std::sin(tilt),lz=height-.022*std::cos(tilt);
  for(int i=0;i<lightCount;++i) {
    double fraction=lightCount==1?0:2.*i/(lightCount-1)-1,ly=.28*fraction;
    auto lamp=scene->CreateSpotLight();lamp->SetLocalPosition(lx,ly,lz);
    lamp->SetDirection(gz::math::Vector3d(centerX-lx,.8*config["radiometry"]["led_half_width_m"].as<double>()*fraction-ly,-lz).Normalized());
    lamp->SetDiffuseColor(1,1,1);lamp->SetSpecularColor(1,1,1);
    lamp->SetIntensity(config["gz_led"]["intensity_at_reference"].as<double>());
    lamp->SetAttenuationRange(2);lamp->SetAttenuationConstant(1);lamp->SetAttenuationLinear(0);lamp->SetAttenuationQuadratic(0);
    double angle=std::atan(config["radiometry"]["led_half_depth_m"].as<double>()/std::hypot(height,forward));
    lamp->SetInnerAngle(1.2*angle);lamp->SetOuterAngle(2*angle);lamp->SetFalloff(1);lamp->SetCastShadows(true);
    scene->RootVisual()->AddChild(lamp);
  }
  std::vector<gz::rendering::CameraPtr> cameras;std::vector<gz::rendering::Image> images;
  for(size_t i=0;i<batch*3;++i) {
    auto c=scene->CreateCamera("line_"+std::to_string(i));scene->RootVisual()->AddChild(c);
    c->SetImageWidth(optics.renderWidth);c->SetImageHeight(1);c->SetImageFormat(gz::rendering::PF_R8G8B8);
    c->SetAntiAliasing(0);c->SetHFOV(2*std::atan(optics.span));c->SetAspectRatio(optics.renderWidth);
    c->SetNearClipPlane(.01);c->SetFarClipPlane(30);images.push_back(c->CreateImage());cameras.push_back(c);
  }
  double initialization=Elapsed(init),poseSeconds=0,preSeconds=0,renderSeconds=0,postSeconds=0,copySeconds=0,mapSeconds=0;
  std::vector<double> batches;std::vector<uint8_t> output(total*optics.width);
  auto run=[&](size_t first,bool measured) {
    auto begin=Clock::now(),t=begin;
    for(size_t r=0;r<batch;++r) for(size_t k=0;k<3;++k) {
      double x=startX+(first+r)*spacing+speed*exposure*(k+.5)/3;
      cameras[r*3+k]->SetWorldPose(gz::math::Pose3d(x,0,optics.height,0,M_PI/2,M_PI));
    }
    double pose=Elapsed(t);t=Clock::now();scene->PreRender();double pre=Elapsed(t);t=Clock::now();
    for(auto& c:cameras)c->Render();double render=Elapsed(t);t=Clock::now();
    for(auto& c:cameras)c->PostRender();scene->PostRender();double post=Elapsed(t);t=Clock::now();
    if(copy) for(size_t i=0;i<cameras.size();++i)cameras[i]->Copy(images[i]);
    double read=Elapsed(t);t=Clock::now();
    if(copy) for(size_t r=0;r<batch;++r) for(size_t u=0;u<optics.width;++u) {
      double value=0;size_t lo=optics.source[u],hi=std::min(lo+1,optics.renderWidth-1);double f=optics.source[u]-lo;
      for(size_t k=0;k<3;++k) {
        auto rgb=images[r*3+k].Data<unsigned char>();
        auto gray=[&](size_t j){return .2126*rgb[j*3]+.7152*rgb[j*3+1]+.0722*rgb[j*3+2];};
        value+=((1-f)*gray(lo)+f*gray(hi))/3;
      }
      output[(first+r)*optics.width+u]=std::clamp(std::lround(value),0L,255L);
    }
    double map=Elapsed(t);
    if(measured) {poseSeconds+=pose;preSeconds+=pre;renderSeconds+=render;postSeconds+=post;
      copySeconds+=read;mapSeconds+=map;batches.push_back(Elapsed(begin));}
  };
  auto warm=Clock::now();for(int i=0;i<3;++i)run(0,false);double warmSeconds=Elapsed(warm);
  auto begin=Clock::now();for(size_t first=0;first<total;first+=batch)run(first,true);
  auto drain=Clock::now();
  if(!copy)cameras.back()->Copy(images.back()); // End fence: count completed GPU work, not just submissions.
  double drainSeconds=Elapsed(drain),wall=Elapsed(begin);
  if(copy) {std::ofstream image(out/"scan.pgm",std::ios::binary);image<<"P5\n"<<optics.width<<" "<<total<<"\n255\n";image.write(reinterpret_cast<char*>(output.data()),output.size());}
  Json result={{"scope","Ogre2 STATIC scene, independent camera pose per temporal sample; no physics, moving light/object interpolation, ROS or timed archive"},
    {"batch_rows",batch},{"camera_count",cameras.size()},{"total_rows",total},{"width",optics.width},{"render_width",optics.renderWidth},
    {"exposure_samples",3},{"flush_pass_limit",flush},{"led_count",lightCount},{"readback_enabled",copy},
    {"initialization_seconds",initialization},{"warmup_seconds",warmSeconds},{"wall_seconds",wall},{"lines_per_wall_second",total/wall},
    {"pose_seconds",poseSeconds},{"scene_pre_seconds",preSeconds},{"camera_render_seconds",renderSeconds},{"scene_post_seconds",postSeconds},
    {"final_gpu_drain_seconds",drainSeconds},{"readback_seconds",copySeconds},{"mapping_seconds",mapSeconds},{"batch_max_seconds",*std::max_element(batches.begin(),batches.end())},
    {"start_x_m",startX},{"speed_m_s",speed},{"line_spacing_m",spacing},{"exposure_s",exposure},{"full_acceptance_passed",false}};
  std::ofstream(out/"summary.json")<<result.dump(2)<<'\n';std::ofstream(out/"calibration.yaml")<<YAML::Dump(config)<<'\n';
  std::cout<<result.dump()<<'\n';cameras.clear();sun.reset();engine->DestroyScene(scene);scene.reset();gz::rendering::unloadEngine("ogre2");
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
