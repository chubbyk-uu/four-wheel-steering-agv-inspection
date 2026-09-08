// Production OptixScene kernel, isolated shadow fixture; not vehicle throughput acceptance.
#include "agv_linescan/optix_scene.hpp"
#include "agv_linescan/radiometry_config.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>
using namespace agv_linescan;
using Json=nlohmann::json;
int main(int argc,char** argv){try{
 if(argc!=6 && argc!=8)throw std::runtime_error("usage: benchmark_grooves SCENE ROBOT PTX CAMERA_YAML OUTPUT [REPEATS LED_SAMPLES]");
 unsigned repeats=argc==8?std::stoul(argv[6]):128;
 std::vector<unsigned> counts{4,8,16,32,64};
 if(argc==8)counts={static_cast<unsigned>(std::stoul(argv[7]))};
 if(repeats<32 || repeats>65536 || counts[0]<1 || counts[0]>64)throw std::runtime_error("invalid timing repeats or LED samples");
 std::filesystem::path out=argv[5];
 if(std::filesystem::exists(out))throw std::runtime_error("output exists");
 std::filesystem::create_directories(out);
 auto cfg=YAML::LoadFile(argv[4]);auto sensor=LoadRadiometry(cfg);
 sensor.noise=false;sensor.prnu=0;
 unsigned width=cfg["width"].as<unsigned>(),rows=256;
 auto poly=cfg["ray_polynomial"].as<std::vector<float>>();
 float scale=width*cfg["pixel_pitch_m"].as<float>()/(2*cfg["focal_length_m"].as<float>());
 std::vector<float> rays(width);
 for(unsigned u=0;u<width;++u){float q=(float(u)-(width-1)*.5f)/(width*.5f),v=0;
  for(auto it=poly.rbegin();it!=poly.rend();++it)v=v*q+*it;rays[u]=v*scale;
 }
 std::vector<GridExposure> poses(rows);std::vector<LinkTransform> transforms;
 for(unsigned r=0;r<rows;++r){
  float x=1.3f+(r+.5f)*1.2f/4096;
  for(unsigned k=0;k<3;++k){
   float dx=(int(k)-1)*sensor.exposure*.5f*(10.f/3.6f);
   poses[r].samples[k]={{x+dx,0,sensor.cameraHeight},{0,1,0},{0,0,-1}};
   transforms.push_back({1,0,0,-(x+dx-cfg["camera_x_m"].as<float>()),0,1,0,0,
                        0,0,1,-cfg["base_nominal_height_m"].as<float>()});
  }
 }
 Json report={{"scope","production kernel / fixture geometry and material from scene manifest; no ROS/GZ/archive; timings include synchronous readback, exclude image writing"},{"width",width},{"rows",rows},{"noise",false},{"prnu",false},{"timing_repeats",repeats}};
 for(unsigned count:counts){
  OptixScene gpu(rays,argv[1],argv[2],argv[3],rows,sensor,count);
  for(int w=0;w<8;++w)gpu.Sample(poses,transforms,0);
  std::vector<double> ms;GridBatch batch;std::vector<unsigned char> pixels;
  for(unsigned repeat=0;repeat<repeats;++repeat){
   unsigned block=repeat%32;
   for(unsigned r=0;r<rows;++r)for(unsigned k=0;k<3;++k){
    float x=1.3f+(block*rows+r+.5f)*1.2f/4096+(int(k)-1)*sensor.exposure*.5f*(10.f/3.6f);
    poses[r].samples[k].origin[0]=x;
    transforms[r*3+k][3]=-(x-cfg["camera_x_m"].as<float>());
   }
   auto start=std::chrono::steady_clock::now();batch=gpu.Sample(poses,transforms,block*rows);
   ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
   if(repeat<32)pixels.insert(pixels.end(),batch.pixels.begin(),batch.pixels.end());
  }
  double total=0;for(auto v:ms)total+=v;
  std::sort(ms.begin(),ms.end());auto name="samples_"+std::to_string(count)+".pgm";
  std::ofstream f(out/name,std::ios::binary);f<<"P5\n"<<width<<" "<<rows*32<<"\n255\n";
  f.write(reinterpret_cast<const char*>(pixels.data()),pixels.size());
  report["measurements"].push_back({{"samples",count},{"image",name},
    {"allocated_device_bytes",gpu.AllocatedDeviceBytes()},
    {"median_batch_ms",ms[ms.size()/2]},{"p99_batch_ms",ms[size_t((ms.size()-1)*.99)]},
    {"max_batch_ms",ms.back()},{"total_sample_seconds",total/1000},
    {"active_lines_per_second",double(rows)*repeats*1000/total}});

 }
 std::ofstream(out/"timing.json")<<report.dump(2)<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
