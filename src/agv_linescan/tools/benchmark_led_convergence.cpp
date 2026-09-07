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
 if(argc!=6)throw std::runtime_error("usage: benchmark_led_convergence SCENE ROBOT PTX CAMERA_YAML OUTPUT");
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
  float x=3.f-.004f+.008f*(r+.5f)/rows;
  for(unsigned k=0;k<3;++k){
   float dx=(int(k)-1)*sensor.exposure*.5f*(10.f/3.6f);
   poses[r].samples[k]={{x+dx,0,sensor.cameraHeight},{0,1,0},{0,0,-1}};
   transforms.push_back({1,0,0,-(x+dx-cfg["camera_x_m"].as<float>()),0,1,0,0,
                        0,0,1,-cfg["base_nominal_height_m"].as<float>()});
  }
 }
 Json report={{"scope","production kernel / synthetic occluder, actual configured lamp mount; no ROS/GZ/archive"},{"width",width},{"rows",rows},{"noise",false},{"prnu",false},{"timing_repeats",8}};
 for(unsigned count:{4,8,16,32,64,128,256}){
  OptixScene gpu(rays,argv[1],argv[2],argv[3],rows,sensor,count);
  gpu.Sample(poses,transforms,0);
  std::vector<double> ms;GridBatch batch;
  for(int repeat=0;repeat<8;++repeat){
   auto start=std::chrono::steady_clock::now();batch=gpu.Sample(poses,transforms,0);
   ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  }
  std::sort(ms.begin(),ms.end());auto name="samples_"+std::to_string(count)+".pgm";
  std::ofstream f(out/name,std::ios::binary);f<<"P5\n"<<width<<" "<<rows<<"\n255\n";
  f.write(reinterpret_cast<const char*>(batch.pixels.data()),batch.pixels.size());
  report["measurements"].push_back({{"samples",count},{"image",name},{"median_batch_ms",ms[4]},{"max_batch_ms",ms.back()},{"median_active_lines_per_second",rows*1000/ms[4]}});
 }
 std::ofstream(out/"timing.json")<<report.dump(2)<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
