#include "agv_linescan/radiometry_config.hpp"
// Streamed material / geometry optical fixture; independent from whole-vehicle physics.
#include "agv_linescan/optix_scene.hpp"
#include "agv_linescan/sampling.hpp"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc,char **argv) {
  try {
    if (argc!=10) throw std::runtime_error("usage: benchmark_optix_stream CONFIG OUTPUT BLOCKS TARGET_HZ FSYNC_0_OR_1 ROS_0_OR_1 SCENE ROBOT PTX");
    auto c=YAML::LoadFile(argv[1]);
    std::filesystem::path out(argv[2]);
    if (!std::filesystem::create_directory(out)) throw std::runtime_error("output must be a new directory");
    const bool ros=argc>=7 && std::stoi(argv[6]);
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ackSubscription;
    int64_t acknowledged=-1;
    if (ros) {
      rclcpp::init(0,nullptr); node=std::make_shared<rclcpp::Node>("optix_stream_benchmark");
      publisher=node->create_publisher<sensor_msgs::msg::Image>("/benchmark/image",rclcpp::QoS(1).reliable());
      ackSubscription=node->create_subscription<std_msgs::msg::String>("/benchmark/ack",10,
          [&](std_msgs::msg::String::ConstSharedPtr m) {acknowledged=std::stoll(m->data);});
      auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(30);
      while (!publisher->get_subscription_count()) {
        if (std::chrono::steady_clock::now()>deadline) throw std::runtime_error("no benchmark ROS receiver");
        rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    const size_t width=c["width"].as<size_t>(),rows=c["block_rows"].as<size_t>(),blocks=std::stoul(argv[3]);
    const double target=std::stod(argv[4]); const bool durable=std::stoi(argv[5]);
    if (!blocks || blocks>4096 || target<0 || !std::isfinite(target)) throw std::runtime_error("invalid benchmark arguments");
    agv_linescan::Optics optics(width,c["render_width"].as<size_t>(),c["pixel_pitch_m"].as<double>(),
        c["focal_length_m"].as<double>(),c["nominal_width_m"].as<double>(),c["ray_polynomial"].as<std::vector<double>>());
    std::vector<float> rays;
    auto poly=c["ray_polynomial"].as<std::vector<double>>();
    for (size_t u=0;u<width;++u) rays.push_back(optics.scale*agv_linescan::Polynomial(poly,(double(u)-(width-1)/2.)/(width/2.)));
    const size_t batchRows=c["sampling"]["batch_rows"].as<size_t>(256);
    agv_linescan::OptixScene gpu(rays,argv[7],argv[8],argv[9],batchRows,agv_linescan::LoadRadiometry(c),16);
    std::vector<agv_linescan::LinkTransform> transforms;
    std::vector<agv_linescan::GridExposure> poses(batchRows);
    double spacing=c["line_spacing_m"].as<double>(), exposure=c["exposure_s"].as<double>();
    const double sampleRate=target>0 ? target : 11000;
    const auto fixture=c["stream_benchmark"];
    const size_t passBlocks=fixture?fixture["pass_blocks"].as<size_t>(80):80;
    const double startX=fixture?fixture["start_x_m"].as<double>(2):2;
    const double trackY=fixture?fixture["track_y_m"].as<double>(0):0;
    if(!passBlocks||passBlocks>4096||!std::isfinite(startX)||!std::isfinite(trackY))throw std::runtime_error("invalid fixture path");
    const double endX=startX+passBlocks*rows*spacing;
    auto position=[&](size_t index,double fraction) {
      size_t pass=index/(passBlocks*rows),local=index%(passBlocks*rows);
      double direction=pass%2 ? -1 : 1;
      return std::array<double,3>{(direction>0?startX:endX)+direction*(local*spacing+sampleRate*spacing*exposure*fraction),trackY,direction};
    };
    auto fill=[&](size_t first,size_t count) {
      poses.resize(count);transforms.clear();
      for(size_t i=0;i<count;++i) for(int k=0;k<3;++k) {
        auto point=position(first+i,(k+.5)/3);
        poses[i].samples[k]={{float(point[0]),float(point[1]),float(optics.height)},{0,1,0},{0,0,-1}};
        transforms.push_back({1,0,0,-float(point[0]-c["camera_x_m"].as<double>()),0,1,0,-float(trackY),
          0,0,1,-c["base_nominal_height_m"].as<float>()});
      }
    };
    fill(0,batchRows);
    auto warmStart=std::chrono::steady_clock::now();
    for(int i=0;i<5;++i)gpu.Sample(poses,transforms,0);
    double coldWarm=std::chrono::duration<double>(std::chrono::steady_clock::now()-warmStart).count();
    using Clock=std::chrono::steady_clock;
    auto start=Clock::now(); double compute=0,writeTime=0,waitTime=0,rosTime=0;
    std::vector<uint8_t> image; image.reserve(width*rows);
    size_t invalid=0; std::vector<double> batchSeconds,boundarySeconds,blockIntervals;
    double previousComplete=0; int previousTile=-1;
    for (size_t b=0;b<blocks;++b) {
      image.clear();
      for (size_t r=0;r<rows;r+=batchRows) {
        auto t=Clock::now(); fill(b*rows+r,std::min(batchRows,rows-r));
        auto batch=gpu.Sample(poses,transforms,b*rows+r);
        image.insert(image.end(),batch.pixels.begin(),batch.pixels.end());
        for (auto n:batch.invalidPerLine) invalid+=n;
        double dt=std::chrono::duration<double>(Clock::now()-t).count();
        compute+=dt;
        batchSeconds.push_back(dt);
        int tile=int(std::floor(poses[0].samples[1].origin[0]/.512));
        if(tile!=previousTile)boundarySeconds.push_back(dt);
        previousTile=tile;
      }
      auto t=Clock::now();
      auto path=out/("block_"+std::to_string(b)+".pgm");
      std::ofstream stream(path,std::ios::binary);
      stream<<"P5\n"<<width<<' '<<rows<<"\n255\n";
      stream.write(reinterpret_cast<const char*>(image.data()),image.size()); stream.close();
      if (!stream) throw std::runtime_error("PGM write failed");
      if (durable) {
        int fd=open(path.c_str(),O_RDONLY);
        if (fd<0) throw std::runtime_error("archive open failed");
        int result=fsync(fd); close(fd);
        if (result) throw std::runtime_error("archive fsync failed");
      }
      auto tag=[&](size_t row) {
        size_t index=b*rows+row;
        double mid=index/sampleRate+exposure/2;
        auto point=position(index,.5);
        return nlohmann::json{{"global_line",index},{"time_s",mid},
          {"encoder_distance_m",position(index,0)[0]-startX},{"scan_direction",int(point[2])},
          {"camera_position_world_m",{point[0],point[1],optics.height}},
          {"camera_rotation_world",{{0,1,0},{1,0,0},{0,0,-1}}}};
      };
      nlohmann::json tags=nlohmann::json::array();
      for (size_t r=0;r<rows;r+=std::max(size_t(1),rows/4)) tags.push_back(tag(r));
      if (tags.back()["global_line"]!=(b+1)*rows-1) tags.push_back(tag(rows-1));
      nlohmann::json meta={{"block_id",b},{"segment_id",b/passBlocks},{"width",width},{"rows",rows},{"encoding","mono8"},
        {"reference","last_line_exposure_midpoint"},{"first",tag(0)},{"last",tag(rows-1)},
        {"pose_tags",tags},{"line_spacing_m",spacing},{"exposure_s",exposure},
        {"calibration_id",c["calibration_id"].as<std::string>()},
        {"scene_backend","optix_streamed_material_fixture"}};
      std::ofstream metadata(out/("block_"+std::to_string(b)+".json"));
      metadata<<meta.dump()<<'\n'; metadata.close();
      if (!metadata) throw std::runtime_error("metadata write failed");
      writeTime+=std::chrono::duration<double>(Clock::now()-t).count();
      if (ros) {
        t=Clock::now();
        sensor_msgs::msg::Image message;
        message.header.frame_id="cuda_grid_block_"+std::to_string(b);
        int64_t ns=std::llround(((b+1)*rows-1)/sampleRate*1e9+exposure*.5e9);
        message.header.stamp.sec=ns/1000000000; message.header.stamp.nanosec=ns%1000000000;
        message.width=width; message.height=rows; message.step=width; message.encoding="mono8";
        message.data=image; publisher->publish(message);
        while (acknowledged!=int64_t(b)) {
          if (Clock::now()-t>std::chrono::seconds(10)) throw std::runtime_error("ROS block acknowledgement timeout");
          rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        rosTime+=std::chrono::duration<double>(Clock::now()-t).count();
      }
      double completed=std::chrono::duration<double>(Clock::now()-start).count();
      if(b) blockIntervals.push_back(completed-previousComplete);
      previousComplete=completed;
      if (target>0) {
        auto due=start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((b+1)*rows/target));
        t=Clock::now(); std::this_thread::sleep_until(due);
        waitTime+=std::chrono::duration<double>(Clock::now()-t).count();
      }
    }
    double elapsed=std::chrono::duration<double>(Clock::now()-start).count();
    std::sort(batchSeconds.begin(),batchSeconds.end());
    nlohmann::json report={{"backend","optix_streamed_material_fixture"},{"device","OptiX GPU"},
      {"width",width},{"rows_per_block",rows},{"blocks",blocks},{"lines",blocks*rows},
      {"exposure_samples",3},{"batch_rows",batchRows},
      {"invalid_pixels",invalid},{"target_lines_per_second",target},
      {"wall_seconds",elapsed},{"effective_lines_per_wall_second",blocks*rows/elapsed},
      {"active_pipeline_lines_per_second",blocks*rows/(elapsed-waitTime)},
      {"generation_readback_assembly_seconds",compute},{"write_seconds",writeTime},
      {"batch_p99_seconds",batchSeconds[size_t((batchSeconds.size()-1)*.99)]},
      {"file_fsync",durable},{"ros_delivery_included",ros},{"ros_publish_receive_ack_seconds",rosTime},{"ros_acknowledged_blocks",acknowledged+1},{"full_acceptance_passed",false},
      {"note","Static unobstructed grid only; no GZ physics, material or LED radiometry. Optional separate-process ROS receive/ack included. Pacing excluded from active rate; filesystem and device caching still apply."}};
    report["tiles"]=nlohmann::json::parse(gpu.MaterialStatistics());
    report["allocated_device_bytes"]=gpu.AllocatedDeviceBytes();
    report["cold_gpu_warmup_seconds"]=coldWarm;
    report["fixture_path"]={{"start_x_m",startX},{"end_x_m",endX},{"track_y_m",trackY},{"pass_blocks",passBlocks}};
    report["note"]="Shared road geometry, streamed color+normal, roughness 0.60, 16 LED shadow rays per exposure, sensor noise, readback, raw image fsync and separate ROS byte verification. Body/LED fixture, no wheel dynamics or GZ physics; instant direction reversal at fixture path endpoints. No warmup excluded at reversal. No correction or stitching in this benchmark. OS file cache may be warm.";
    report["batch_seconds_max"]=batchSeconds.back();
    report["boundary_batch_seconds_max"]=boundarySeconds.empty()?0:*std::max_element(boundarySeconds.begin(),boundarySeconds.end());
    report["boundary_batches"]=boundarySeconds.size();
    report["block_receive_interval_seconds_max"]=blockIntervals.empty()?0:*std::max_element(blockIntervals.begin(),blockIntervals.end());
    report["continuous_delay_budget"]={{"batch_seconds",double(batchRows)/sampleRate},{"block_interval_seconds",1.5*rows/sampleRate},{"excludes_explicit_lane_start_warmup",false}};
    std::ofstream(out/"summary.json")<<report.dump(2)<<'\n';
    std::ofstream(out/"calibration.yaml")<<YAML::Dump(c)<<'\n';
    std::cout<<report.dump()<<'\n';
    if (ros) rclcpp::shutdown();
    return invalid ? 1 : 0;
  } catch (const std::exception &e) {std::cerr<<e.what()<<'\n'; return 1;}
}
