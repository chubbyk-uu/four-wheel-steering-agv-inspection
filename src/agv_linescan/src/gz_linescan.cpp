#include "agv_linescan/radiometry_config.hpp"
// Encoder-triggered camera owned by a GZ system, not a standard camera sensor.
// GL work stays on one thread. Ogre2 uses synchronized scene sampling; the
// tiled CUDA plane uses a bounded asynchronous queue to keep physics advancing.
#include "agv_linescan/sampling.hpp"
#include "agv_linescan/scan_motion.hpp"
#include "agv_linescan/queue_budget.hpp"
#include "agv_linescan/preview.hpp"
#ifdef AGV_HAS_CUDA
#include "agv_linescan/cuda_grid.hpp"
#include "agv_linescan/cuda_tiles.hpp"
#endif
#ifdef AGV_HAS_OPTIX
#include "agv_linescan/optix_scene.hpp"
#endif
#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <mutex>
#include <thread>
#include <sstream>
#include <gz/math/Matrix3.hh>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/rendering/RenderUtil.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace agv_linescan {
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
double Seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
struct WheelMotion {bool valid=false;double zMin=0,zMax=0,lastZ=0,maxStep=0;gz::math::Vector3d localMin,localMax;};
struct Sample { double time; gz::math::Pose3d pose; std::vector<gz::math::Pose3d> links; };
struct Line { Event event; std::array<gz::math::Pose3d,3> exposure; gz::math::Pose3d midpoint; uint64_t sequence=0; std::array<std::vector<gz::math::Pose3d>,3> links; };
struct Job { std::vector<Line> lines; std::string endReason; double sceneTime=0; bool warm=false; bool holdBoundary=false; };

class GzLineScan final: public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate {
 public:
  ~GzLineScan() override {
    { std::lock_guard<std::mutex> lock(mutex_); quit_=true; }
    wake_.notify_one();
    if (renderThread_.joinable()) renderThread_.join();
    for (auto &write:writes_) { try { write.get(); } catch (...) {} }
    if (context_) context_->shutdown("line-scan system destroyed");
  }

  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    const auto configPath=sdf->Get<std::string>("config");
    config_=YAML::LoadFile(configPath);
    wheelSpreadAbsolute_=config_["wheel_speed_spread_absolute_m_s"].as<double>(.01);
    wheelSpreadRelative_=config_["wheel_speed_spread_relative"].as<double>(0);
    WheelSpeedSpreadLimit(0,wheelSpreadAbsolute_,wheelSpreadRelative_);
    backend_=sdf->Get<std::string>("backend","render").first;
    if (backend_!="render" && backend_!="cuda_grid" && backend_!="cuda_tiles" && backend_!="optix") throw std::runtime_error("unknown sampling backend");
#ifndef AGV_HAS_CUDA
    if (backend_!="render") throw std::runtime_error("cuda_grid requires building with a CUDA toolkit");
#endif
#ifndef AGV_HAS_OPTIX
    if(backend_=="optix") throw std::runtime_error("OptiX backend not built; configure AGV_OPTIX_SDK");
#endif
    if(backend_=="optix") {
      if(!LoadRadiometry(config_).enabled)throw std::runtime_error("OptiX strip backend requires radiometry.enabled=true");
      scenePath_=sdf->Get<std::string>("scene_manifest");robotPath_=sdf->Get<std::string>("robot_manifest");ptxPath_=sdf->Get<std::string>("optix_ptx");
      std::ifstream file(robotPath_);Json robot;file>>robot;
      if(!robot.contains("led_emitters"))throw std::runtime_error("robot export lacks physical LED emitters");
      for(auto g:robot.at("groups"))linkNames_.push_back(g.at("name").get<std::string>());
      config_["calibration_id"]=config_["calibration_id"].as<std::string>()+"-optix-strip-v1";
      std::ifstream sceneFile(scenePath_);Json scene;sceneFile>>scene;
      sceneContract_=scene;robotContract_={{"source_sha256",robot.at("source_sha256")},{"cylinder_facets",robot.at("cylinder_facets")},{"link_names",linkNames_}};
    }
    if (backend_=="cuda_tiles") terrainPath_=sdf->Get<std::string>("terrain_manifest");
    auto sampling=config_["sampling"];
    batchRows_=sampling["batch_rows"].as<size_t>(256);
    tileSlots_=sampling["tile_cache_slots"].as<size_t>(24);
    prefetchDistance_=sampling["prefetch_distance_m"].as<double>(4);
    queueBudget_=std::make_unique<QueueBudget>(sampling["queue_capacity_lines"].as<size_t>(2048),
        sampling["queue_capacity_jobs"].as<size_t>(512),sampling["queue_max_wait_s"].as<double>(.15));
    if(!batchRows_ || batchRows_>16384 || tileSlots_<4 || tileSlots_>64 ||
       queueBudget_->lineLimit<batchRows_ || queueBudget_->lineLimit>1048576 ||
       queueBudget_->jobLimit>65536 || !std::isfinite(prefetchDistance_) || prefetchDistance_<0)
      throw std::runtime_error("invalid sampling capacity configuration");
    auto platform=YAML::LoadFile(sdf->Get<std::string>("platform"));
    radius_=platform["wheel_radius"].as<double>();
    track_=platform["track"].as<double>();
    wheelbase_=platform["wheelbase"].as<double>();
    rows_=sdf->Get<size_t>("block_rows",config_["block_rows"].as<size_t>()).first;
    config_["block_rows"]=rows_;
    exposure_=config_["exposure_s"].as<double>();
    // Read immutable configuration once, not for every exposure/link SLERP.
    maxSampleGap_=config_["max_sample_gap_s"].as<double>();
    if(!std::isfinite(maxSampleGap_) || maxSampleGap_<=0)
      throw std::runtime_error("invalid maximum pose sample gap");
    spacing_=config_["line_spacing_m"].as<double>();
    maxSpeed_=sdf->Get<double>("max_scan_speed",.25).first;
    if (!rows_ || exposure_<=0 || maxSpeed_<=0 || rows_>16384)
      throw std::runtime_error("invalid acquisition configuration");
    optics_=std::make_unique<Optics>(config_["width"].as<size_t>(),
        config_["render_width"].as<size_t>(10240), config_["pixel_pitch_m"].as<double>(),
        config_["focal_length_m"].as<double>(),config_["nominal_width_m"].as<double>(),
        config_["ray_polynomial"].as<std::vector<double>>());
    trigger_=std::make_unique<Trigger>(spacing_);
    offset_=gz::math::Pose3d(config_["camera_x_m"].as<double>(),0,
        optics_->height-config_["base_nominal_height_m"].as<double>(),0,0,0);
    output_=std::filesystem::path(sdf->Get<std::string>("output_dir")) /
        ("session_cpp_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(output_);
    if(backend_=="optix") {
      std::filesystem::copy_file(robotPath_,output_/"robot_scene.json");
      std::filesystem::copy_file(std::filesystem::path(robotPath_).parent_path()/"robot.urdf",output_/"robot.urdf");
      std::filesystem::copy_file(scenePath_,output_/"scene_manifest.json");
    }
    std::ofstream(output_/"calibration.yaml")<<YAML::Dump(config_)<<'\n';
    buffer_.reserve(rows_*optics_->width);
    context_=std::make_shared<rclcpp::Context>(); context_->init(0,nullptr);
    auto options=rclcpp::NodeOptions().context(context_).use_global_arguments(false);
    node_=std::make_shared<rclcpp::Node>("gz_linescan_camera",options);
    rclcpp::ExecutorOptions eo; eo.context=context_;
    executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor_->add_node(node_);
    imagePub_=node_->create_publisher<sensor_msgs::msg::Image>("/linescan/image_raw",2);
    previewPub_=node_->create_publisher<sensor_msgs::msg::Image>("/linescan/image_preview",2);
    metaPub_=node_->create_publisher<std_msgs::msg::String>("/linescan/block_metadata",10);
    statusPub_=node_->create_publisher<std_msgs::msg::String>("/linescan/status",10);
    statePub_=node_->create_publisher<std_msgs::msg::String>("/linescan/state",10);
    motionSub_=node_->create_subscription<std_msgs::msg::String>("/motion_state",10,
        [this](std_msgs::msg::String::ConstSharedPtr m) { motion_=m->data; });
    enableService_=node_->create_service<std_srvs::srv::SetBool>("/linescan/set_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          if(req->data && !RecoverSampler()) {
            res->success=false;res->message="sampler recovery failed; see status and repair terrain/storage";return;
          }
          Dispatch(Job{{},"capture_toggle",now_});
          if(terrainFailed_) {res->success=false;res->message="terrain sampling failed; see /linescan/status and restart after fixing the data";return;}
          Reset("capture_toggle"); enabled_=false;
          if(req->data && AsyncBackend()) {
            if(history_.empty()) {res->success=false;res->message="wait for robot pose before preparing terrain";return;}
            Line warm;warm.midpoint=history_.back().pose*offset_;
            warm.exposure.fill(warm.midpoint);warm.links.fill(history_.back().links);warm.event={now_,0,lastDriveSpeed_<0?-1:1};
            Dispatch(Job{{warm},"",now_,true});
            if(terrainFailed_) {res->success=false;res->message="terrain prefetch failed; capture remains disabled";return;}
          }
          enabled_=req->data;
          // Disabling acknowledges only after all completed blocks reach storage.
          if (!enabled_) DrainWrites();
          res->success=true; res->message=output_.string();
        });
    wheelMotion_.resize(linkNames_.size());
    renderThread_=std::thread([this] { RenderLoop(); });
    { std::unique_lock<std::mutex> lock(mutex_);
      done_.wait(lock,[this]{return ready_ || error_;});
      if (error_) std::rethrow_exception(error_);
    }
    RCLCPP_INFO(node_->get_logger(),"C++ GZ line-scan archive: %s",output_.c_str());
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override {
    if (model_==gz::sim::kNullEntity) {
      model_=ecm.EntityByComponents(gz::sim::components::Name("agv"));
      if (model_==gz::sim::kNullEntity) return;
      for(const auto& name:linkNames_) {
        auto entity=ecm.EntityByComponents(gz::sim::components::Name(name),gz::sim::components::ParentEntity(model_));
        if(!entity)throw std::runtime_error("missing ray-scene link: "+name);
        linkEntities_.push_back(entity);
      }
      const std::array<std::string,4> names={"fl","fr","rl","rr"};
      for (size_t i=0;i<4;++i) {
        drive_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_drive_joint"),gz::sim::components::ParentEntity(model_));
        steer_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_steer_joint"),gz::sim::components::ParentEntity(model_));
        if (!drive_[i] || !steer_[i]) throw std::runtime_error("missing AGV joint");
        for (auto entity:{drive_[i],steer_[i]}) {
          if (!ecm.Component<gz::sim::components::JointPosition>(entity))
            ecm.CreateComponent(entity,gz::sim::components::JointPosition());
          if (!ecm.Component<gz::sim::components::JointVelocity>(entity))
            ecm.CreateComponent(entity,gz::sim::components::JointVelocity());
        }
      }
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    now_=Seconds(info.simTime);
    auto wallNow=Clock::now();
    { std::lock_guard<std::mutex> metricsLock(metricsMutex_);
    if(enabled_ && active_ && !info.paused) {
      if(stepWallValid_) {
        double gap=Seconds(wallNow-stepWall_);stepGapMax_=std::max(stepGapMax_,gap);
        ++stepGaps_[std::min(size_t(999),size_t(gap*10000))];++stepGapCount_;
      }
      stepWall_=wallNow;stepWallValid_=true;
    } else stepWallValid_=false;
    }
    // Copy ECM changes before the render thread consumes them; no concurrent ECM access.
    if (backend_=="render") render_.UpdateFromECM(info,ecm);
    {std::lock_guard<std::mutex> lock(mutex_);HandleTerrainError();}
    executor_->spin_some();
    if (Clock::now()-lastStatePublish_>=std::chrono::milliseconds(50)) {
      Json state={{"time_s",now_},{"enabled",enabled_},{"sampling_active",active_}};
      std_msgs::msg::String message;message.data=state.dump();statePub_->publish(message);
      lastStatePublish_=Clock::now();
    }
    if (info.paused) return;
    Job job; job.sceneTime=now_;
    if (model_) {
      if (!history_.empty() && now_<=history_.back().time) {
        job.endReason="time_reset"; Reset(job.endReason); history_.clear(); projectedEncoder_.Reset();
      }
      Sample snapshot{now_,gz::sim::worldPose(model_,ecm),{}};
      for(auto entity:linkEntities_)snapshot.links.push_back(gz::sim::worldPose(entity,ecm));
      if(enabled_ && active_) {
        std::lock_guard<std::mutex> lock(metricsMutex_);
        for(size_t i=0;i<linkNames_.size();++i)if(linkNames_[i].find("_wheel_link")!=std::string::npos){
          auto& stat=wheelMotion_[i];double z=snapshot.links[i].Pos().Z();
          auto local=(snapshot.pose.Inverse()*snapshot.links[i]).Pos();
          if(!stat.valid){stat.valid=true;stat.zMin=stat.zMax=stat.lastZ=z;stat.localMin=stat.localMax=local;}
          stat.zMin=std::min(stat.zMin,z);stat.zMax=std::max(stat.zMax,z);
          stat.maxStep=std::max(stat.maxStep,std::abs(z-stat.lastZ));stat.lastZ=z;
          for(size_t j=0;j<3;++j){stat.localMin[j]=std::min(stat.localMin[j],local[j]);stat.localMax[j]=std::max(stat.localMax[j],local[j]);}
        }
      }
      history_.push_back(std::move(snapshot));
      while (history_.size()>8) history_.pop_front();
      bool valid=true;
      double distance=0,maxSteer=0;
      std::array<double,4> speeds{},angles{},wheelDistance{};
      for (size_t i=0;i<4;++i) {
        auto p=ecm.Component<gz::sim::components::JointPosition>(drive_[i]);
        auto v=ecm.Component<gz::sim::components::JointVelocity>(drive_[i]);
        auto s=ecm.Component<gz::sim::components::JointPosition>(steer_[i]);
        if (!p || !v || !s || p->Data().empty() || v->Data().empty() || s->Data().empty()) { valid=false; break; }
        maxSteer=std::max(maxSteer,std::abs(s->Data()[0]));
        distance+=p->Data()[0]*radius_/4;
        angles[i]=s->Data()[0];wheelDistance[i]=p->Data()[0]*radius_;
        speeds[i]=v->Data()[0]*radius_;
        valid=valid && std::isfinite(distance) && std::isfinite(speeds[i]) &&
            std::isfinite(s->Data()[0]) && (config_["projected_encoder"].as<bool>(false) || std::abs(s->Data()[0])<=config_["max_steer_rad"].as<double>()) &&
            std::abs(speeds[i])<=maxSpeed_;
      }
      lastDriveSpeed_=(speeds[0]+speeds[1]+speeds[2]+speeds[3])/4;
      const auto range=std::minmax_element(speeds.begin(),speeds.end());
      const double spreadLimit=WheelSpeedSpreadLimit(std::isfinite(lastDriveSpeed_)?lastDriveSpeed_:0,wheelSpreadAbsolute_,wheelSpreadRelative_);
      const bool wheelDataValid=valid;
      valid=valid && *range.second-*range.first<=spreadLimit &&
          std::abs((speeds[1]+speeds[3]-speeds[0]-speeds[2])/(2*track_))<=config_["max_yaw_rate_rad_s"].as<double>();
      if(config_["projected_encoder"].as<bool>(false)) {
        try {
          if(!wheelDataValid) {projectedEncoder_.Reset();throw std::runtime_error("invalid wheel data");}
          auto velocity=FitScanVelocity(speeds,angles,wheelbase_,track_);
          distance=projectedEncoder_.Update(wheelDistance,angles);
          lastDriveSpeed_=velocity.vx;
          valid=wheelDataValid && std::abs(velocity.vx)<=maxSpeed_ &&
              std::abs(velocity.vy)<=config_["max_scan_lateral_m_s"].as<double>() &&
              std::abs(velocity.wz)<=config_["max_yaw_rate_rad_s"].as<double>() &&
              velocity.residual<=config_["max_scan_residual_m_s"].as<double>();
          scanMotion_={{"vx",velocity.vx},{"vy",velocity.vy},{"wz",velocity.wz},{"residual",velocity.residual}};
        } catch(const std::exception&) {valid=false;}
      }
      if(enabled_ && !valid && !config_["projected_encoder"].as<bool>(false)) scanMotion_={{"wheel_speeds_m_s",speeds},{"wheel_speed_spread_m_s",*range.second-*range.first},
        {"max_abs_steer_rad",maxSteer},{"encoder_yaw_estimate_rad_s",(speeds[1]+speeds[3]-speeds[0]-speeds[2])/(2*track_)},
        {"speed_limit_m_s",maxSpeed_},{"wheel_spread_limit_m_s",spreadLimit},
        {"steer_limit_rad",config_["max_steer_rad"].as<double>()},{"yaw_limit_rad_s",config_["max_yaw_rate_rad_s"].as<double>()}};
      // Start a pass only after initial wheel alignment. Once started, the
      // armed camera keeps its partial frame and encoder phase in HOLD/ALIGN.
      if (enabled_ && (motion_=="DRIVE" || (active_ &&
          (motion_=="BRAKE" || motion_=="HOLD" || motion_=="ALIGN"))) && valid) {
        try {
          auto events=trigger_->Update(now_,distance);
          pending_.insert(pending_.end(),events.begin(),events.end());
          active_=true;
          while (!pending_.empty() && pending_.front().time+exposure_<=now_) {
            auto event=pending_.front(); pending_.pop_front();
            Line line; line.event=event;
            for (size_t i=0;i<3;++i) {
              double t=event.time+exposure_*(i+.5)/3;
              line.exposure[i]=PoseAt(t)*offset_;
              for(size_t j=0;j<linkNames_.size();++j)line.links[i].push_back(PoseAt(t,j));
            }
            line.midpoint=PoseAt(event.time+exposure_/2)*offset_;
            line.sequence=nextCaptureLine_++;
            job.lines.push_back(line);
          }
          if (motion_=="HOLD" && !holding_ && pending_.empty()) {
            job.holdBoundary=true; holding_=true;
          } else if (motion_!="HOLD") holding_=false;
        } catch (const std::exception &error) {
          job.endReason=error.what(); Reset(job.endReason);
        }
      } else if (active_) {
        job.endReason=valid ? "motion_"+motion_ : "unsupported_scan_motion";
        Reset(job.endReason);
      }
    }
    // ECM changes are already buffered. No need to wake GL / update the
    // rendering graph on physics steps with no exposure or segment boundary.
    if (!job.lines.empty() || !job.endReason.empty() || job.holdBoundary) {
      if(AsyncBackend() && job.endReason.empty() && !job.holdBoundary) QueueCuda(std::move(job));
      else Dispatch(std::move(job));
    }
  }

 private:
  bool AsyncBackend() const {return backend_=="cuda_tiles" || backend_=="optix";}

  gz::math::Pose3d PoseAt(double t,size_t link=SIZE_MAX) const {
    for (size_t i=1;i<history_.size();++i) {
      const auto &a=history_[i-1],&b=history_[i];
      if (t>=a.time && t<=b.time) {
        if (b.time-a.time>maxSampleGap_) throw std::runtime_error("pose_gap");
        return Interpolate(link==SIZE_MAX?a.pose:a.links.at(link),link==SIZE_MAX?b.pose:b.links.at(link),(t-a.time)/(b.time-a.time));
      }
    }
    throw std::runtime_error("pose_outside_history");
  }

  void Reset(const std::string &reason) {
    Json event={{"reason",reason},{"simulation_time_s",now_},{"discarded_pending_lines",pending_.size()}};
    if(reason=="unsupported_scan_motion")event["motion_condition"]=scanMotion_;
    std::ofstream(output_/"events.jsonl",std::ios::app)<<event.dump()<<'\n';
    std_msgs::msg::String message; message.data=event.dump(); statusPub_->publish(message);
    pending_.clear(); trigger_->Reset(); active_=false; holding_=false;
  }

  bool HandleTerrainError() {
    if(!error_ || !AsyncBackend()) return false;
    enabled_=false;
    if(!terrainFailed_) {
      terrainFailed_=true;wake_.notify_all();
      std::string reason="terrain_sampling_failure";
      try {std::rethrow_exception(error_);} catch(const std::exception& e) {reason+=std::string(": ")+e.what();}
      Reset(reason);
    }
    return true;
  }

  bool RecoverSampler() {
    if(!AsyncBackend()) return true;
    {std::lock_guard<std::mutex> lock(mutex_);if(!error_)return true;HandleTerrainError();}
    wake_.notify_all();
    if(renderThread_.joinable())renderThread_.join();
    // A failed writer future cannot be reused. The previous failure is retained in events.
    for(auto &write:writes_) {try{write.get();}catch(...) {}}
    writes_.clear();
    {std::lock_guard<std::mutex> lock(mutex_);
      Json event={{"reason","sampler_recovery"},{"simulation_time_s",now_},
        {"discarded_queued_lines",queueBudget_->lines},{"next_capture_line",nextCaptureLine_}};
      std::ofstream(output_/"events.jsonl",std::ios::app)<<event.dump()<<'\n';
      queuedJobs_.clear();queueBudget_->Clear();error_=nullptr;terrainFailed_=false;busy_=false;ready_=false;
      buffer_.clear();tags_=Json::array();++segment_;
    }
    renderThread_=std::thread([this]{RenderLoop();});
    std::unique_lock<std::mutex> lock(mutex_);done_.wait(lock,[this]{return ready_||error_;});
    if(error_){HandleTerrainError();return false;}return true;
  }

  void QueueCuda(Job job) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(HandleTerrainError()) return;
    try {queueBudget_->Push(job.lines.size(),Seconds(Clock::now().time_since_epoch()));}
    catch(...) {
      error_=std::current_exception();
      HandleTerrainError();
      return;
    }
    queuedJobs_.push_back(std::move(job));

    wake_.notify_one();
  }

  void Dispatch(Job job) {
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock,[this]{return !busy_ || error_;});
    if(HandleTerrainError()) return;
    if (error_) std::rethrow_exception(error_);
    job_=std::move(job); busy_=true; wake_.notify_one();
    done_.wait(lock,[this]{return !busy_ || error_;});
    if(HandleTerrainError()) return;
    if (error_) std::rethrow_exception(error_);
  }

  void RenderLoop() {
#ifdef AGV_HAS_CUDA
    if (backend_!="render") { CudaLoop(); return; }
#endif
    try {
      render_.SetEngineName("ogre2"); render_.SetSceneName("agv_linescan_scene");
      // Use the configured WSL D3D12 OpenGL context; do not silently select software/EGL.
      render_.Init();
      auto scene=render_.Scene();
      if (!scene) throw std::runtime_error("no Ogre2 scene");
      // Submit all three exposure views together before any GPU readback.
      scene->SetCameraPassCountPerGpuFlush(6);
      std::vector<gz::rendering::Image> images;
      for (size_t i=0;i<3;++i) {
        auto camera=scene->CreateCamera("agv_scanline_"+std::to_string(i));
        scene->RootVisual()->AddChild(camera);
        camera->SetImageWidth(optics_->renderWidth); camera->SetImageHeight(1);
        camera->SetImageFormat(gz::rendering::PF_R8G8B8);
        camera->SetAntiAliasing(0); camera->SetHFOV(2*std::atan(optics_->span));
        camera->SetAspectRatio(optics_->renderWidth);
        camera->SetNearClipPlane(.01); camera->SetFarClipPlane(30);
        images.push_back(camera->CreateImage()); cameras_.push_back(camera);
      }
      { std::lock_guard<std::mutex> lock(mutex_); ready_=true; }
      done_.notify_one();
      RCLCPP_INFO(node_->get_logger(),"Ogre2 scanline renderer ready: %zux1 -> %zu pixels",optics_->renderWidth,optics_->width);
      while (true) {
        Job job;
        { std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait(lock,[this]{return busy_ || quit_;});
          if (quit_) break;
          job=std::move(job_);
        }
        auto sceneStart=Clock::now(); render_.Update();
        sceneSeconds_+=Seconds(Clock::now()-sceneStart);
        for (const auto &line:job.lines) {
          std::vector<double> accumulated(optics_->width,0);
          for (size_t i=0;i<3;++i) {
            const auto &pose=line.exposure[i];
            // GZ local +X looks down; image-right is body +Y.
            cameras_[i]->SetWorldPose(gz::math::Pose3d(pose.Pos(),pose.Rot()*gz::math::Quaterniond(0,M_PI/2,M_PI)));
          }
          auto start=Clock::now();
          scene->PreRender();
          for (auto &camera:cameras_) camera->Render();
          for (auto &camera:cameras_) camera->PostRender();
          scene->PostRender();
          renderSeconds_+=Seconds(Clock::now()-start);
          for (size_t i=0;i<3;++i) {
            start=Clock::now(); cameras_[i]->Copy(images[i]);
            readSeconds_+=Seconds(Clock::now()-start);
            start=Clock::now();
            const auto rgb=images[i].Data<unsigned char>();
            for (size_t u=0;u<optics_->width;++u) {
              const auto lo=static_cast<size_t>(optics_->source[u]);
              const auto hi=std::min(lo+1,optics_->renderWidth-1);
              const double fraction=optics_->source[u]-lo;
              auto gray=[rgb](size_t p) {return .2126*rgb[3*p]+.7152*rgb[3*p+1]+.0722*rgb[3*p+2];};
              accumulated[u]+=((1-fraction)*gray(lo)+fraction*gray(hi))/3;
            }
            mappingSeconds_+=Seconds(Clock::now()-start); ++renders_;
          }
          std::vector<uint8_t> pixels; pixels.reserve(optics_->width);
          for (double p:accumulated) pixels.push_back(static_cast<uint8_t>(std::clamp(std::lround(p),0L,255L)));
          AddLine(pixels.data(),line,job.sceneTime);
        }
        if (job.holdBoundary) MarkHoldBoundary();
        if (!job.endReason.empty()) { Flush(job.endReason); ++segment_; }
        { std::lock_guard<std::mutex> lock(mutex_); busy_=false; }
        done_.notify_one();
      }
      Flush("shutdown"); DrainWrites(); cameras_.clear(); render_.Destroy();
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_); error_=std::current_exception(); busy_=false;
      done_.notify_all();
    }
  }

#ifdef AGV_HAS_CUDA
  void CudaLoop() {
    try {
      std::vector<float> rays;
      auto poly=config_["ray_polynomial"].as<std::vector<double>>();
      for (size_t u=0;u<optics_->width;++u)
        rays.push_back(optics_->scale*Polynomial(poly,(double(u)-(optics_->width-1)/2.)/(optics_->width/2.)));
      GridParameters p;
      p.spacing=config_["grid_spacing_m"].as<float>(); p.lineWidth=config_["grid_line_width_m"].as<float>();
      p.extent=config_["grid_extent_m"].as<float>();
      p.dark=config_["grid_dark"].as<int>(); p.light=config_["grid_light"].as<int>();
      std::unique_ptr<CudaGrid> gpu;
      std::unique_ptr<CudaTiles> terrain;
#ifdef AGV_HAS_OPTIX
      std::unique_ptr<OptixScene> optix;
      if(backend_=="optix") {
        optix=std::make_unique<OptixScene>(rays,scenePath_,robotPath_,ptxPath_,batchRows_,LoadRadiometry(config_));
        if(optix->Links()!=linkNames_)throw std::runtime_error("OptiX link order mismatch");
      }
      else
#endif
      if(backend_=="cuda_tiles") terrain=std::make_unique<CudaTiles>(rays,terrainPath_,batchRows_,tileSlots_,LoadRadiometry(config_));
      else gpu=std::make_unique<CudaGrid>(rays,p,batchRows_);
      RCLCPP_INFO(node_->get_logger(),"GPU sampler ready: %s",backend_.c_str());
      { std::lock_guard<std::mutex> lock(mutex_); ready_=true; } done_.notify_one();
      std::vector<Line> lines; std::vector<double> sceneTimes;
      auto sample=[&](bool warm=false) {
        if (lines.empty()) return;
        auto start=Clock::now();
        std::array<double,2> anchor={0,0};
        if(terrain) anchor=terrain->Anchor(lines.front().midpoint.Pos().X(),lines.front().midpoint.Pos().Y());
        std::vector<GridExposure> poses(lines.size());
        for (size_t r=0;r<lines.size();++r) for (size_t k=0;k<3;++k) {
          const auto &pose=lines[r].exposure[k];
          auto across=pose.Rot().RotateVector(gz::math::Vector3d(0,1,0));
          auto down=pose.Rot().RotateVector(gz::math::Vector3d(0,0,-1));
          auto &p=poses[r].samples[k];
          for (size_t j=0;j<3;++j) {p.origin[j]=pose.Pos()[j]-(j<2?anchor[j]:0); p.across[j]=across[j]; p.down[j]=down[j];}
        }
        if(terrain) {
          auto forward=lines.back().midpoint.Rot().RotateVector(gz::math::Vector3d(lines.back().event.direction,0,0));
          terrain->Prefetch(poses,anchor,{prefetchDistance_*forward.X(),prefetchDistance_*forward.Y()});
          if(warm) {
            terrain->Warm(poses,anchor);terrain->Sample(poses,anchor,0);
            tileStatistics_=Json::parse(terrain->Statistics());
            lines.clear();sceneTimes.clear();return;
          }
        }
        for(size_t r=0;r<lines.size();++r) if(lines[r].sequence!=lines.front().sequence+r)
          throw std::runtime_error("noncontiguous capture IDs within sampling batch");
        GridBatch result;
#ifdef AGV_HAS_OPTIX
        if(optix) {
          std::vector<LinkTransform> transforms;transforms.reserve(lines.size()*3*linkNames_.size());
          for(const auto& line:lines)for(const auto& exposure:line.links) {
            if(exposure.size()!=linkNames_.size())throw std::runtime_error("missing exposure link poses");
            for(const auto& pose:exposure) {
              auto inverse=pose.Inverse();gz::math::Matrix3d rotation(inverse.Rot());LinkTransform m;
              for(size_t row=0;row<3;++row){for(size_t col=0;col<3;++col)m[row*4+col]=rotation(row,col);m[row*4+3]=inverse.Pos()[row];}
              transforms.push_back(m);
            }
          }
          result=optix->Sample(poses,transforms,warm?0:lines.front().sequence,warm);
          tileStatistics_=Json::parse(optix->MaterialStatistics());
          if(warm){lines.clear();sceneTimes.clear();return;}
        } else
#endif
        result=terrain?terrain->Sample(poses,anchor,lines.front().sequence):gpu->Sample(poses);
        if(terrain) tileStatistics_=Json::parse(terrain->Statistics());
        double elapsed=Seconds(Clock::now()-start);
        batchMax_=std::max(batchMax_,elapsed);renderSeconds_+=elapsed; renders_+=lines.size()*3;
        for (size_t r=0;r<lines.size();++r) {
          invalidPixels_+=result.invalidPerLine[r];
          AddLine(result.pixels.data()+r*optics_->width,lines[r],sceneTimes[r]);
        }
        lines.clear(); sceneTimes.clear();
      };
      bool stoppedByFailure=false;
      while (true) {
        Job job; bool synchronous=false;
        { std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait(lock,[this]{return busy_ || !queuedJobs_.empty() || quit_ || terrainFailed_;});
          if(terrainFailed_) {stoppedByFailure=true;break;}
          if ((quit_ || terrainFailed_) && queuedJobs_.empty() && !busy_) {stoppedByFailure=terrainFailed_;break;}
          if(!queuedJobs_.empty()) {queueBudget_->Pop(Seconds(Clock::now().time_since_epoch()));job=std::move(queuedJobs_.front());queuedJobs_.pop_front();}
          else {job=std::move(job_);synchronous=true;}
        }
        for (const auto &line:job.lines) {
          lines.push_back(line); sceneTimes.push_back(job.sceneTime);
          if (lines.size()==batchRows_ || buffer_.size()/optics_->width+lines.size()==rows_) sample();
        }
        if(job.warm) sample(true);
        if(job.holdBoundary) {sample(); MarkHoldBoundary();}
        if (!job.endReason.empty()) {sample(); Flush(job.endReason); ++segment_;}
        if(synchronous) {{std::lock_guard<std::mutex> lock(mutex_);busy_=false;} done_.notify_one();}
      }
      sample(); Flush(stoppedByFailure?"sampling_failure":"shutdown"); DrainWrites();
    } catch (...) {
      auto failure=std::current_exception();
      try {Flush("sampling_failure");DrainWrites();} catch(...) {}
      std::lock_guard<std::mutex> lock(mutex_); error_=failure; busy_=false; done_.notify_all();
    }
  }
#endif

  Json Tag(const Line &line,double sceneTime) {
    const auto &p=line.midpoint.Pos();
    const auto q=line.midpoint.Rot()*gz::math::Quaterniond(M_PI,0,M_PI/2);
    gz::math::Matrix3d r(q);
    return {{"time_s",line.event.time+exposure_/2},{"global_line",line.sequence},
      {"encoder_distance_m",line.event.distance},{"scan_direction",line.event.direction},
      {"camera_position_world_m",{p.X(),p.Y(),p.Z()}},
      {"camera_rotation_world",{{r(0,0),r(0,1),r(0,2)},{r(1,0),r(1,1),r(1,2)},{r(2,0),r(2,1),r(2,2)}}},
      {"scene_snapshot_time_s",sceneTime}};
  }

  void MarkHoldBoundary() {
    // Sparse tags on both sides prevent interpolation across stopped time.
    if (!buffer_.empty()) {
      auto tag=Tag(lastLine_,lastSceneTime_);
      if (tags_.back()["global_line"]!=tag["global_line"]) tags_.push_back(std::move(tag));
    }
    tagAfterHold_=true;
  }

  void AddLine(const uint8_t *pixels,const Line &line,double sceneTime) {
    const size_t row=buffer_.size()/optics_->width;
    if (row%std::max(size_t(1),rows_/4)==0 || tagAfterHold_) {
      auto tag=Tag(line,sceneTime);
      if (!row) first_=tag;
      tags_.push_back(std::move(tag));
    }
    tagAfterHold_=false;
    lastLine_=line; lastSceneTime_=sceneTime;
    buffer_.insert(buffer_.end(),pixels,pixels+optics_->width);
    ++globalLine_;
    if (row+1==rows_) Flush("full");
  }

  void DrainWrites() {
    for (auto &write:writes_) write.get();
    writes_.clear();
  }

  void Flush(const std::string &reason) {
    if (buffer_.empty()) return;
    last_=Tag(lastLine_,lastSceneTime_);
    if (tags_.back()["global_line"]!=last_["global_line"]) tags_.push_back(last_);
    Json meta={{"encoder_distance_model",config_["projected_encoder"].as<bool>(false)?"four_wheel_body_x_projection":"mean_signed_wheel_distance"},{"schema","agv.linescan.render.v1"},{"block_id",block_++},
      {"segment_id",segment_},{"width",optics_->width},{"rows",buffer_.size()/optics_->width},
      {"encoding","mono8"},{"first",first_},{"last",last_},{"pose_tags",tags_},
      {"reference","last_line_exposure_midpoint"},{"line_spacing_m",spacing_},
      {"exposure_s",exposure_},{"end_reason",reason},{"calibration_id",config_["calibration_id"].as<std::string>()},
      {"scene_backend",backend_=="render" ? "gz_ogre2_cpp" : "cuda_unobstructed_grid_plane"},{"exposure_render_batch",3},{"gpu_flush_pass_limit",6},{"pose_source","simulation_truth_sensor_generation_only"},
      {"scene_time_model","latest_physics_step_scene; camera pose interpolated per exposure sample"},
      {"radiometry","rendered RGB luminance; three temporal samples; not calibrated irradiance"},
      {"cumulative_sampling_metrics",{{"lines",globalLine_},{"renders",renders_},{"render_seconds",renderSeconds_},
        {"readback_seconds",readSeconds_},{"mapping_seconds",mappingSeconds_},{"scene_update_seconds",sceneSeconds_}}}};
    if(config_["projected_encoder"].as<bool>(false))
      meta["scan_motion_limits"]={{"max_lateral_m_s",config_["max_scan_lateral_m_s"].as<double>()},
        {"max_yaw_rate_rad_s",config_["max_yaw_rate_rad_s"].as<double>()},
        {"max_wheel_vector_residual_m_s",config_["max_scan_residual_m_s"].as<double>()},
        {"brake_sampling",true}};
    meta["cumulative_sampling_metrics"]["sampling_batch_max_seconds"]=batchMax_;
    { std::lock_guard<std::mutex> metricsLock(metricsMutex_);
    for(size_t i=0;i<wheelMotion_.size();++i){const auto& w=wheelMotion_[i];if(w.valid)
      meta["wheel_motion_diagnostics"][linkNames_[i]]={{"z_min_m",w.zMin},{"z_max_m",w.zMax},{"max_z_step_m",w.maxStep},
        {"local_position_range_m",{w.localMax.X()-w.localMin.X(),w.localMax.Y()-w.localMin.Y(),w.localMax.Z()-w.localMin.Z()}}};}

    meta["cumulative_sampling_metrics"]["physics_step_wall_interval_max_seconds"]=stepGapMax_;
    size_t cumulative=0;
    for(size_t i=0;i<stepGaps_.size();++i) {
      cumulative+=stepGaps_[i];
      if(cumulative>=std::max(size_t(1),size_t(std::ceil(stepGapCount_*.99)))) {
        meta["cumulative_sampling_metrics"]["physics_step_wall_interval_p99_upper_seconds"]=(i+1)*.0001;break;
      }
    }
    }
    if(AsyncBackend()) {
      std::lock_guard<std::mutex> lock(mutex_);
      meta["sampling_queue_capacity_jobs"]=queueBudget_->jobLimit;
      meta["sampling_queue_capacity_lines"]=queueBudget_->lineLimit;
      meta["sampling_queue_high_water_jobs"]=queueBudget_->peakJobs;
      meta["sampling_queue_high_water_lines"]=queueBudget_->peakLines;
      meta["sampling_queue_max_wait_s"]=queueBudget_->ageLimit;
      meta["sampling_queue_observed_wait_max_s"]=queueBudget_->peakAge;
    }
    if (backend_!="render") {
      if(backend_=="cuda_tiles") {
        meta["scene_backend"]="cuda_tiled_plane";meta["terrain_manifest"]=terrainPath_;
        meta["tile_statistics"]=tileStatistics_;
      }
      meta["invalid_pixels"]=invalidPixels_; invalidPixels_=0;
      meta["cuda_batch_rows"]=batchRows_;
      meta["invalid_sample_policy"]="reject entire batch; no partial exposure pixels archived"; meta["gpu_flush_pass_limit"]=nullptr;
      meta["scene_time_model"]="static z=0 plane; per-exposure camera poses from GZ physics";
      meta["radiometry"]="grid or bilinear Mono8 texture; geometric exposure only; no occlusion or LED irradiance";
      if(backend_=="cuda_tiles" && config_["radiometry"] && config_["radiometry"]["enabled"].as<bool>(false)) {
        meta["radiometry"]="relative_projected_strip_v1: linear reflectance, moving finite LED band, analytic ambient shadow, exposure integration, vignette, PRNU, approximate shot/read noise, full-well and ADC clipping; no geometry occlusion";
        meta["radiometry_config_yaml"]=YAML::Dump(config_["radiometry"]);
      }
    }
    if(backend_=="optix") {
      meta["scene_backend"]="optix_shared_mesh_dynamic_robot";
      meta["tile_statistics"]=tileStatistics_;
      meta["scene_contract"]=sceneContract_;meta["robot_contract"]=robotContract_;
      meta["scene_time_model"]="static shared world; camera and robot links interpolated per exposure from physics snapshots";
      meta["radiometry"]="optix_relative_strip_v1: projected LED band with four actual fixture shadow rays; scene-declared grid or material, diffuse robot colors; ambient constant; not absolute photometry";
      meta["radiometry_config_yaml"]=YAML::Dump(config_["radiometry"]);
      meta["dynamic_pose_archive"]="ephemeral per exposure; block archive contains sparse camera tags only";
    }
    while (!writes_.empty() && writes_.front().wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      writes_.front().get(); writes_.pop_front();
    }
    // Bounded two-block writer queue: block physics under storage pressure, never discard.
    if (writes_.size()>=2) { writes_.front().get(); writes_.pop_front(); }
    auto pixels=std::move(buffer_); buffer_.clear(); buffer_.reserve(rows_*optics_->width); tags_=Json::array();
    writes_.push_back(std::async(std::launch::async,[this,pixels=std::move(pixels),meta=std::move(meta)]() mutable {
      auto start=Clock::now();
      std::ostringstream name; name<<"block_"<<std::setw(6)<<std::setfill('0')<<meta["block_id"].get<size_t>();
      std::ofstream image(output_/(name.str()+".pgm"),std::ios::binary);
      image<<"P5\n"<<optics_->width<<' '<<meta["rows"].get<size_t>()<<"\n255\n";
      image.write(reinterpret_cast<const char*>(pixels.data()),pixels.size()); image.close();
      if (!image) throw std::runtime_error("image archive write failed");
      meta["write_seconds"]=Seconds(Clock::now()-start);
      sensor_msgs::msg::Image message;
      auto ns=static_cast<int64_t>(std::llround(meta["last"]["time_s"].get<double>()*1e9));
      message.header.stamp.sec=ns/1000000000; message.header.stamp.nanosec=ns%1000000000;
      message.header.frame_id="camera_optical_frame";
      message.width=optics_->width; message.height=meta["rows"].get<size_t>();
      message.step=optics_->width; message.encoding="mono8"; message.data=std::move(pixels);
      start=Clock::now(); imagePub_->publish(message); meta["publish_call_seconds"]=Seconds(Clock::now()-start);
      meta["image_subscribers"]=imagePub_->get_subscription_count();
      meta["preview_subscribers"]=previewPub_->get_subscription_count();
      if(previewPub_->get_subscription_count()){
        start=Clock::now();sensor_msgs::msg::Image preview;preview.header=message.header;
        const size_t previewBin=config_["preview_bin"].as<size_t>(32);
        if(previewBin<1 || previewBin>64)throw std::runtime_error("invalid preview_bin");
        preview.width=(message.width+previewBin-1)/previewBin;preview.height=(message.height+previewBin-1)/previewBin;preview.step=preview.width;preview.encoding="mono8";
        preview.data=AreaPreview(message.data,message.width,message.height,previewBin);
        const auto radiometry=LoadRadiometry(config_);
        const bool displaySrgb=config_["preview_srgb"].as<bool>(true) &&
          (backend_=="optix" || (backend_=="cuda_tiles" && radiometry.enabled));
        if(displaySrgb)DisplaySrgb(preview.data,radiometry.black);
        meta["preview_transfer"]=displaySrgb?"srgb_display_only":"identity";
        previewPub_->publish(preview);
        meta["preview_seconds"]=Seconds(Clock::now()-start);meta["preview_area_bin"]=previewBin;
      }
      std::ofstream json(output_/(name.str()+".json")); json<<meta.dump(2)<<'\n'; json.close();
      if (!json) throw std::runtime_error("metadata archive write failed");
      std_msgs::msg::String m; m.data=meta.dump(); metaPub_->publish(m);
      RCLCPP_INFO(node_->get_logger(),"%s: %zux%u %s",name.str().c_str(),optics_->width,message.height,meta["end_reason"].get<std::string>().c_str());
    }));
  }

  YAML::Node config_;
  std::unique_ptr<Optics> optics_;
  std::unique_ptr<Trigger> trigger_;
  ProjectedEncoder projectedEncoder_;
  gz::sim::RenderUtil render_;
  std::vector<gz::rendering::CameraPtr> cameras_;
  gz::sim::Entity model_=gz::sim::kNullEntity;
  std::array<gz::sim::Entity,4> drive_{},steer_{};
  std::deque<Sample> history_;
  std::deque<Event> pending_;
  gz::math::Pose3d offset_;
  double radius_=0,track_=0,wheelbase_=0,now_=0,exposure_=0,spacing_=0,maxSpeed_=0,maxSampleGap_=0;
  double wheelSpreadAbsolute_=.01,wheelSpreadRelative_=0;
  bool enabled_=false,active_=false,holding_=false,tagAfterHold_=false;
  std::string motion_,backend_,terrainPath_;
  Json tileStatistics_,sceneContract_,robotContract_,scanMotion_;
  std::vector<WheelMotion> wheelMotion_;
  std::string scenePath_,robotPath_,ptxPath_;
  std::vector<std::string> linkNames_;
  std::vector<gz::sim::Entity> linkEntities_;
  double lastDriveSpeed_=0,batchMax_=0,stepGapMax_=0;
  std::array<size_t,1000> stepGaps_{};
  size_t stepGapCount_=0;
  Clock::time_point stepWall_;
  bool stepWallValid_=false,terrainFailed_=false;
  Line lastLine_;
  double lastSceneTime_=0;
  size_t invalidPixels_=0;
  size_t rows_=0,block_=0,segment_=0,globalLine_=0,renders_=0;
  double renderSeconds_=0,readSeconds_=0,mappingSeconds_=0,sceneSeconds_=0;
  std::vector<uint8_t> buffer_;
  Json first_,last_,tags_=Json::array();
  std::filesystem::path output_;
  std::deque<std::future<void>> writes_;
  std::thread renderThread_;
  std::mutex mutex_,metricsMutex_;
  std::deque<Job> queuedJobs_;
  std::unique_ptr<QueueBudget> queueBudget_;
  size_t batchRows_=256,tileSlots_=24;
  uint64_t nextCaptureLine_=0;
  double prefetchDistance_=4;
  std::condition_variable wake_,done_;
  bool busy_=false,quit_=false,ready_=false;
  Job job_;
  std::exception_ptr error_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_,previewPub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metaPub_,statusPub_,statePub_;
  Clock::time_point lastStatePublish_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motionSub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableService_;
};
}  // namespace agv_linescan
GZ_ADD_PLUGIN(agv_linescan::GzLineScan,gz::sim::System,
              agv_linescan::GzLineScan::ISystemConfigure,
              agv_linescan::GzLineScan::ISystemPreUpdate,
              agv_linescan::GzLineScan::ISystemPostUpdate)
