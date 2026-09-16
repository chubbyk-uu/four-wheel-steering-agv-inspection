#include "agv_linescan/radiometry_config.hpp"
// Encoder-triggered camera owned by a GZ system, not a standard camera sensor.
// GL work stays on one thread. Ogre2 uses synchronized scene sampling; the
// tiled CUDA plane uses a bounded asynchronous queue to keep physics advancing.
#include "agv_linescan/sampling.hpp"
#include "agv_linescan/scan_motion.hpp"
#include "agv_linescan/camera_encoder_geometry.hpp"
#include "agv_linescan/ground_geometry.hpp"
#include "agv_linescan/flight_recorder.hpp"
#include "agv_linescan/contact_kinematics.hpp"
#include "agv_linescan/queue_budget.hpp"
#include "agv_linescan/mount_geometry.hpp"
#include "agv_linescan/preview.hpp"
#include "agv_linescan/tail_policy.hpp"
#ifdef AGV_HAS_CUDA
#include "agv_linescan/cuda_grid.hpp"
#include "agv_linescan/cuda_tiles.hpp"
#endif
#ifdef AGV_HAS_OPTIX
#include "agv_linescan/optix_scene.hpp"
#endif
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <mutex>
#include <thread>
#include <sstream>
#include <utility>
#include <gz/math/Matrix3.hh>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/rendering/RenderUtil.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Visual.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gz/sim/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Static.hh>

namespace agv_linescan {
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
double Seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
struct WheelMotion {bool valid=false;double zMin=0,zMax=0,lastZ=0,maxStep=0;gz::math::Vector3d localMin,localMax;};
struct Sample { double time; gz::math::Pose3d pose; gz::math::Pose3d camera; std::vector<gz::math::Pose3d> links; };
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
    StopDiagnosticWriter();
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
      if(scene.contains("physics_heightmap")) {
        geometryPath_=std::filesystem::path(scenePath_).parent_path()/scene.at("physics_heightmap").at("source_heightfield").get<std::string>();
        std::ifstream geometryFile(geometryPath_);Json field;geometryFile>>field;
        groundGeometry_=std::make_unique<GroundGeometry>(field);
      }
      sceneContract_=scene;robotContract_={{"source_sha256",robot.at("source_sha256")},{"cylinder_facets",robot.at("cylinder_facets")},{"link_names",linkNames_}};
    }
    if (backend_=="cuda_tiles") terrainPath_=sdf->Get<std::string>("terrain_manifest");
    auto sampling=config_["sampling"];
    batchRows_=sampling["batch_rows"].as<size_t>(256);
    tileSlots_=sampling["tile_cache_slots"].as<size_t>(24);
    prefetchDistance_=sampling["prefetch_distance_m"].as<double>(4);
    queueBudget_=std::make_unique<QueueBudget>(sampling["queue_capacity_lines"].as<size_t>(2048),
        sampling["queue_capacity_jobs"].as<size_t>(512),sampling["queue_max_wait_s"].as<double>(.15));
    // Flight recorder: the seconds before a scan-motion rejection, kept in memory
    // so the physics step pays only an assignment. The count caps memory; the span
    // is what keeps the window the same duration if the step size ever changes.
    auto flight=config_["flight_recorder"];
    const double flightSpan=flight?flight["span_s"].as<double>(2.):2.;
    const size_t flightCapacity=flight?flight["max_samples"].as<size_t>(4000):4000;
    residualHalfCooldown_=flight?flight["half_limit_cooldown_s"].as<double>(10.):10.;
    flightDumpLimit_=flight?flight["max_dumps"].as<std::uint64_t>(20):20;
    // The offline audit divides camera-centre horizontal travel by nominal
    // encoder distance over each pose-tag interval. By the time it runs the two
    // seconds that would explain an alert are long gone, so the same test runs
    // here against the same floor. Pass starts are expected to trip it -- a
    // pitching body moves the camera without any contact sliding -- so geometry
    // alerts get their own budget and cannot starve a scan-motion dump.
    geometryFloor_=flight?flight["camera_encoder_geometry_floor"].as<double>(.95):.95;
    geometryDumpLimit_=flight?flight["max_geometry_dumps"].as<std::uint64_t>(40):40;
    if(!std::isfinite(geometryFloor_)||geometryFloor_<0||geometryFloor_>1||geometryDumpLimit_>1000)
      throw std::runtime_error("invalid flight_recorder configuration");
    probeKinematics_=flight && flight["probe_contact_kinematics"].as<bool>(false);
    contactEvidenceRequired_=flight?flight["contact_evidence"].as<bool>(true):true;
    if(!std::isfinite(flightSpan)||flightSpan<=0||flightSpan>60||flightCapacity<100||flightCapacity>200000||
       !std::isfinite(residualHalfCooldown_)||residualHalfCooldown_<0||flightDumpLimit_==0||flightDumpLimit_>1000)
      throw std::runtime_error("invalid flight_recorder configuration");
    flight_=std::make_unique<FlightRecorder>(flightCapacity,flightSpan);
    residualStats_=std::make_unique<ResidualStats>(config_["max_scan_residual_m_s"].as<double>(.03));
    contactStats_=std::make_unique<CaptureContactStats>();
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
    minTailRows_=config_["min_tail_rows"].as<size_t>(1000);
    DiscardShortTail(0,false,minTailRows_);
    maxTagGap_=config_["pose_tag_time_gap_s"].as<double>(.1);
    if(!std::isfinite(maxTagGap_) || maxTagGap_<=0)throw std::runtime_error("invalid pose tag time gap");
    exposure_=config_["exposure_s"].as<double>();
    // Read immutable configuration once, not for every exposure/link SLERP.
    maxSampleGap_=config_["max_sample_gap_s"].as<double>();
    if(!std::isfinite(maxSampleGap_) || maxSampleGap_<=0)
      throw std::runtime_error("invalid maximum pose sample gap");
    spacing_=config_["line_spacing_m"].as<double>();
    if(auto encoder=config_["wheel_encoder"]; encoder) {
      const auto wheel=encoder["wheel"].as<std::string>();
      const std::array<std::string,4> names={"fl","fr","rl","rr"};
      auto found=std::find(names.begin(),names.end(),wheel);
      if(found==names.end())throw std::runtime_error("invalid encoder wheel");
      encoderWheel_=std::distance(names.begin(),found);
      WheelEncoderScale scale(encoder["ppr"].as<int>(),encoder["decode"].as<int>(),
          encoder["multiplier"].as<int>(),encoder["divider"].as<int>(),radius_);
      spacing_=scale.spacing;
      config_["line_spacing_m"]=spacing_;
      encoderContract_={{"model","wheel_ab_resampled_phase_v1"},{"wheel",wheel},
        {"ppr",encoder["ppr"].as<int>()},{"decode",encoder["decode"].as<int>()},
        {"multiplier",encoder["multiplier"].as<int>()},{"divider",encoder["divider"].as<int>()},
        {"lines_per_revolution",scale.linesPerTurn},{"calibrated_diameter_m",2*radius_},
        {"actual_diameter_m_truth",sdf->Get<double>("actual_wheel_diameter",2*radius_).first},
        {"timing_model","continuous angular phase interpolation; no AB electrical waveform"}};
    }
    maxSpeed_=sdf->Get<double>("max_scan_speed",.25).first;
    if (!rows_ || exposure_<=0 || maxSpeed_<=0 || rows_>16384)
      throw std::runtime_error("invalid acquisition configuration");
    optics_=std::make_unique<Optics>(config_["width"].as<size_t>(),
        config_["render_width"].as<size_t>(10240), config_["pixel_pitch_m"].as<double>(),
        config_["focal_length_m"].as<double>(),config_["nominal_width_m"].as<double>(),
        config_["ray_polynomial"].as<std::vector<double>>());
    const auto encoderMode=config_["encoder_output_mode"].as<std::string>("strict");
    if(encoderMode!="strict" && encoderMode!="position")throw std::runtime_error("invalid encoder output mode");
    trigger_=std::make_unique<Trigger>(spacing_,encoderMode=="position");
    offset_=gz::math::Pose3d(config_["camera_x_m"].as<double>(),0,
        optics_->height-config_["base_nominal_height_m"].as<double>(),0,0,0);
    flexible_=config_["mount_flex"] && config_["mount_flex"]["enabled"].as<bool>(false);
    if(flexible_) {
      const auto pivot=LoadMountPivot(config_);
      offset_.Pos()-=gz::math::Vector3d(pivot.x,0,pivot.z);
    }
    output_=std::filesystem::path(sdf->Get<std::string>("output_dir")) /
        ("session_cpp_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(output_);
    if(groundGeometry_)std::filesystem::copy_file(geometryPath_,output_/"ground_geometry_heightfield.json");
    // Establish the status contract before normal operation. It must exist even
    // if the process is killed before the first diagnostic event is produced.
    WriteDiagnosticStatus();
    diagnosticThread_=std::thread([this]{DiagnosticLoop();});
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
    if(probeKinematics_)probeService_=node_->create_service<std_srvs::srv::Trigger>("/linescan/dump_probe",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success=DumpFlight("startup_probe");
        res->message=res->success?"probe queued; wait for diagnostic_status at shutdown":"probe unavailable, suppressed or could not be queued";
      });
    enableService_=node_->create_service<std_srvs::srv::SetBool>("/linescan/set_enabled",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          if(req->data && contactEvidenceRequired_ && !contactEvidenceReady_) {
            std::ostringstream detail;detail<<"wheel contact evidence is not ready";
            const std::array<const char*,4> names={"fl","fr","rl","rr"};
            for(size_t i=0;i<4;++i)detail<<' '<<names[i]<<"(sensor="<<contactSensor_[i]
              <<",collision="<<wheelCollision_[i]<<",data="<<(contactDataAvailable_[i]?1:0)<<')';
            res->success=false;res->message=detail.str();return;
          }
          if(req->data && !RecoverSampler()) {
            res->success=false;res->message="sampler recovery failed; see status and repair terrain/storage";return;
          }
          Dispatch(Job{{},"capture_toggle",now_});
          if(terrainFailed_) {res->success=false;res->message="terrain sampling failed; see /linescan/status and restart after fixing the data";return;}
          Reset("capture_toggle"); enabled_=false;
          if(req->data && AsyncBackend()) {
            if(history_.empty()) {res->success=false;res->message="wait for robot pose before preparing terrain";return;}
            Line warm;warm.midpoint=history_.back().camera;
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
      if(flexible_) {
        cameraEntity_=ecm.EntityByComponents(gz::sim::components::Name("camera_carrier_link"),gz::sim::components::ParentEntity(model_));
        if(!cameraEntity_)throw std::runtime_error("missing flexible camera carrier");
      }
      const std::array<std::string,4> names={"fl","fr","rl","rr"};
      for (size_t i=0;i<4;++i) {
        wheelLink_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_wheel_link"),gz::sim::components::ParentEntity(model_));
        drive_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_drive_joint"),gz::sim::components::ParentEntity(model_));
        steer_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_steer_joint"),gz::sim::components::ParentEntity(model_));
        if (!wheelLink_[i] || !drive_[i] || !steer_[i]) throw std::runtime_error("missing AGV wheel or joint");
        wheelCollision_[i]=ecm.EntityByComponents(gz::sim::components::Collision(),gz::sim::components::ParentEntity(wheelLink_[i]));
        // Suspension travel is diagnostic only: it hints at losing contact, and a
        // missing joint must not stop a capture that never needed it.
        suspension_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_suspension_joint"),gz::sim::components::ParentEntity(model_));
        if(probeKinematics_ && wheelLink_[i]!=gz::sim::kNullEntity)gz::sim::Link(wheelLink_[i]).EnableVelocityChecks(ecm);
        for (auto entity:{drive_[i],steer_[i],suspension_[i]}) {
          if (!entity) continue;
          if (!ecm.Component<gz::sim::components::JointPosition>(entity))
            ecm.CreateComponent(entity,gz::sim::components::JointPosition());
          if (!ecm.Component<gz::sim::components::JointVelocity>(entity))
            ecm.CreateComponent(entity,gz::sim::components::JointVelocity());
        }
      }
    }
    // Sensors are spawned with the robot and their data component is created by
    // the Contact system. Retry discovery until both exist; capture enable then
    // fails loudly instead of producing a diagnostic dump with empty evidence.
    contactEvidenceReady_=!contactEvidenceRequired_;
    if(!contactEvidenceRequired_)return;
    contactEvidenceReady_=true;
    const std::array<std::string,4> names={"fl","fr","rl","rr"};
    for(size_t i=0;i<4;++i) {
      if(contactSensor_[i]==gz::sim::kNullEntity && wheelLink_[i]!=gz::sim::kNullEntity)
        contactSensor_[i]=ecm.EntityByComponents(gz::sim::components::Name(names[i]+"_wheel_contact"),
            gz::sim::components::ParentEntity(wheelLink_[i]));
      // Contact is configured by a sensor entity, but gz-sim stores the physics
      // ContactSensorData component on the monitored collision entity.
      contactDataAvailable_[i]=wheelCollision_[i]!=gz::sim::kNullEntity &&
          ecm.Component<gz::sim::components::ContactSensorData>(wheelCollision_[i])!=nullptr;
      contactEvidenceReady_=contactEvidenceReady_ && wheelCollision_[i]!=gz::sim::kNullEntity &&
          contactSensor_[i]!=gz::sim::kNullEntity && contactDataAvailable_[i];
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
      Sample snapshot{now_,gz::sim::worldPose(model_,ecm),{}, {}};
      snapshot.camera=(flexible_?gz::sim::worldPose(cameraEntity_,ecm):snapshot.pose)*offset_;
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
      ScanVelocity fitted{};bool fitted_ok=false;
      FlightSample flight{};
      flight.passData=wheelDataValid?1:0;
      if(config_["projected_encoder"].as<bool>(false)) {
        try {
          if(!wheelDataValid) {projectedEncoder_.Reset();throw std::runtime_error("invalid wheel data");}
          auto velocity=FitScanVelocity(speeds,angles,wheelbase_,track_);
          fitted=velocity;fitted_ok=true;
          distance=projectedEncoder_.Update(wheelDistance,angles);
          lastDriveSpeed_=velocity.vx;
          // Each guard is kept separately: the reason a step was rejected is not
          // recoverable from the combined flag once it is false.
          flight.passSpeed=std::abs(velocity.vx)<=maxSpeed_;
          flight.passLateral=std::abs(velocity.vy)<=config_["max_scan_lateral_m_s"].as<double>();
          flight.passYaw=std::abs(velocity.wz)<=config_["max_yaw_rate_rad_s"].as<double>();
          flight.passResidual=velocity.residual<=config_["max_scan_residual_m_s"].as<double>();
          valid=wheelDataValid && flight.passSpeed && flight.passLateral && flight.passYaw && flight.passResidual;
          scanMotion_={{"vx",velocity.vx},{"vy",velocity.vy},{"wz",velocity.wz},{"residual",velocity.residual},
                       {"wheel_residual_m_s",velocity.wheelResidual}};
        } catch(const std::exception&) {valid=false;}
      }
      RecordFlight(flight,fitted,fitted_ok,speeds,angles,ecm,history_.back().pose);
      if(enabled_ && !valid && !config_["projected_encoder"].as<bool>(false)) scanMotion_={{"wheel_speeds_m_s",speeds},{"wheel_speed_spread_m_s",*range.second-*range.first},
        {"max_abs_steer_rad",maxSteer},{"encoder_yaw_estimate_rad_s",(speeds[1]+speeds[3]-speeds[0]-speeds[2])/(2*track_)},
        {"speed_limit_m_s",maxSpeed_},{"wheel_spread_limit_m_s",spreadLimit},
        {"steer_limit_rad",config_["max_steer_rad"].as<double>()},{"yaw_limit_rad_s",config_["max_yaw_rate_rad_s"].as<double>()}};
      // Quality gates may use all four wheels; the trigger is one physical
      // shaft, never a steering-projected or truth-distance virtual encoder.
      if(encoderWheel_>=0)distance=wheelDistance[encoderWheel_];
      // Start a pass only after initial wheel alignment. Once started, the
      // armed camera keeps its partial frame and encoder phase in HOLD/ALIGN.
      if (enabled_ && (motion_=="DRIVE" || (active_ &&
          (motion_=="BRAKE" || motion_=="HOLD" || motion_=="ALIGN"))) && valid) {
        try {
          if(encoderWheel_>=0) {
            const int polarity=std::cos(angles[encoderWheel_])<0 ? -1 : 1;
            if(active_ && polarity!=encoderPolarity_) {
              // This guard runs after the sample was pushed, so correct the stored
              // record; otherwise the dump would show polarity passing on the very
              // step it rejected.
              MarkPolarityFailure();
              throw std::runtime_error("unsupported_scan_motion");
            }
            encoderPolarity_=polarity;
          }
          auto events=trigger_->Update(now_,distance);
          // Existing scan_direction means body forward/backward, not AB polarity.
          if(encoderWheel_>=0)for(auto& event:events)event.direction*=encoderPolarity_;
          pending_.insert(pending_.end(),events.begin(),events.end());
          active_=true;
          while (!pending_.empty() && pending_.front().time+exposure_<=now_) {
            auto event=pending_.front(); pending_.pop_front();
            Line line; line.event=event;
            for (size_t i=0;i<3;++i) {
              double t=event.time+exposure_*(i+.5)/3;
              line.exposure[i]=PoseAt(t,SIZE_MAX,true);
              for(size_t j=0;j<linkNames_.size();++j)line.links[i].push_back(PoseAt(t,j));
            }
            line.midpoint=PoseAt(event.time+exposure_/2,SIZE_MAX,true);
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

  gz::math::Pose3d PoseAt(double t,size_t link=SIZE_MAX,bool camera=false) const {
    for (size_t i=1;i<history_.size();++i) {
      const auto &a=history_[i-1],&b=history_[i];
      if (t>=a.time && t<=b.time) {
        if (b.time-a.time>maxSampleGap_) throw std::runtime_error("pose_gap");
        if(camera && !flexible_)return Interpolate(a.pose,b.pose,(t-a.time)/(b.time-a.time))*offset_;
        return Interpolate(camera?a.camera:(link==SIZE_MAX?a.pose:a.links.at(link)),camera?b.camera:(link==SIZE_MAX?b.pose:b.links.at(link)),(t-a.time)/(b.time-a.time));
      }
    }
    throw std::runtime_error("pose_outside_history");
  }

  // All diagnostic files share one FIFO writer. In particular, events.jsonl
  // must never be appended by independent physics and worker threads: append
  // mode does not make a multi-part C++ stream insertion atomic or ordered.
  template<class Job>
  bool QueueDiagnostic(Job&& job) noexcept {
    try {
      std::function<void()> work(std::forward<Job>(job));
      {
        std::lock_guard<std::mutex> lock(diagnosticMutex_);
        if(diagnosticQuit_) {
          ++diagnosticDropped_;
          return false;
        }
        diagnosticJobs_.push_back(std::move(work));
      }
      diagnosticWake_.notify_one();
      return true;
    } catch(const std::exception &e) {
      ++diagnosticDropped_;
      std::cerr<<"linescan diagnostic queue rejected a record: "<<e.what()<<std::endl;
    } catch(...) {
      ++diagnosticDropped_;
      std::cerr<<"linescan diagnostic queue rejected a record: unknown exception"<<std::endl;
    }
    return false;
  }

  void ArchiveEvent(Json event) noexcept {
    try {
      const auto path=output_/"events.jsonl";
      QueueDiagnostic([path,event=std::move(event)]() {
        std::ofstream out(path,std::ios::app);
        out<<event.dump()<<'\n';
        if(!out)throw std::runtime_error("diagnostic event write failed: "+path.string());
      });
    } catch(...) {
      ++diagnosticDropped_;
      std::cerr<<"linescan diagnostic event could not be queued"<<std::endl;
    }
  }

  void DiagnosticLoop() {
    for(;;) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(diagnosticMutex_);
        diagnosticWake_.wait(lock,[this]{return diagnosticQuit_ || !diagnosticJobs_.empty();});
        if(diagnosticJobs_.empty()) {
          if(diagnosticQuit_)return;
          continue;
        }
        job=std::move(diagnosticJobs_.front());diagnosticJobs_.pop_front();
      }
      try {job();}
      catch(const std::exception &e) {
        ++diagnosticWriteFailures_;
        std::cerr<<"linescan diagnostic write failed: "<<e.what()<<std::endl;
      } catch(...) {
        ++diagnosticWriteFailures_;
        std::cerr<<"linescan diagnostic write failed: unknown exception"<<std::endl;
      }
      WriteDiagnosticStatus();
    }
  }

  void WriteDiagnosticStatus() {
    if(output_.empty())return;
    const auto path=output_/"diagnostic_status.json";
    const auto temporary=output_/"diagnostic_status.json.tmp";
    try {
      std::error_code removeError;
      std::filesystem::remove(temporary,removeError);
      if(removeError)throw std::runtime_error("cannot remove stale diagnostic status: "+removeError.message());
      std::ofstream out(temporary);
      out<<Json{{"schema","agv.linescan.diagnostic_status.v1"},
                {"write_failures",diagnosticWriteFailures_.load()},
                {"dropped_records",diagnosticDropped_.load()}}.dump(2)<<'\n';
      out.close();
      if(!out)throw std::runtime_error("diagnostic status write failed: "+temporary.string());
      std::filesystem::rename(temporary,path);
    } catch(const std::exception &e) {
      std::error_code ignored;
      std::filesystem::remove(temporary,ignored);
      std::cerr<<"linescan diagnostic status write failed: "<<e.what()<<std::endl;
    } catch(...) {
      std::error_code ignored;
      std::filesystem::remove(temporary,ignored);
      std::cerr<<"linescan diagnostic status write failed: unknown exception"<<std::endl;
    }
  }

  void StopDiagnosticWriter() {
    if(diagnosticThread_.joinable()) {
      {
        std::lock_guard<std::mutex> lock(diagnosticMutex_);
        diagnosticQuit_=true;
      }
      diagnosticWake_.notify_one();diagnosticThread_.join();
    }
    WriteDiagnosticStatus();
  }

  // Fill one flight sample and fold it into the running statistics. Everything
  // here is arithmetic on preallocated storage: no allocation, no file access.
  void RecordFlight(FlightSample &flight,const ScanVelocity &fitted,bool fitted_ok,
                    const std::array<double,4> &speeds,const std::array<double,4> &angles,
                    const gz::sim::EntityComponentManager &ecm,const gz::math::Pose3d &pose) {
    if(!flight_)return;
    flight.simTime=now_;
    if(probeKinematics_ && !history_.empty()) {
      const auto &camera=history_.back().camera;
      for(size_t j=0;j<3;++j)flight.cameraPos[j]=camera.Pos()[j];
      flight.cameraQuat[0]=camera.Rot().X();flight.cameraQuat[1]=camera.Rot().Y();
      flight.cameraQuat[2]=camera.Rot().Z();flight.cameraQuat[3]=camera.Rot().W();
      const std::array<const char*,2> names={"camera_roll_joint","camera_pitch_joint"};
      for(size_t j=0;j<2;++j){auto entity=ecm.EntityByComponents(gz::sim::components::Name(names[j]),gz::sim::components::ParentEntity(model_));
        auto pos=ecm.Component<gz::sim::components::JointPosition>(entity);
        if(pos && !pos->Data().empty())flight.mountAngle[j]=pos->Data()[0];}
    }
    for(size_t i=0;i<4;++i) {
      flight.wheelSpeed[i]=speeds[i];
      flight.driveRate[i]=radius_>0?speeds[i]/radius_:0;
      flight.steerAngle[i]=angles[i];
      if(suspension_[i]!=gz::sim::kNullEntity) {
        auto sp=ecm.Component<gz::sim::components::JointPosition>(suspension_[i]);
        auto sv=ecm.Component<gz::sim::components::JointVelocity>(suspension_[i]);
        if(sp && !sp->Data().empty())flight.suspensionPos[i]=sp->Data()[0];
        if(sv && !sv->Data().empty())flight.suspensionVel[i]=sv->Data()[0];
      }
      flight.wheelResidual[i]=fitted_ok?fitted.wheelResidual[i]:0;
      RecordContact(flight.contact[i],i,ecm);
    }
    flight.vx=fitted.vx;flight.vy=fitted.vy;flight.wz=fitted.wz;flight.residual=fitted.residual;
    flight.bodyX=pose.Pos().X();flight.bodyY=pose.Pos().Y();flight.bodyZ=pose.Pos().Z();
    flight.bodyQx=pose.Rot().X();flight.bodyQy=pose.Rot().Y();
    flight.bodyQz=pose.Rot().Z();flight.bodyQw=pose.Rot().W();
    // Body motion from the poses themselves. Four wheels agreeing with each other
    // but not with this is whole-vehicle slip, which per-wheel residuals cannot show
    // -- but only if both are in the same frame. The fit is in base_link, so the
    // world displacement is rotated into base_link before it is compared; left in
    // world, a normal pass after a turnaround (yaw near pi) would read as slip.
    if(bodyPrevValid_ && now_>bodyPrevTime_) {
      const double dt=now_-bodyPrevTime_;
      const auto world=(pose.Pos()-bodyPrevPose_.Pos())/dt;
      const auto body=pose.Rot().RotateVectorReverse(world);
      flight.bodyVx=body.X();flight.bodyVy=body.Y();flight.bodyVz=body.Z();
      // Relative rotation rather than a yaw difference: subtracting yaw across
      // +/-pi invents a spike of about 2*pi/dt exactly during a turnaround.
      const auto delta=bodyPrevPose_.Rot().Inverse()*pose.Rot();
      flight.bodyWz=delta.Yaw()/dt;
    }
    bodyPrevValid_=true;bodyPrevTime_=now_;bodyPrevPose_=pose;
    flight.encoderPolarity=encoderPolarity_;
    flight.capturing=active_?1:0;
    flight.motionState=MotionCode(motion_);
    flight_->Push(flight);
    if(contactStats_)contactStats_->Add(flight);
    if(!fitted_ok || !residualStats_)return;
    residualStats_->Add(fitted.residual);
    // Crossing half the gate is written once, then only after a cooldown, so a
    // residual that lives near the threshold cannot turn into a stream of writes.
    if(fitted.residual>residualStats_->limit()/2 && now_-lastHalfEvent_>=residualHalfCooldown_) {
      lastHalfEvent_=now_;++halfEvents_;
      // Serialising and appending here would put a file write on the physics
      // thread at the first approach to the gate -- precisely the moment least
      // able to absorb a disturbance. Hand it to the ordered diagnostic writer.
      Json event={{"reason","scan_residual_near_limit"},{"simulation_time_s",now_},
                  {"residual_m_s",fitted.residual},{"limit_m_s",residualStats_->limit()},
                  {"wheel_residual_m_s",fitted.wheelResidual},
                  {"position_m",{flight.bodyX,flight.bodyY,flight.bodyZ}},
                  {"capturing",active_},{"occurrence",halfEvents_}};
      ArchiveEvent(std::move(event));
    }
  }

  static float VectorMagnitude(const gz::msgs::Vector3d &v) {
    return static_cast<float>(std::sqrt(v.x()*v.x()+v.y()*v.y()+v.z()*v.z()));
  }

  void RecordContact(WheelContactEvidence &out,size_t wheel,
                     const gz::sim::EntityComponentManager &ecm) const {
    if(!contactEvidenceRequired_)return;
    if(contactSensor_[wheel]==gz::sim::kNullEntity || wheelCollision_[wheel]==gz::sim::kNullEntity)return;
    const auto component=ecm.Component<gz::sim::components::ContactSensorData>(wheelCollision_[wheel]);
    if(!component)return;
    out.available=1;
    const auto &contacts=component->Data();
    out.pairs=static_cast<std::uint16_t>(std::min(contacts.contact_size(),65535));
    for(const auto &contact:contacts.contact()) {
      const bool wheelFirst=contact.collision1().id()==wheelCollision_[wheel];
      const bool wheelSecond=contact.collision2().id()==wheelCollision_[wheel];
      const auto other=wheelFirst?contact.collision2().id():(wheelSecond?contact.collision1().id():0);
      const std::uint8_t side=wheelFirst?1:(wheelSecond?2:0);
      for(int p=0;p<contact.position_size();++p) {
        if(out.points!=65535)++out.points;
        const double depth=p<contact.depth_size()?contact.depth(p):0.;
        out.maxDepth=std::max(out.maxDepth,static_cast<float>(depth));
        float force=0;
        if(p<contact.wrench_size()) {
          const auto &wrench=contact.wrench(p);
          force=VectorMagnitude(side==2?wrench.body_2_wrench().force():wrench.body_1_wrench().force());
          out.maxForceMagnitude=std::max(out.maxForceMagnitude,force);
        }
        if(out.stored>=kContactPointsPerWheel) {out.truncated=1;continue;}
        auto &sample=out.point[out.stored++];
        const auto &position=contact.position(p);
        sample.position[0]=position.x();sample.position[1]=position.y();sample.position[2]=position.z();
        if(p<contact.normal_size()) {
          const auto &normal=contact.normal(p);
          sample.normal[0]=normal.x();sample.normal[1]=normal.y();sample.normal[2]=normal.z();
        }
        if(probeKinematics_ && other) {
          auto parent=ecm.Component<gz::sim::components::ParentEntity>(other);
          auto modelParent=parent?ecm.Component<gz::sim::components::ParentEntity>(parent->Data()):nullptr;
          auto stationary=modelParent?ecm.Component<gz::sim::components::Static>(modelParent->Data()):nullptr;
          auto lv=ecm.Component<gz::sim::components::WorldLinearVelocity>(wheelLink_[wheel]);
          auto av=ecm.Component<gz::sim::components::WorldAngularVelocity>(wheelLink_[wheel]);
          if(stationary && stationary->Data() && lv && av) {
            const auto center=gz::sim::worldPose(wheelLink_[wheel],ecm).Pos();
            const auto point=gz::math::Vector3d(position.x(),position.y(),position.z());
            const auto material=lv->Data()+av->Data().Cross(point-center);
            auto n=gz::math::Vector3d(sample.normal[0],sample.normal[1],sample.normal[2]);
            const auto tangent=ContactTangentVelocity(lv->Data(),av->Data(),point-center,n);
            if(tangent) {
              sample.velocityAvailable=1;sample.tangentialSpeed=tangent->Length();
              for(size_t j=0;j<3;++j)sample.relativeVelocityWorld[j]=material[j];}
          }
        }
        sample.depth=depth;sample.forceMagnitude=force;
        sample.otherCollision=other;sample.wheelCollisionSide=side;
      }
    }
  }

  void MarkPolarityFailure() {
    if(!flight_)return;
    if(auto *newest=flight_->Newest())newest->passPolarity=0;
  }

  static uint8_t MotionCode(const std::string &name) {
    if(name=="DRIVE")return 1;
    if(name=="BRAKE")return 2;
    if(name=="HOLD")return 3;
    if(name=="ALIGN")return 4;
    if(name=="FEEDBACK_HOLD")return 5;
    return name.empty()?0:6;
  }

  Json ResidualSummary() const {
    if(!residualStats_)return Json::object();
    return Json{{"samples",residualStats_->samples()},
                {"limit_m_s",residualStats_->limit()},
                {"max_m_s",residualStats_->max()},
                {"p95_m_s",residualStats_->Quantile(.95)},
                {"p99_m_s",residualStats_->Quantile(.99)},
                {"over_half_limit",residualStats_->overHalf()},
                {"over_limit",residualStats_->overLimit()},
                {"near_limit_events",halfEvents_},
                {"flight_dumps",flightDumps_},{"flight_dumps_suppressed",flightDumpsSuppressed_},
                {"diagnostic_write_failures",diagnosticWriteFailures_.load()},
                {"diagnostic_dropped_records",diagnosticDropped_.load()},
                {"histogram_bin_width_m_s",2*residualStats_->limit()/(ResidualStats::kBins-1)},
                {"histogram",residualStats_->histogram()}};
  }

  Json ContactSummary() const {
    Json wheels=Json::array();
    if(!contactStats_)return wheels;
    static constexpr std::array<const char*,4> names={"fl","fr","rl","rr"};
    for(std::size_t i=0;i<4;++i) {
      const auto &w=contactStats_->wheels()[i];
      wheels.push_back({{"wheel",names[i]},{"capturing_samples",w.samples},
        {"contact_available_samples",w.available},{"no_contact_samples",w.noContact},
        {"no_contact_episodes",w.noContactEpisodes},
        {"longest_no_contact_samples",w.longestNoContact},{"max_contact_points",w.maxPoints},
        {"max_depth_m",w.maxDepth},{"max_normal_tilt_rad",w.maxNormalTiltRad},
        {"max_suspension_rate_m_s",w.maxSuspensionRate},
        {"suspension_rate_histogram_bin_width_m_s",CaptureContactStats::kSuspensionBinWidth},
        {"suspension_rate_histogram",w.suspensionRateHistogram}});
    }
    return wheels;
  }

  // Copy the ring on the physics thread, hand the copy to a writer, and return.
  // The step must not wait for the disk, and stopping must not wait for data.
  bool DumpFlight(const std::string &reason) noexcept {
    return DumpFlight(reason,flightDumps_,flightDumpLimit_,flightDumpsSuppressed_);
  }

  bool DumpFlight(const std::string &reason,std::uint64_t &count,std::uint64_t limit,
                  std::uint64_t &suppressed) noexcept {
    try {
      if(!flight_ || flight_->size()==0)return false;
      // A repeating fault must not turn the diagnostic into its own disturbance.
      // The count keeps being reported, so a capped run is never mistaken for a
      // run that only faulted this many times.
      if(count>=limit){++suppressed;return false;}
      auto rows=flight_->Snapshot();
      Json summary=ResidualSummary();
      const auto index=++count;
      const double when=now_;
      auto path=output_/("flight_"+std::to_string(index)+"_"+reason+".json");
      const bool probe=probeKinematics_;
      QueueDiagnostic([path,rows=std::move(rows),summary=std::move(summary),reason,when,probe]() {
      Json doc={{"schema","agv.linescan.flight_recorder.v2"},{"reason",reason},
                {"probe_contact_kinematics",probe},{"camera_pose_available",probe},
                {"fault_simulation_time_s",when},{"samples",rows.size()},
                {"window_s",rows.empty()?0.:rows.back().simTime-rows.front().simTime},
                {"note","bounded window ending at dump request or rejected step; velocity fields require opt-in probe and static other collision"},
                {"residual_statistics",summary}};
      Json records=Json::array();
      for(const auto &r:rows) {
        Json contact=Json::array();
        for(const auto &wheel:r.contact) {
          Json points=Json::array();
          for(size_t i=0;i<wheel.stored;++i) {
            const auto &p=wheel.point[i];
            points.push_back({{"position_world_m",p.position},{"normal",p.normal},{"depth_m",p.depth},
              {"relative_velocity_world_m_s",p.relativeVelocityWorld},{"tangential_speed_m_s",p.tangentialSpeed},
              {"velocity_available",p.velocityAvailable!=0},{"force_magnitude_n",p.forceMagnitude},{"other_collision_id",p.otherCollision},
              {"wheel_collision_side",p.wheelCollisionSide}});
          }
          contact.push_back({{"available",wheel.available!=0},{"pair_count",wheel.pairs},
            {"point_count",wheel.points},{"stored_point_count",wheel.stored},{"truncated",wheel.truncated!=0},
            {"max_depth_m",wheel.maxDepth},{"max_force_magnitude_n",wheel.maxForceMagnitude},
            {"points",std::move(points)}});
        }
        records.push_back({{"t",r.simTime},{"drive_rate_rad_s",r.driveRate},{"wheel_speed_m_s",r.wheelSpeed},
          {"steer_rad",r.steerAngle},{"suspension_m",r.suspensionPos},{"suspension_rate_m_s",r.suspensionVel},
          {"wheel_residual_m_s",r.wheelResidual},{"vx",r.vx},{"vy",r.vy},{"wz",r.wz},{"residual",r.residual},
          {"camera_xyz",r.cameraPos},{"camera_quat_xyzw",r.cameraQuat},{"mount_roll_pitch_rad",r.mountAngle},
          {"body_xyz",{r.bodyX,r.bodyY,r.bodyZ}},{"body_quat_xyzw",{r.bodyQx,r.bodyQy,r.bodyQz,r.bodyQw}},
          {"body_velocity_base_link_m_s",{r.bodyVx,r.bodyVy,r.bodyVz}},{"body_yaw_rate_rad_s",r.bodyWz},
          {"encoder_polarity",r.encoderPolarity},{"motion_state",r.motionState},{"capturing",r.capturing!=0},
          {"wheel_contact",std::move(contact)},
          {"pass",{{"speed",r.passSpeed!=0},{"lateral",r.passLateral!=0},{"yaw",r.passYaw!=0},
                   {"residual",r.passResidual!=0},{"data",r.passData!=0},{"polarity",r.passPolarity!=0}}}});
      }
      doc["records"]=std::move(records);
      std::ofstream out(path);
      out<<doc.dump();
      if(!out)throw std::runtime_error("flight recorder write failed: "+path.string());
      });
      return true;
    } catch(...) {
      ++diagnosticDropped_;
      std::cerr<<"linescan flight record could not be queued"<<std::endl;
      return false;
    }
  }

  void Reset(const std::string &reason) {
    Json event={{"reason",reason},{"simulation_time_s",now_},{"discarded_pending_lines",pending_.size()}};
    if(reason=="unsupported_scan_motion" || reason=="direction change")event["motion_condition"]=scanMotion_;
    // Hand the ring to a writer before the capture state is cleared. The copy is
    // synchronous and cheap; the file is not, and the step must not wait for it.
    if(reason=="unsupported_scan_motion"){
      DumpFlight(reason);
      event["flight_recorder"]=flightDumps_;
      if(flightDumpsSuppressed_)event["flight_recorder_suppressed"]=flightDumpsSuppressed_;
    }
    if(residualStats_ && residualStats_->samples())event["residual_statistics"]=ResidualSummary();
    event["encoder_output_mode"]=config_["encoder_output_mode"].as<std::string>("strict");
    event["encoder_max_retrace_m"]=trigger_->MaxRetrace();
    event["encoder_retrace_episodes"]=trigger_->RetraceEpisodes();
    std_msgs::msg::String message; message.data=event.dump(); statusPub_->publish(message);
    ArchiveEvent(std::move(event));
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
      ArchiveEvent(std::move(event));
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

  // Same interval, same quantities and same floor as tools/audit_scan_slip.py,
  // so the two cannot disagree about what counts as an alert. The event keeps
  // the terms rather than a verdict: the pitch-reference estimate that explains
  // these on a startup probe is explicitly provisional, and freezing it here
  // would lose the ability to re-derive it. The two-second window carries the
  // attitude and contact history the terms are read against.
  void CheckCameraEncoderGeometry() {
    if(!flight_ || tags_.size()<2 || geometryFloor_<=0 || spacing_<=0)return;
    const auto &a=tags_[tags_.size()-2];const auto &b=tags_.back();
    const double from=a["camera_position_world_m"][0].template get<double>();
    const double to=b["camera_position_world_m"][0].template get<double>();
    const auto measured=MeasureCameraEncoder(a["global_line"].template get<double>(),
        b["global_line"].template get<double>(),spacing_,from,to);
    if(!measured)return;
    std::optional<double> groundTravel;
    if(groundGeometry_) {
      auto ga=groundGeometry_->Hit(a),gb=groundGeometry_->Hit(b);
      if(ga&&gb)groundTravel=std::abs(gb->X()-ga->X());
    }
    std::string verdict=GroundVerdict(measured->encoder,groundTravel,geometryFloor_);
    if(groundGeometry_ && verdict=="pass")return;
    if(!groundGeometry_ && measured->ratio>=geometryFloor_)return;
    Json event={{"reason","camera_encoder_geometry_alert"},{"simulation_time_s",now_},
      {"geometry_reference",groundGeometry_?"triangular_heightfield_centre_ray_v1":"legacy_camera_centre"},
      {"ground_verdict",groundGeometry_?verdict:"legacy_alert"},{"minimum_encoder_m",.30},
      {"footprint_horizontal_m",groundTravel?Json(*groundTravel):Json(nullptr)},
      {"footprint_ratio",groundTravel?Json(*groundTravel/measured->encoder):Json(nullptr)},
      {"floor",geometryFloor_},{"lines",measured->lines},{"line_spacing_m",spacing_},
      {"encoder_m",measured->encoder},{"camera_horizontal_m",measured->travel},{"ratio",measured->ratio},
      {"camera_x_from_m",from},{"camera_x_to_m",to},
      {"first_line",a["global_line"]},{"last_line",b["global_line"]},
      // Both tag times, so the interval can be placed in the diagnostic window
      // exactly instead of being reconstructed from integrated wheel travel.
      {"first_time_s",a["time_s"]},{"last_time_s",b["time_s"]},
      {"camera_rotation_from",a["camera_rotation_world"]},
      {"camera_rotation_to",b["camera_rotation_world"]},
      {"camera_position_from_m",a["camera_position_world_m"]},
      {"camera_position_to_m",b["camera_position_world_m"]},
      {"block_id",block_},{"segment_id",segment_},{"capturing",active_}};
    if(const auto *newest=flight_->Newest()) {
      event["body_xyz"]={newest->bodyX,newest->bodyY,newest->bodyZ};
      event["body_quat_xyzw"]={newest->bodyQx,newest->bodyQy,newest->bodyQz,newest->bodyQw};
      event["mount_roll_pitch_rad"]=newest->mountAngle;
    }
    event["flight_recorder"]=DumpFlight("camera_encoder_geometry_alert",geometryDumps_,
                                        geometryDumpLimit_,geometryDumpsSuppressed_)?geometryDumps_:0;
    if(geometryDumpsSuppressed_)event["flight_recorder_suppressed"]=geometryDumpsSuppressed_;
    // Archive only, no /linescan/status publish. Anything published there with a
    // reason outside the expected set becomes CAMERA_<reason> and faults the
    // mission (agv_mission/capture.py). Capture is not interrupted here: this is
    // a measurement for review, and calling it a capture failure is precisely
    // what the geometry criterion exists to avoid. scan_residual_near_limit is
    // archived the same way for the same reason.
    ArchiveEvent(std::move(event));
  }

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
      if (tags_.back()["global_line"]!=tag["global_line"]) { tags_.push_back(std::move(tag)); CheckCameraEncoderGeometry(); }
    }
    tagAfterHold_=true;
  }

  void AddLine(const uint8_t *pixels,const Line &line,double sceneTime) {
    const size_t row=buffer_.size()/optics_->width;
    // A final residual encoder pulse may arrive after HOLD and consume its
    // first anchor. Anchor the actual long inter-line gap as well.
    if(row && line.event.time-lastLine_.event.time>maxTagGap_) MarkHoldBoundary();
    if (row%std::max(size_t(1),rows_/4)==0 || tagAfterHold_) {
      auto tag=Tag(line,sceneTime);
      if (!row) first_=tag;
      tags_.push_back(std::move(tag));
      CheckCameraEncoderGeometry();
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
    if (tags_.back()["global_line"]!=last_["global_line"]) { tags_.push_back(last_); CheckCameraEncoderGeometry(); }
    const size_t count=buffer_.size()/optics_->width;
    if(DiscardShortTail(count,reason=="full",minTailRows_)) {
      Json event={{"reason","tail_discarded"},{"end_reason",reason},{"rows",count},{"minimum_rows",minTailRows_},
        {"block_id",block_++},{"segment_id",segment_},{"first",first_},{"last",last_},{"simulation_time_s",lastSceneTime_}};
      std_msgs::msg::String message;message.data=event.dump();statusPub_->publish(message);
      ArchiveEvent(std::move(event));
      buffer_.clear();tags_=Json::array();invalidPixels_=0;return;
    }
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
    if(encoderWheel_>=0) {
      meta["encoder_distance_model"]="single_wheel_calibrated_arc";
      meta["wheel_encoder"]=encoderContract_;
    }
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
      meta["scan_residual_statistics"]=ResidualSummary();
      meta["capture_contact_statistics"]=ContactSummary();
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

  size_t minTailRows_=1000;
  double maxTagGap_=.1;
  YAML::Node config_;
  std::unique_ptr<Optics> optics_;
  std::unique_ptr<Trigger> trigger_;
  int encoderWheel_=-1,encoderPolarity_=1;
  Json encoderContract_;
  ProjectedEncoder projectedEncoder_;
  gz::sim::RenderUtil render_;
  std::vector<gz::rendering::CameraPtr> cameras_;
  bool flexible_=false;
  gz::sim::Entity cameraEntity_=gz::sim::kNullEntity;
  gz::sim::Entity model_=gz::sim::kNullEntity;
  std::array<gz::sim::Entity,4> drive_{},steer_{},suspension_{},wheelLink_{},wheelCollision_{},contactSensor_{};
  std::array<bool,4> contactDataAvailable_{};
  bool contactEvidenceReady_=false;
  bool contactEvidenceRequired_=true;
  bool probeKinematics_=false;
  std::unique_ptr<FlightRecorder> flight_;
  std::unique_ptr<ResidualStats> residualStats_;
  std::unique_ptr<CaptureContactStats> contactStats_;
  double residualHalfCooldown_=10.,lastHalfEvent_=-1e9;
  std::uint64_t halfEvents_=0,flightDumps_=0,flightDumpLimit_=20,flightDumpsSuppressed_=0;
  std::uint64_t geometryDumps_=0,geometryDumpLimit_=40,geometryDumpsSuppressed_=0;
  double geometryFloor_=.95;
  std::filesystem::path geometryPath_;
  std::unique_ptr<GroundGeometry> groundGeometry_;
  std::atomic<std::uint64_t> diagnosticWriteFailures_{0},diagnosticDropped_{0};
  std::thread diagnosticThread_;
  std::mutex diagnosticMutex_;
  std::condition_variable diagnosticWake_;
  std::deque<std::function<void()>> diagnosticJobs_;
  bool diagnosticQuit_=false;
  bool bodyPrevValid_=false;double bodyPrevTime_=0;gz::math::Pose3d bodyPrevPose_;
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr probeService_;
};
}  // namespace agv_linescan
GZ_ADD_PLUGIN(agv_linescan::GzLineScan,gz::sim::System,
              agv_linescan::GzLineScan::ISystemConfigure,
              agv_linescan::GzLineScan::ISystemPreUpdate,
              agv_linescan::GzLineScan::ISystemPostUpdate)
