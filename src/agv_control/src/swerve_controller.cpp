#include "agv_control/swerve.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <vector>

class SwerveNode : public rclcpp::Node {
 public:
  SwerveNode():Node("swerve_controller") {
    agv::Config c;
    c.wheelbase=declare_parameter("wheelbase",c.wheelbase);c.track=declare_parameter("track",c.track);
    c.radius=declare_parameter("wheel_radius",c.radius);radius_=c.radius;
    c.soft=declare_parameter("steer_soft_limit",c.soft);
    hard_=declare_parameter("steer_hard_limit",190*agv::pi/180);
    c.rate=declare_parameter("steer_rate",c.rate);c.steer_accel=declare_parameter("steer_accel",c.steer_accel);
    c.max_speed=declare_parameter("max_speed",c.max_speed);c.max_yaw=declare_parameter("max_yaw_rate",c.max_yaw);
    c.accel=declare_parameter("drive_accel",c.accel);
    c.decel=declare_parameter("drive_decel",c.decel);
    c.reorient=declare_parameter("reorient_angle",c.reorient);c.aligned=declare_parameter("aligned_angle",c.aligned);
    c.lateral_mismatch=declare_parameter("max_lateral_mismatch",c.lateral_mismatch);
    c.max_lateral_speed=declare_parameter("max_lateral_speed",c.max_lateral_speed);
    c.stopped=declare_parameter("stopped_speed",c.stopped);
    c.alignment_motion_confirm_s=declare_parameter("alignment_motion_confirm_s",c.alignment_motion_confirm_s);
    c.limit_reserve=declare_parameter("steering_limit_reserve",c.limit_reserve);
    c.segment_margin=declare_parameter("steering_segment_margin",c.segment_margin);
    c.wheel_deadband=declare_parameter("wheel_speed_deadband",c.wheel_deadband);
    timeout_=declare_parameter("command_timeout",0.4);
    if(!std::isfinite(hard_)||hard_<=c.soft||!std::isfinite(timeout_)||timeout_<=0)
      throw std::invalid_argument("invalid limits or timeout");
    core_=std::make_unique<agv::Controller>(c);
    steer_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("steering_controller/commands",10);
    drive_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands",10);
    state_pub_=create_publisher<std_msgs::msg::String>("motion_state",10);
    reason_pub_=create_publisher<std_msgs::msg::String>("motion_transition_reason",10);
    command_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel",10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){
        const auto &t=m->twist;const double age=(now()-rclcpp::Time(m->header.stamp)).seconds();
        if(m->header.frame_id!="base_link" || age< -0.05 || age>timeout_ ||
           !std::isfinite(t.linear.x)||!std::isfinite(t.linear.y)||!std::isfinite(t.angular.z)||
           t.linear.z!=0||t.angular.x!=0||t.angular.y!=0) {command_={}; command_valid_=false;return;}
        command_={t.linear.x,t.linear.y,t.angular.z};command_stamp_=rclcpp::Time(m->header.stamp);command_valid_=true;
      });
    joint_sub_=create_subscription<sensor_msgs::msg::JointState>("joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){
        agv::Four a{},v{},sr{}; bool ok=true;
        for(size_t i=0;i<4;++i) {
          for(int type=0;type<2;++type) {
            const std::string joint=names_[i]+(type==0?"_steer_joint":"_drive_joint");
            auto it=std::find(m->name.begin(),m->name.end(),joint);
            const size_t j=std::distance(m->name.begin(),it);
            if(it==m->name.end()||j>=m->velocity.size()||(type==0&&j>=m->position.size())){ok=false;continue;}
            if(type==0){a[i]=m->position[j];sr[i]=m->velocity[j];}else v[i]=m->velocity[j]*radius_;
          }
          ok &= std::isfinite(a[i])&&std::isfinite(v[i])&&std::isfinite(sr[i])&&std::abs(a[i])<=hard_+0.01;
        }
        if(ok){angles_=a;speeds_=v;steer_rates_=sr;feedback_stamp_=rclcpp::Time(m->header.stamp);have_feedback_=true;}
        else have_feedback_=false;
      });
    last_=now();
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{tick();});
  }
 private:
  void tick(){
    const auto time=now();const double dt=(time-last_).seconds();
    const auto wall=std::chrono::steady_clock::now();
    if(dt==0) {
      // A stopped ROS clock may mean a lost clock bridge while physics still
      // runs. Do not let the previous motor velocities persist indefinitely.
      if(std::chrono::duration<double>(wall-clock_wall_).count()>timeout_) {
        std_msgs::msg::Float64MultiArray stop;stop.data={0,0,0,0};drive_pub_->publish(stop);
        command_valid_=false;core_->reset();
        std_msgs::msg::String state;state.data="FEEDBACK_HOLD";state_pub_->publish(state);
      }
      return;
    }
    clock_wall_=wall;
    last_=time;
    const double feedback_age=(time-feedback_stamp_).seconds();
    if(dt<0||dt>0.1||!have_feedback_||feedback_age>0.2||feedback_age< -0.05){
      std_msgs::msg::Float64MultiArray stop;stop.data={0,0,0,0};drive_pub_->publish(stop);
      command_valid_=false;
      core_->reset();
      std_msgs::msg::String s;s.data="FEEDBACK_HOLD";state_pub_->publish(s);return;
    }
    const double age=(time-command_stamp_).seconds();
    const auto request=command_valid_&&age>= -0.05&&age<=timeout_?command_:agv::Twist{};
    auto out=core_->update(request,angles_,speeds_,dt,steer_rates_);
    std_msgs::msg::Float64MultiArray a,v;
    for(size_t i=0;i<4;++i){a.data.push_back(out.angle[i]);v.data.push_back(out.speed[i]/radius_);}
    steer_pub_->publish(a);drive_pub_->publish(v);
    std_msgs::msg::String state;state.data=agv::name(core_->mode());state_pub_->publish(state);
    std_msgs::msg::String reason;reason.data=core_->reason();reason_pub_->publish(reason);
  }
  const std::array<std::string,4> names_{"fl","fr","rl","rr"};
  std::unique_ptr<agv::Controller> core_;
  agv::Four angles_{},speeds_{},steer_rates_{};agv::Twist command_;
  double radius_{},hard_{},timeout_{};bool have_feedback_{false},command_valid_{false};
  std::chrono::steady_clock::time_point clock_wall_{std::chrono::steady_clock::now()};
  rclcpp::Time last_{0,0,RCL_ROS_TIME},feedback_stamp_{0,0,RCL_ROS_TIME},command_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_,drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_,reason_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<SwerveNode>());rclcpp::shutdown();}
