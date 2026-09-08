#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/cameratrack.pb.h>

namespace agv {
// Consume wheel events only while following, changing the persistent follow
// offset instead of briefly moving a camera that is tethered to a fixed offset.
class FollowCameraZoom : public gz::gui::Plugin {
 public: void LoadConfig(const tinyxml2::XMLElement *) override {
   publisher_=node_.Advertise<gz::msgs::CameraTrack>("/gui/track");
   gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
 }
 protected: bool eventFilter(QObject *object,QEvent *event) override {
   if(event->type()==gz::gui::events::ScrollOnScene::kType && following_.load()) {
     auto *scroll=static_cast<gz::gui::events::ScrollOnScene *>(event);
     std::lock_guard<std::mutex> lock(mutex_);
     wheel_+=scroll->Mouse().Scroll().Y();
     return true;
   }
   if(event->type()==gz::gui::events::Render::kType) {
     if(!camera_) {
       auto scene=gz::rendering::sceneFromFirstRenderEngine();
       if(scene)for(unsigned i=0;i<scene->NodeCount();++i) {
         auto camera=std::dynamic_pointer_cast<gz::rendering::Camera>(scene->NodeByIndex(i));
         if(camera) {camera_=camera;break;}
       }
     }
     if(camera_) {
       bool follows=bool(camera_->FollowTarget());following_.store(follows);
       double wheel;
       {std::lock_guard<std::mutex> lock(mutex_);wheel=wheel_;wheel_=0;}
       if(follows && wheel!=0) {
         auto offset=camera_->FollowOffset();double distance=offset.Length();
         if(distance>1e-6) {
           double next=std::clamp(distance*std::exp(std::clamp(wheel,-20.,20.)*.15),.8,80.);
           offset*=next/distance;
           camera_->SetFollowOffset(offset);
           // Keep CameraTracking's advertised state and future updates coherent.
           gz::msgs::CameraTrack message;message.set_track_mode(gz::msgs::CameraTrack::USE_LAST);
           auto *p=message.mutable_follow_offset();p->set_x(offset.X());p->set_y(offset.Y());p->set_z(offset.Z());
           publisher_.Publish(message);
         }
       }
     }
   }
   return QObject::eventFilter(object,event);
 }
 private: gz::transport::Node node_;
 private: gz::transport::Node::Publisher publisher_;
 private: gz::rendering::CameraPtr camera_;
 private: std::mutex mutex_;
 private: double wheel_=0;
 private: std::atomic<bool> following_{false};
};
}
GZ_ADD_PLUGIN(agv::FollowCameraZoom,gz::gui::Plugin)
