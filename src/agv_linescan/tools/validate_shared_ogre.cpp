// GZ rendering scene built from the exported SDF, not from sampler geometry arrays.
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/rendering/MeshDescriptor.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Camera.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Visual.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
using Json=nlohmann::json;
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("WORLD_SDF OUTPUT_JSON");
 sdf::Root root;auto errors=root.Load(argv[1]);if(!errors.empty())throw std::runtime_error(errors.front().Message());
 auto world=root.WorldByIndex(0);if(!world)throw std::runtime_error("missing world");
 auto engine=gz::rendering::engine("ogre2");if(!engine)throw std::runtime_error("Ogre2 init failed");
 auto scene=engine->CreateScene("shared_geometry_check");scene->SetAmbientLight(1,1,1);
 std::map<unsigned,std::string> names;
 for(uint64_t i=0;i<world->ModelCount();++i){auto model=world->ModelByIndex(i);auto link=model->LinkByIndex(0);auto visual=link->VisualByIndex(0);
  if(!model->Static()||model->RawPose()!=gz::math::Pose3d::Zero||link->RawPose()!=gz::math::Pose3d::Zero||visual->RawPose()!=gz::math::Pose3d::Zero)throw std::runtime_error("nonidentity/static scene");
  auto v=scene->CreateVisual(model->Name());gz::rendering::MeshDescriptor desc;desc.meshName=visual->Geom()->MeshShape()->Uri();desc.centerSubMesh=false;desc.mesh=gz::common::MeshManager::Instance()->Load(desc.meshName);
  auto mesh=scene->CreateMesh(desc);if(!mesh)throw std::runtime_error("mesh import failed");v->AddGeometry(mesh);v->SetLocalScale(visual->Geom()->MeshShape()->Scale());scene->RootVisual()->AddChild(v);names[v->Id()]=model->Name();
 }
 // Render a real Ogre2 image as an import smoke check. Geometry queries below use CPU mode.
 auto camera=scene->CreateCamera("overview");scene->RootVisual()->AddChild(camera);camera->SetImageWidth(640);camera->SetImageHeight(480);camera->SetHFOV(1.2);camera->SetAspectRatio(640./480);camera->SetWorldPose(gz::math::Pose3d(3,-2,2,0,.65,1.0));camera->SetImageFormat(gz::rendering::PF_R8G8B8);auto image=camera->CreateImage();camera->Capture(image);
 std::ofstream pic(std::string(argv[2])+".ppm",std::ios::binary);pic<<"P6\n640 480\n255\n";pic.write((const char*)image.Data<unsigned char>(),640*480*3);pic.close();
 auto query=scene->CreateRayQuery();query->SetPreferGpu(false);Json rays=Json::array();
 auto cast=[&](std::string kind,gz::math::Vector3d o,gz::math::Vector3d d,double limit){
  d.Normalize();query->SetOrigin(o);query->SetDirection(d);auto hit=query->ClosestPoint();bool valid=hit&&hit.distance<limit;
  std::string name=valid&&names.count(hit.objectId)?names.at(hit.objectId):"";
  if(valid&&name.empty())throw std::runtime_error("unknown Ogre object ID "+std::to_string(hit.objectId));
  rays.push_back({{"kind",kind},{"origin",{o.X(),o.Y(),o.Z()}},{"direction",{d.X(),d.Y(),d.Z()}},{"max_distance",limit},{"distance",valid?hit.distance:-1},{"object",name}});
  return hit;
 };
 for(double x:{4.33,4.97,5.121,5.63,7.83,9.13,95.33})for(double y:{-.57,-.31,-.13,.017,.17,.34,.59}){
  auto hit=cast("camera",{x,0,.8370535714},{0,y,-.8370535714},5);
  if(hit)for(double ly:{-.525,-.175,.175,.525}){
   auto o=hit.point+gz::math::Vector3d(0,0,.0002);auto delta=gz::math::Vector3d(x+.25,ly,.30)-o;
   cast("led_visibility",o,delta,delta.Length()-.0001);
  }
 }
 cast("background",{-2,0,1},{0,0,-1},5);
 std::ofstream f(argv[2]);f<<Json({{"world",argv[1]},{"backend","Ogre2 CPU RayQuery after actual mesh import and image render"},{"queries",rays}}).dump(2)<<'\n';
 engine->DestroyScene(scene);std::cout<<"Checked "<<rays.size()<<" rays from "<<names.size()<<" SDF visuals\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
