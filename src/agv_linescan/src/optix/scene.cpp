#include "shared.h"
#include "scene_io.h"
#include <optix_stubs.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <algorithm>
#include <limits>
#define CU(x) do{auto e=(x);if(e!=cudaSuccess)throw std::runtime_error(std::string(#x)+": "+cudaGetErrorString(e));}while(0)
#define OX(x) do{auto e=(x);if(e!=OPTIX_SUCCESS)throw std::runtime_error(std::string(#x)+": "+std::to_string(e));}while(0)
namespace agv_linescan {
struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) Record {char header[OPTIX_SBT_RECORD_HEADER_SIZE];};
struct OptixScene::Impl {
 OptixDeviceContext context=nullptr;OptixModule module=nullptr;OptixPipeline pipeline=nullptr;
 OptixProgramGroup groups[4]={};OptixShaderBindingTable sbt={};cudaStream_t stream=nullptr;
 std::vector<CUdeviceptr> allocations;std::vector<std::string> links;
 ScanParams params={};CUdeviceptr dp=0;size_t capacity=0;
 unsigned char* host=nullptr;unsigned* invalidHost=nullptr;
 CUdeviceptr alloc(size_t n,const void* data=nullptr){CUdeviceptr p=0;CU(cudaMalloc((void**)&p,n));allocations.push_back(p);if(data)CU(cudaMemcpy((void*)p,data,n,cudaMemcpyHostToDevice));return p;}
 ~Impl(){
  if(stream)cudaStreamSynchronize(stream);
  if(pipeline)optixPipelineDestroy(pipeline);
  for(auto g:groups)if(g)optixProgramGroupDestroy(g);
  if(module)optixModuleDestroy(module);
  for(auto p:allocations)cudaFree((void*)p);
  if(host)cudaFreeHost(host);if(invalidHost)cudaFreeHost(invalidHost);
  if(stream)cudaStreamDestroy(stream);if(context)optixDeviceContextDestroy(context);
 }
 RayGeometry build(const std::vector<float3>& vertices,const std::vector<float>& reflectance){
  if(vertices.empty()||vertices.size()%3||reflectance.size()!=vertices.size()/3)throw std::runtime_error("invalid ray mesh");
  RayGeometry result={};result.lo=make_float3(INFINITY,INFINITY,INFINITY);result.hi=make_float3(-INFINITY,-INFINITY,-INFINITY);
  for(auto v:vertices){if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z))throw std::runtime_error("nonfinite mesh");
   result.lo=make_float3(std::min(result.lo.x,v.x),std::min(result.lo.y,v.y),std::min(result.lo.z,v.z));
   result.hi=make_float3(std::max(result.hi.x,v.x),std::max(result.hi.y,v.y),std::max(result.hi.z,v.z));}
  auto dv=alloc(vertices.size()*sizeof(float3),vertices.data());result.vertices=(float3*)dv;
  result.reflectance=(float*)alloc(reflectance.size()*sizeof(float),reflectance.data());
  unsigned flags=OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;OptixBuildInput input={};input.type=OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
  input.triangleArray.vertexFormat=OPTIX_VERTEX_FORMAT_FLOAT3;input.triangleArray.numVertices=vertices.size();input.triangleArray.vertexBuffers=&dv;input.triangleArray.flags=&flags;input.triangleArray.numSbtRecords=1;
  OptixAccelBuildOptions ab={};ab.buildFlags=OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;ab.operation=OPTIX_BUILD_OPERATION_BUILD;OptixAccelBufferSizes sizes;
  OX(optixAccelComputeMemoryUsage(context,&ab,&input,1,&sizes));CUdeviceptr scratch=alloc(sizes.tempSizeInBytes),gas=alloc(sizes.outputSizeInBytes);
  OX(optixAccelBuild(context,stream,&ab,&input,1,scratch,sizes.tempSizeInBytes,gas,sizes.outputSizeInBytes,&result.handle,nullptr,0));CU(cudaStreamSynchronize(stream));
  // Scratch is owned until construction completes / destruction; no dangling handle on exceptions.
  return result;
 }
};
OptixScene::OptixScene(const std::vector<float>& rays,const std::string& scene,const std::string& robot,
                      const std::string& ptxPath,size_t capacity,Radiometry sensor,unsigned lampSamples):impl_(std::make_unique<Impl>()){
 auto& s=*impl_;sensor.Validate();
 if(!lampSamples||lampSamples>256)throw std::invalid_argument("invalid LED sample count");
 s.params.lampSamples=lampSamples;
 if(rays.empty()||rays.size()>16384||!capacity||capacity>16384)throw std::invalid_argument("invalid OptiX capacity");
 for(auto r:rays)if(!std::isfinite(r))throw std::invalid_argument("nonfinite camera ray");
 s.capacity=capacity;s.params.width=rays.size();
 CU(cudaFree(0));OX(optixInit());OptixDeviceContextOptions options={};OX(optixDeviceContextCreate(nullptr,&options,&s.context));CU(cudaStreamCreate(&s.stream));
 auto vertices=LoadScene(scene);auto manifest=ReadJson(scene);std::vector<float> reflectance;
 for(auto a:manifest.at("assets")){float value=a.value("linear_reflectance",a.at("name").get<std::string>()=="terrain"?-1.f:.45f);if(!std::isfinite(value)||value>1||(a.contains("linear_reflectance")&&value<0))throw std::runtime_error("invalid static reflectance");reflectance.insert(reflectance.end(),a.at("triangles").get<size_t>(),value);}
 std::vector<RayGeometry> geometry={s.build(vertices,reflectance)};
 auto description=ReadJson(robot);
 if(description.at("schema")!="agv.robot.ray_scene.v1" || Sha256(std::filesystem::path(robot).parent_path()/description.at("source_urdf").get<std::string>())!=description.at("source_sha256"))throw std::runtime_error("robot source contract mismatch");
 for(auto group:description.at("groups")){
  auto name=group.at("name").get<std::string>();if(name.empty()||std::find(s.links.begin(),s.links.end(),name)!=s.links.end())throw std::runtime_error("duplicate/empty robot link");s.links.push_back(name);
  std::vector<float3> v;for(auto p:group.at("vertices")){if(p.size()!=3)throw std::runtime_error("invalid robot vertex");v.push_back(make_float3(p[0],p[1],p[2]));}
  auto values=group.at("reflectance").get<std::vector<float>>();for(float a:values)if(!std::isfinite(a)||a<0||a>1)throw std::runtime_error("invalid reflectance");
  geometry.push_back(s.build(v,values));
 }
 if(s.links.empty()||s.links.size()>64)throw std::runtime_error("invalid robot link count");
 s.params.lampLink=-1;
 if(description.contains("led_emitters")){
  auto light=description.at("led_emitters");auto it=std::find(s.links.begin(),s.links.end(),light.at("link").get<std::string>());
  if(it==s.links.end()||light.at("positions_m").size()!=4)throw std::runtime_error("invalid LED attachment");
  s.params.lampLink=it-s.links.begin();
  float3 original[4];
  for(int i=0;i<4;++i){
   auto p=light.at("positions_m")[i];if(p.size()!=3)throw std::runtime_error("invalid emitter");
   original[i]=make_float3(p[0],p[1],p[2]);
   for(auto x:p)if(!std::isfinite(x.get<float>()))throw std::runtime_error("nonfinite emitter");
  }
  auto step=mul(sub(original[3],original[0]),1.f/3);
  for(int i=1;i<3;++i){
   auto error=sub(original[i],add(original[0],mul(step,float(i))));
   if(dot(error,error)>1e-10f)throw std::runtime_error("LED samples must be equally spaced on a straight bar");
  }
  // Original positions are four cell midpoints, not the physical endpoints.
  for(unsigned i=0;i<lampSamples;++i)
   s.params.emitters[i]=lampSamples==4?original[i]:add(original[0],mul(step,4.f*(i+.5f)/lampSamples-.5f));
 }
 s.params.links=s.links.size();s.params.geometry=(RayGeometry*)s.alloc(geometry.size()*sizeof(RayGeometry),geometry.data());
 std::ifstream file(ptxPath);std::string ptx((std::istreambuf_iterator<char>(file)),{});if(ptx.empty())throw std::runtime_error("empty OptiX PTX");
 OptixPipelineCompileOptions pc={};pc.traversableGraphFlags=OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;pc.numPayloadValues=2;pc.numAttributeValues=2;pc.pipelineLaunchParamsVariableName="params";pc.usesPrimitiveTypeFlags=OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
 OptixModuleCompileOptions mc={};char log[8192];size_t logSize=sizeof(log);
 OX(optixModuleCreate(s.context,&mc,&pc,ptx.data(),ptx.size(),log,&logSize,&s.module));
 OptixProgramGroupDesc desc[4]={};OptixProgramGroupOptions go={};
 desc[0].kind=OPTIX_PROGRAM_GROUP_KIND_RAYGEN;desc[0].raygen.module=s.module;desc[0].raygen.entryFunctionName="__raygen__scan";
 for(int i=1;i<3;++i){desc[i].kind=OPTIX_PROGRAM_GROUP_KIND_MISS;desc[i].miss.module=s.module;desc[i].miss.entryFunctionName=i==1?"__miss__primary":"__miss__shadow";}
 desc[3].kind=OPTIX_PROGRAM_GROUP_KIND_HITGROUP;desc[3].hitgroup.moduleCH=s.module;desc[3].hitgroup.entryFunctionNameCH="__closesthit__primary";
 logSize=sizeof(log);OX(optixProgramGroupCreate(s.context,desc,4,&go,log,&logSize,s.groups));OptixPipelineLinkOptions pl={};pl.maxTraceDepth=1;
 logSize=sizeof(log);OX(optixPipelineCreate(s.context,&pc,&pl,s.groups,4,log,&logSize,&s.pipeline));
 OptixStackSizes stack={};for(auto g:s.groups)OX(optixUtilAccumulateStackSizes(g,&stack,s.pipeline));unsigned a,b,c;OX(optixUtilComputeStackSizes(&stack,1,0,0,&a,&b,&c));OX(optixPipelineSetStackSize(s.pipeline,a,b,c,1));
 Record records[4]={};for(int i=0;i<4;++i)OX(optixSbtRecordPackHeader(s.groups[i],records+i));auto dr=s.alloc(sizeof(records),records);
 s.sbt.raygenRecord=dr;s.sbt.missRecordBase=dr+sizeof(Record);s.sbt.missRecordCount=2;s.sbt.missRecordStrideInBytes=sizeof(Record);s.sbt.hitgroupRecordBase=dr+3*sizeof(Record);s.sbt.hitgroupRecordCount=1;s.sbt.hitgroupRecordStrideInBytes=sizeof(Record);
 s.params.rays=(float*)s.alloc(rays.size()*sizeof(float),rays.data());s.params.sensor=(Radiometry*)s.alloc(sizeof(sensor),&sensor);
 s.params.poses=(GridExposure*)s.alloc(capacity*sizeof(GridExposure));s.params.transforms=(LinkTransform*)s.alloc(capacity*3*s.links.size()*sizeof(LinkTransform));
 s.params.pixels=(unsigned char*)s.alloc(capacity*rays.size());s.params.invalid=(unsigned*)s.alloc(capacity*sizeof(unsigned));s.dp=s.alloc(sizeof(ScanParams));
 CU(cudaMallocHost((void**)&s.host,capacity*rays.size()));CU(cudaMallocHost((void**)&s.invalidHost,capacity*sizeof(unsigned)));
}
OptixScene::~OptixScene()=default;
const std::vector<std::string>& OptixScene::Links()const{return impl_->links;}
GridBatch OptixScene::Sample(const std::vector<GridExposure>& poses,const std::vector<LinkTransform>& transforms,uint64_t first){
 auto& s=*impl_;size_t n=poses.size();if(!n||n>s.capacity||transforms.size()!=n*3*s.links.size()||first>UINT64_MAX-(n-1))throw std::invalid_argument("invalid OptiX sampling batch");
 for(const auto& p:poses)for(const auto& sample:p.samples)for(size_t j=0;j<3;++j)
  if(!std::isfinite(sample.origin[j])||!std::isfinite(sample.across[j])||!std::isfinite(sample.down[j]))throw std::invalid_argument("nonfinite camera pose");
 // Checked each batch: these are dynamic world-to-link transforms, not mesh scales.
 for(const auto& m:transforms){
  for(auto x:m)if(!std::isfinite(x))throw std::invalid_argument("nonfinite link pose");
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){
   double dot=0;for(int k=0;k<3;++k)dot+=double(m[4*i+k])*m[4*j+k];
   if(std::abs(dot-(i==j?1.:0.))>1e-4)throw std::invalid_argument("link pose is not rigid: nonorthogonal rotation");
  }
  double det=double(m[0])*(double(m[5])*m[10]-double(m[6])*m[9])
            -double(m[1])*(double(m[4])*m[10]-double(m[6])*m[8])
            +double(m[2])*(double(m[4])*m[9]-double(m[5])*m[8]);
  if(std::abs(det-1)>1e-4)throw std::invalid_argument("link pose is not rigid: improper rotation");
 }
 s.params.first=first;
 CU(cudaMemcpyAsync((void*)s.params.poses,poses.data(),n*sizeof(GridExposure),cudaMemcpyHostToDevice,s.stream));
 CU(cudaMemcpyAsync((void*)s.params.transforms,transforms.data(),transforms.size()*sizeof(LinkTransform),cudaMemcpyHostToDevice,s.stream));
 CU(cudaMemsetAsync(s.params.invalid,0,n*sizeof(unsigned),s.stream));CU(cudaMemcpyAsync((void*)s.dp,&s.params,sizeof(s.params),cudaMemcpyHostToDevice,s.stream));
 OX(optixLaunch(s.pipeline,s.stream,s.dp,sizeof(s.params),&s.sbt,s.params.width,n,1));
 CU(cudaMemcpyAsync(s.host,s.params.pixels,n*s.params.width,cudaMemcpyDeviceToHost,s.stream));CU(cudaMemcpyAsync(s.invalidHost,s.params.invalid,n*sizeof(unsigned),cudaMemcpyDeviceToHost,s.stream));CU(cudaStreamSynchronize(s.stream));
 if(std::any_of(s.invalidHost,s.invalidHost+n,[](auto v){return v!=0;}))throw std::runtime_error("OptiX batch rejected: missing primary exposure sample");
 return {{s.host,s.host+n*s.params.width},{s.invalidHost,s.invalidHost+n}};
}
}
