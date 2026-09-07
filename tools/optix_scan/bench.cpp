#include "shared.h"
#include "scene_io.h"
#include <optix_stubs.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <filesystem>
#include <nlohmann/json.hpp>
using Clock=std::chrono::steady_clock;
using Json=nlohmann::json;
#define CU(x) do{auto e=(x);if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}while(0)
#define OX(x) do{auto e=(x);if(e!=OPTIX_SUCCESS)throw std::runtime_error(std::string(#x)+":"+std::to_string(e));}while(0)
struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) Record {char header[OPTIX_SBT_RECORD_HEADER_SIZE];};
CUdeviceptr alloc(size_t n,const void* data=nullptr){CUdeviceptr p;CU(cudaMalloc((void**)&p,n));if(data)CU(cudaMemcpy((void*)p,data,n,cudaMemcpyHostToDevice));return p;}
void logcb(unsigned level,const char* tag,const char* message,void*){if(level<3)std::cerr<<tag<<":"<<message<<"\n";}
// Independent brute-force Moller-Trumbore CPU traversal, no OptiX/BVH calls.
bool intersect(const std::vector<float3>& v,float3 o,float3 d,float lo,float hi,unsigned& id,float& t){
 bool hit=false;t=hi;
 for(unsigned i=0;i<v.size()/3;++i){auto a=v[i*3],e1=sub(v[i*3+1],a),e2=sub(v[i*3+2],a);
  auto p=cross(d,e2);float det=dot(e1,p);if(fabsf(det)<1e-9f)continue;
  float inv=1/det;auto s=sub(o,a);float u=dot(s,p)*inv;if(u<0||u>1)continue;
  auto q=cross(s,e1);float w=dot(d,q)*inv;if(w<0||u+w>1)continue;
  float z=dot(e2,q)*inv;if(z>=lo&&z<t){hit=true;t=z;id=i;}
 }return hit;
}
int referencePixel(const std::vector<float3>& v,const Params& cfg,uint64_t line,int col){
 agv_linescan::Radiometry sensor; sensor.noise=cfg.noise;
 float sum=0;
 for(int k=0;k<3;++k){auto o=origin(line,k),d=direction(col);unsigned id;float t;
  if(!intersect(v,o,d,0,100,id,t))return 0;
  auto p=add(o,mul(d,t));auto n=unit(cross(sub(v[id*3+1],v[id*3]),sub(v[id*3+2],v[id*3])));if(dot(n,d)>0)n=mul(n,-1);
  float total=0,ref=0;
  for(int l=0;l<cfg.lights;++l){auto light=lamp(o,l,cfg.lights),delta=sub(light,p);float dist=sqrtf(dot(delta,delta));unsigned sid;float st;
   bool block=cfg.shadows&&intersect(v,add(p,mul(n,1e-4f)),unit(delta),1e-4f,dist-2e-4f,sid,st);
   if(!block)total+=lightWeight(o,p,n,light);
   ref+=lightWeight(o,make_float3(o.x,0,0),make_float3(0,0,1),light);
  }
  sum+=agv_linescan::MeanElectrons(sensor,reflectance(p),40*total/ref,1,qcol(col),col);
 }return agv_linescan::SensorCode(sensor,sum/3,line,col);
}
int main(int argc,char**argv){try{
 if(argc!=6 && argc!=7)throw std::runtime_error("usage: bench PTX OUTPUT_DIR DURATION_SECONDS BATCH_ROWS SCENE_MANIFEST [GZ_QUERY_JSON]");
 auto contract=ReadJson(argv[5]);
 if(argc==6 && (contract.at("length_m").get<double>()<100 || contract.at("width_m").get<double>()<10))throw std::runtime_error("benchmark trajectory requires at least 100x10m scene");
 auto out=std::filesystem::path(argv[2]);if(std::filesystem::exists(out))throw std::runtime_error("output exists");std::filesystem::create_directories(out);
 double duration=std::stod(argv[3]);int rows=std::stoi(argv[4]);if(duration<1||duration>120||rows<1||rows>512)throw std::runtime_error("invalid benchmark bounds");
 CU(cudaFree(0));OX(optixInit());OptixDeviceContext ctx;OptixDeviceContextOptions opts={};opts.logCallbackFunction=logcb;opts.logCallbackLevel=3;
 OX(optixDeviceContextCreate(nullptr,&opts,&ctx));auto begin=Clock::now();auto vertices=LoadScene(argv[5]);auto dv=alloc(vertices.size()*sizeof(float3),vertices.data());
 unsigned flags=OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT;OptixBuildInput input={};input.type=OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
 input.triangleArray.vertexFormat=OPTIX_VERTEX_FORMAT_FLOAT3;input.triangleArray.numVertices=vertices.size();input.triangleArray.vertexBuffers=&dv;input.triangleArray.flags=&flags;input.triangleArray.numSbtRecords=1;
 OptixAccelBuildOptions ab={};ab.buildFlags=OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;ab.operation=OPTIX_BUILD_OPERATION_BUILD;OptixAccelBufferSizes sizes;
 OX(optixAccelComputeMemoryUsage(ctx,&ab,&input,1,&sizes));auto scratch=alloc(sizes.tempSizeInBytes),gas=alloc(sizes.outputSizeInBytes);OptixTraversableHandle handle;
 OX(optixAccelBuild(ctx,0,&ab,&input,1,scratch,sizes.tempSizeInBytes,gas,sizes.outputSizeInBytes,&handle,nullptr,0));CU(cudaDeviceSynchronize());CU(cudaFree((void*)scratch));
 double buildSeconds=std::chrono::duration<double>(Clock::now()-begin).count();
 std::ifstream f(argv[1]);std::string ptx((std::istreambuf_iterator<char>(f)),{});if(ptx.empty())throw std::runtime_error("empty PTX");
 OptixPipelineCompileOptions pc={};pc.traversableGraphFlags=OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;pc.numPayloadValues=2;pc.numAttributeValues=2;pc.pipelineLaunchParamsVariableName="params";pc.usesPrimitiveTypeFlags=OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
 OptixModuleCompileOptions mc={};OptixModule module;char log[8192];size_t logSize=sizeof(log);
 OX(optixModuleCreate(ctx,&mc,&pc,ptx.data(),ptx.size(),log,&logSize,&module));
 OptixProgramGroup groups[4];OptixProgramGroupDesc desc[4]={};OptixProgramGroupOptions go={};
 desc[0].kind=OPTIX_PROGRAM_GROUP_KIND_RAYGEN;desc[0].raygen.module=module;desc[0].raygen.entryFunctionName="__raygen__scan";
 for(int i=1;i<3;++i){desc[i].kind=OPTIX_PROGRAM_GROUP_KIND_MISS;desc[i].miss.module=module;desc[i].miss.entryFunctionName=i==1?"__miss__primary":"__miss__shadow";}
 desc[3].kind=OPTIX_PROGRAM_GROUP_KIND_HITGROUP;desc[3].hitgroup.moduleCH=module;desc[3].hitgroup.entryFunctionNameCH="__closesthit__primary";
 logSize=sizeof(log);OX(optixProgramGroupCreate(ctx,desc,4,&go,log,&logSize,groups));OptixPipelineLinkOptions pl={};pl.maxTraceDepth=1;OptixPipeline pipeline;
 logSize=sizeof(log);OX(optixPipelineCreate(ctx,&pc,&pl,groups,4,log,&logSize,&pipeline));
 OptixStackSizes stack={};for(auto g:groups)OX(optixUtilAccumulateStackSizes(g,&stack,pipeline));unsigned a,b,c;OX(optixUtilComputeStackSizes(&stack,1,0,0,&a,&b,&c));OX(optixPipelineSetStackSize(pipeline,a,b,c,1));
 Record records[4]={};for(int i=0;i<4;++i)OX(optixSbtRecordPackHeader(groups[i],records+i));auto dr=alloc(sizeof(records),records);
 OptixShaderBindingTable sbt={};sbt.raygenRecord=dr;sbt.missRecordBase=dr+sizeof(Record);sbt.missRecordCount=2;sbt.missRecordStrideInBytes=sizeof(Record);sbt.hitgroupRecordBase=dr+3*sizeof(Record);sbt.hitgroupRecordCount=1;sbt.hitgroupRecordStrideInBytes=sizeof(Record);
 Params p={};p.handle=handle;p.vertices=(float3*)dv;p.image=(unsigned char*)alloc(4096*4096);p.lights=4;p.shadows=1;p.noise=false;
 auto dp=alloc(sizeof(p));unsigned char* host;CU(cudaMallocHost((void**)&host,4096*4096));cudaStream_t stream;CU(cudaStreamCreate(&stream));
 auto launch=[&](int n){CU(cudaMemcpyAsync((void*)dp,&p,sizeof(p),cudaMemcpyHostToDevice,stream));OX(optixLaunch(pipeline,stream,dp,sizeof(p),&sbt,4096,n,1));CU(cudaMemcpyAsync(host,p.image,4096*n,cudaMemcpyDeviceToHost,stream));CU(cudaStreamSynchronize(stream));};
 if(argc==7){
  auto source=ReadJson(argv[6]);std::vector<QueryRay> rays;
  for(const auto&r:source.at("queries")){
   auto o=r.at("origin"),d=r.at("direction");rays.push_back({make_float3(o[0],o[1],o[2]),make_float3(d[0],d[1],d[2]),r.at("max_distance").get<float>()});
  }
  if(rays.empty()||rays.size()>4096)throw std::runtime_error("invalid query count");
  p.queryCount=rays.size();p.queryRays=(QueryRay*)alloc(rays.size()*sizeof(QueryRay),rays.data());p.queryHits=(QueryHit*)alloc(rays.size()*sizeof(QueryHit));
  CU(cudaMemcpy((void*)dp,&p,sizeof(p),cudaMemcpyHostToDevice));OX(optixLaunch(pipeline,stream,dp,sizeof(p),&sbt,rays.size(),1,1));CU(cudaStreamSynchronize(stream));
  std::vector<QueryHit> hits(rays.size());CU(cudaMemcpy(hits.data(),p.queryHits,hits.size()*sizeof(QueryHit),cudaMemcpyDeviceToHost));
  Json result=Json::array();auto manifest=ReadJson(argv[5]);
  for(auto h:hits){std::string name;unsigned offset=0;
   if(h.distance>=0)for(auto asset:manifest.at("assets")){unsigned n=asset.at("triangles");if(h.primitive>=offset&&h.primitive<offset+n){name=asset.at("name");break;}offset+=n;}
   result.push_back({{"distance",h.distance},{"primitive",h.primitive},{"object",name}});
  }
  std::ofstream(out/"query_results.json")<<result.dump(2)<<'\n';
  CU(cudaFree(p.queryRays));CU(cudaFree(p.queryHits));
  CU(cudaStreamDestroy(stream));CU(cudaFreeHost(host));CU(cudaFree(p.image));CU(cudaFree((void*)dp));CU(cudaFree((void*)dr));OX(optixPipelineDestroy(pipeline));for(auto g:groups)OX(optixProgramGroupDestroy(g));OX(optixModuleDestroy(module));CU(cudaFree((void*)gas));CU(cudaFree((void*)dv));OX(optixDeviceContextDestroy(ctx));return 0;
 }
 // Validation scan spans the pit, screen and raised occluder.
 p.first=8192;launch(4096);std::vector<unsigned char> shadow(host,host+4096*4096);
 int maxError=0;Json checks=Json::array();
 for(int r: {12,2017,3001,4001})for(int u:{317,1733,2461,3791}){
  int expected=referencePixel(vertices,p,p.first+r,u),actual=host[r*4096+u];maxError=std::max(maxError,abs(expected-actual));checks.push_back({{"row",r},{"column",u},{"cpu",expected},{"gpu",actual}});
 }
 if(maxError>2)throw std::runtime_error("CPU geometry/shadow oracle mismatch: "+std::to_string(maxError));
 auto save=[&](const char* name,const unsigned char* data){std::ofstream s(out/name,std::ios::binary);s<<"P5\n4096 4096\n255\n";s.write((const char*)data,4096*4096);if(!s)throw std::runtime_error("image write failed");};save("shadow.pgm",host);
 p.shadows=0;launch(4096);save("unshadowed.pgm",host);size_t changed=0;int maxDrop=0;for(size_t i=0;i<shadow.size();++i){if(int(host[i])-shadow[i]>3)++changed;maxDrop=std::max(maxDrop,int(host[i])-shadow[i]);}if(changed<100)throw std::runtime_error("occluder has no measurable shadow");
 p.shadows=1;for(int offset=0;offset<4096;offset+=rows){p.first=8192+offset;int n=std::min(rows,4096-offset);launch(n);if(!std::equal(host,host+4096*n,shadow.data()+4096*offset))throw std::runtime_error("partition mismatch");}
 Json report={{"scope","OptiX static 100x10m triangle mesh, procedural grid, finite strip point quadrature, sensor response, host readback; no GZ/ROS/archive/streaming/dynamic updates"},{"triangles",vertices.size()/3},{"gas_bytes",sizes.outputSizeInBytes},{"scene_upload_build_seconds",buildSeconds},{"width",4096},{"exposure_samples",3},{"exposure_s",.00002},{"batch_rows",rows},{"cpu_oracle_max_dn_error",maxError},{"cpu_checks",checks},{"shadow_changed_pixels",changed},{"shadow_max_drop_dn",maxDrop},{"partition_identical",true},{"full_acceptance_passed",false}};
 report["scene_manifest"]=std::filesystem::absolute(argv[5]).string();
 report["scene_manifest_sha256"]=Sha256(argv[5]);
 report["cases"]=Json::array();p.noise=true;
 p.first=8192;launch(4096);std::vector<unsigned char> noisy(host,host+4096*4096);
 for(int offset=0;offset<4096;offset+=rows){p.first=8192+offset;int n=std::min(rows,4096-offset);launch(n);if(!std::equal(host,host+4096*n,noisy.data()+4096*offset))throw std::runtime_error("noise partition mismatch");}
 report["noise_partition_identical"]=true;
 for(int lights:{1,4,8}){p.lights=lights;p.first=0;for(int i=0;i<3;++i)launch(rows);
  std::vector<double> times;uint64_t lines=0;auto start=Clock::now();double elapsed=0;
  do{p.first=lines;auto t=Clock::now();launch(rows);times.push_back(std::chrono::duration<double>(Clock::now()-t).count());lines+=rows;elapsed=std::chrono::duration<double>(Clock::now()-start).count();}while(elapsed<duration);
  std::sort(times.begin(),times.end());double rate=lines/elapsed;
  report["cases"].push_back({{"light_samples",lights},{"noise",true},{"lines",lines},{"wall_seconds",elapsed},{"lines_per_wall_second",rate},{"batch_p99_s",times[size_t((times.size()-1)*.99)]},{"batch_max_s",times.back()},{"primary_ray_rate",rate*4096*3},{"nominal_shadow_ray_rate",rate*4096*3*lights},{"imaging_subtest_11khz",rate>=11000},{"imaging_subtest_22khz",rate>=22000}});
 }
 std::ofstream(out/"report.json")<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';
 CU(cudaStreamDestroy(stream));CU(cudaFreeHost(host));CU(cudaFree(p.image));CU(cudaFree((void*)dp));CU(cudaFree((void*)dr));OX(optixPipelineDestroy(pipeline));for(auto g:groups)OX(optixProgramGroupDestroy(g));OX(optixModuleDestroy(module));CU(cudaFree((void*)gas));CU(cudaFree((void*)dv));OX(optixDeviceContextDestroy(ctx));return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
