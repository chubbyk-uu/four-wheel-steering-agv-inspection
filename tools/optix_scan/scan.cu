#include "shared.h"
extern "C" { __constant__ Params params; }
extern "C" __global__ void __miss__primary(){optixSetPayload_0(0xffffffffu);}
extern "C" __global__ void __miss__shadow(){optixSetPayload_0(0);}
extern "C" __global__ void __closesthit__primary(){
 optixSetPayload_0(optixGetPrimitiveIndex());optixSetPayload_1(__float_as_uint(optixGetRayTmax()));
}
extern "C" __global__ void __raygen__scan(){
 if(params.queryCount){
  unsigned i=optixGetLaunchIndex().x;if(i>=params.queryCount)return;
  auto r=params.queryRays[i];unsigned id=0xffffffffu,t=0;
  optixTrace(params.handle,r.origin,r.direction,0.f,r.limit,0.f,255,OPTIX_RAY_FLAG_DISABLE_ANYHIT,0,1,0,id,t);
  params.queryHits[i]={id==0xffffffffu?-1.f:__uint_as_float(t),id};return;
 }
 agv_linescan::Radiometry sensor; sensor.noise=params.noise;
 auto idx=optixGetLaunchIndex();auto line=params.first+idx.y;float sum=0;bool valid=true;
 for(int k=0;k<3;++k){
  float3 o=origin(line,k),d=direction(idx.x);unsigned id=0xffffffffu,t=0;
  optixTrace(params.handle,o,d,0.f,100.f,0.f,255,OPTIX_RAY_FLAG_DISABLE_ANYHIT,0,1,0,id,t);
  if(id==0xffffffffu){valid=false;continue;}
  float3 p=add(o,mul(d,__uint_as_float(t)));
  auto v=params.vertices+id*3;float3 normal=unit(cross(sub(v[1],v[0]),sub(v[2],v[0])));
  if(dot(normal,d)>0)normal=mul(normal,-1);
  float total=0,reference=0;
  for(int l=0;l<params.lights;++l){
   auto light=lamp(o,l,params.lights);auto delta=sub(light,p);float dist=sqrtf(dot(delta,delta));
   unsigned blocked=1,unused=0;
   if(params.shadows)optixTrace(params.handle,add(p,mul(normal,1e-4f)),unit(delta),1e-4f,dist-2e-4f,0.f,255,
    OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT|OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT|OPTIX_RAY_FLAG_DISABLE_ANYHIT,0,1,1,blocked,unused);
   else blocked=0;
   total+=(1-blocked)*lightWeight(o,p,normal,light);
   reference+=lightWeight(o,make_float3(o.x,0,0),make_float3(0,0,1),light);
  }
  float led=40*total/reference;
  sum+=agv_linescan::MeanElectrons(sensor,reflectance(p),led,1,qcol(idx.x),idx.x);
 }
 params.image[idx.y*4096+idx.x]=valid?agv_linescan::SensorCode(sensor,sum/3,line,idx.x):0;
}
