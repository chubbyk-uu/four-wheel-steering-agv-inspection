#pragma once
#include <optix.h>
#include <cuda_runtime.h>
#include "agv_linescan/optix_scene.hpp"
struct RayGeometry {OptixTraversableHandle handle;float3* vertices;float* reflectance;float3 lo,hi;};
struct ScanParams {
 RayGeometry* geometry;unsigned links,width;
 const agv_linescan::GridExposure* poses;
 const agv_linescan::LinkTransform* transforms;
 const agv_linescan::Radiometry* sensor;
 const float* rays;unsigned char* pixels;unsigned* invalid;
 int lampLink;unsigned lampSamples;float3 emitters[256];
 uint64_t first;
};
#ifdef __CUDACC__
#define HD __host__ __device__
#else
#define HD
#endif
HD inline float3 add(float3 a,float3 b){return make_float3(a.x+b.x,a.y+b.y,a.z+b.z);}
HD inline float3 sub(float3 a,float3 b){return make_float3(a.x-b.x,a.y-b.y,a.z-b.z);}
HD inline float3 mul(float3 a,float s){return make_float3(a.x*s,a.y*s,a.z*s);}
HD inline float dot(float3 a,float3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
HD inline float3 cross(float3 a,float3 b){return make_float3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);}
HD inline float3 unit(float3 a){return mul(a,1/sqrtf(dot(a,a)));}
#undef HD
