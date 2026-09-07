#pragma once
#include <optix.h>
#include <cuda_runtime.h>
#include <cmath>
#include <cstdint>
#include "agv_linescan/radiometry.hpp"
#ifdef __CUDACC__
#define HD __host__ __device__
#else
#define HD
#endif
struct QueryRay {float3 origin,direction;float limit;};
struct QueryHit {float distance;unsigned primitive;};
struct Params {
 OptixTraversableHandle handle;
 float3* vertices;
 unsigned char* image;
 uint64_t first;
 int lights, shadows;
 bool noise;
 QueryRay* queryRays;
 QueryHit* queryHits;
 unsigned queryCount;
};
HD inline float3 add(float3 a,float3 b){return make_float3(a.x+b.x,a.y+b.y,a.z+b.z);}
HD inline float3 sub(float3 a,float3 b){return make_float3(a.x-b.x,a.y-b.y,a.z-b.z);}
HD inline float3 mul(float3 a,float s){return make_float3(a.x*s,a.y*s,a.z*s);}
HD inline float dot(float3 a,float3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
HD inline float3 cross(float3 a,float3 b){return make_float3(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);}
HD inline float3 unit(float3 a){return mul(a,1/sqrtf(dot(a,a)));}
HD inline float qcol(int u){return (u-2047.5f)/2048.f;}
HD inline float3 origin(uint64_t line,int k){
 // Explicit 96 m repeating trajectory; the reset is a benchmark discontinuity.
 float x=2+float(line%327680)*.00029296875f+(k-1)*5.55555556f*1e-5f;
 return make_float3(x,0,.8370535714f);
}
HD inline float3 direction(int u){float q=qcol(u);return make_float3(0,.7168f*(q+.04f*q*q*q),-1);}
HD inline float reflectance(float3 p){
 float dx=fabsf(p.x/.1f-roundf(p.x/.1f))*.1f;
 float dy=fabsf(p.y/.1f-roundf(p.y/.1f))*.1f;
 return fminf(dx,dy)<.001f?25.f/255:210.f/255;
}
HD inline float3 lamp(float3 camera,int i,int n){
 return make_float3(camera.x+.25f,n==1?0:-.7f+1.4f*(i+.5f)/n,camera.z-.5370535714f);
}
HD inline float lightWeight(float3 camera,float3 p,float3 normal,float3 light){
 auto delta=sub(light,p);float d2=dot(delta,delta);
 float cosine=fmaxf(0,dot(normal,unit(delta)));
 // Relative finite strip fixture with narrow longitudinal beam; not measured photometry.
 float longitudinal=(p.x-camera.x)/.08f;
 return cosine/fmaxf(d2,.0001f)*expf(-2*powf(longitudinal,4));
}
#undef HD
