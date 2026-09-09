#include "shared.h"
extern "C" { __constant__ ScanParams params; }
extern "C" __global__ void __miss__primary(){optixSetPayload_0(0xffffffffu);}
extern "C" __global__ void __miss__shadow(){optixSetPayload_0(0);}
extern "C" __global__ void __closesthit__primary(){
 optixSetPayload_0(optixGetPrimitiveIndex());optixSetPayload_1(__float_as_uint(optixGetRayTmax()));
}
__device__ float3 vector(const float* m,float3 v){return make_float3(m[0]*v.x+m[1]*v.y+m[2]*v.z,m[4]*v.x+m[5]*v.y+m[6]*v.z,m[8]*v.x+m[9]*v.y+m[10]*v.z);}
__device__ float3 point(const float* m,float3 v){return add(vector(m,v),make_float3(m[3],m[7],m[11]));}
__device__ float3 worldNormal(const float* m,float3 n){return make_float3(m[0]*n.x+m[4]*n.y+m[8]*n.z,m[1]*n.x+m[5]*n.y+m[9]*n.z,m[2]*n.x+m[6]*n.y+m[10]*n.z);}
__device__ bool bounds(const RayGeometry& g,float3 o,float3 d,float limit){
 float near=0,far=limit;float a[3]={o.x,o.y,o.z},b[3]={d.x,d.y,d.z};float lo[3]={g.lo.x,g.lo.y,g.lo.z},hi[3]={g.hi.x,g.hi.y,g.hi.z};
 for(int i=0;i<3;++i){if(fabsf(b[i])<1e-12f){if(a[i]<lo[i]||a[i]>hi[i])return false;}
 else {float x=(lo[i]-a[i])/b[i],y=(hi[i]-a[i])/b[i];near=fmaxf(near,fminf(x,y));far=fminf(far,fmaxf(x,y));if(near>far)return false;}}
 return true;
}
// Separate static GAS per moving link; transform rays at each exposure, no batch-frozen IAS.
__device__ bool trace(float3 o,float3 d,float limit,unsigned exposure,bool shadow,
                      unsigned& geometry,unsigned& primitive,float& distance){
 bool hit=false;distance=limit;
 for(unsigned g=0;g<=params.links;++g){
  float3 ro=o,rd=d;
  if(g){const float* m=reinterpret_cast<const float*>(params.transforms+exposure*params.links+g-1);ro=point(m,o);rd=vector(m,d);}
  if(!bounds(params.geometry[g],ro,rd,distance))continue;
  unsigned id=shadow?1:0xffffffffu,t=0;
  unsigned flags=OPTIX_RAY_FLAG_DISABLE_ANYHIT;
  if(shadow)flags|=OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT|OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT;
  optixTrace(params.geometry[g].handle,ro,rd,1e-5f,distance,0.f,255,flags,0,1,shadow?1:0,id,t);
  if(shadow){if(id)return true;}
  else if(id!=0xffffffffu){hit=true;geometry=g;primitive=id;distance=__uint_as_float(t);}
 }
 return hit;
}
// Dielectric Lambert + isotropic GGX / correlated Smith / Schlick, returned as pi * BRDF.
// F0=.04 and perceptual roughness squared; no multi-bounce/IBL model.
__device__ float materialBRDF(float3 n,float3 l,float3 v,float albedo,float roughness){
 float nl=fmaxf(0,dot(n,l)),nv=fmaxf(0,dot(n,v));if(nl<=0||nv<=0)return 0;
 auto h=unit(add(l,v));float nh=fmaxf(0,dot(n,h)),vh=fmaxf(0,dot(v,h));
 float a=roughness*roughness,a2=a*a,den=nh*nh*(a2-1)+1;
 float D=a2/(3.14159265359f*den*den);
 float V=.5f/fmaxf(1e-8f,nl*sqrtf(nv*nv*(1-a2)+a2)+nv*sqrtf(nl*nl*(1-a2)+a2));
 float f=1-vh,f2=f*f,F=.04f+.96f*f2*f2*f;
 return (1-F)*albedo+3.14159265359f*D*V*F;
}
extern "C" __global__ void __raygen__scan(){
 auto idx=optixGetLaunchIndex();unsigned u=idx.x,r=idx.y;auto sensor=*params.sensor;
 float sum=0;bool valid=true;float q=(float(u)-(params.width-1)*.5f)/(params.width*.5f);
 for(unsigned k=0;k<3;++k){
  auto pose=params.poses[r].samples[k];auto v=[](const float* a){return make_float3(a[0],a[1],a[2]);};
  float3 o=v(pose.origin),across=v(pose.across),down=v(pose.down),forward=cross(down,across);
  float3 d=unit(add(down,mul(across,params.rays[u])));unsigned g=0,id=0;float t;
  if(!trace(o,d,100,r*3+k,false,g,id,t)){valid=false;continue;}
  auto p=add(o,mul(d,t));auto vertices=params.geometry[g].vertices+id*3;
  auto n=unit(cross(sub(vertices[1],vertices[0]),sub(vertices[2],vertices[0])));
  if(g)n=worldNormal(reinterpret_cast<const float*>(params.transforms+(r*3+k)*params.links+g-1),n);
  if(dot(n,d)>0)n=mul(n,-1);
  bool material=!g && params.geometry[0].reflectance[id]==-2.f;
  float albedo=0,roughness=params.roughness;float3 shadingNormal=n;
  if(material){
   float textureU=(p.x-params.textureOriginX)/params.textureSpanX,textureV=(p.y-params.textureOriginY)/params.textureSpanY;
   if(textureU<0||textureU>1||textureV<0||textureV>1){valid=false;continue;} // Never silently clamp an uncovered primary ray.
   auto color=params.colorTexture,normal=params.normalTexture;
   if(params.tileMap){
    float x=(p.x-params.textureOriginX)/params.tileTexel,y=(p.y-params.textureOriginY)/params.tileTexel;
    int tx=int(floorf(x/params.tileCore)),ty=int(floorf(y/params.tileCore));
    if(tx<0||ty<0||tx>=params.tilesX||ty>=params.tilesY){valid=false;continue;}
    int slot=params.tileMap[ty*params.tilesX+tx];if(slot<0){valid=false;continue;}
    auto tile=params.materialTiles[slot];color=tile.color;normal=tile.normal;
    float stride=params.tileCore+2*params.tileGutter;
    textureU=(x-tx*params.tileCore+params.tileGutter)/stride;textureV=(y-ty*params.tileCore+params.tileGutter)/stride;
   }
   albedo=tex2D<float>(color,textureU,textureV);
   if(normal){
    auto xy=tex2D<float2>(normal,textureU,textureV);float nx=2*xy.x-1,ny=2*xy.y-1;
    auto tangent=make_float3(n.z,0,-n.x);
    if(dot(tangent,tangent)<1e-8f)tangent=make_float3(0,n.z,-n.y);
    tangent=unit(tangent);auto bitangent=cross(n,tangent);
    shadingNormal=unit(add(add(mul(tangent,nx),mul(bitangent,ny)),mul(n,sqrtf(fmaxf(0,1-nx*nx-ny*ny)))));
   }
   if(params.roughTexture)roughness=fminf(1,fmaxf(.1f,tex2D<float>(params.roughTexture,textureU,textureV)));
  }
  auto delta=sub(p,o);float pf=dot(delta,forward),pa=dot(delta,across),pd=dot(delta,down);
  float total=0,visible=0;
  // Midpoint quadrature on the actual fixture; count can be varied for convergence.
  float tilt=atan2f(sensor.ledForward,sensor.ledHeight);
  for(unsigned l=0;l<params.lampSamples;++l){
   float emittingLength=sensor.ledLength-.04f; // 20 mm unlit end caps, same as URDF exporter.
   float ly=emittingLength*((l+.5f)/params.lampSamples-.5f);
   auto lamp=add(o,add(mul(forward,sensor.ledForward-.022f*sinf(tilt)),
                 add(mul(across,ly),mul(down,sensor.cameraHeight-sensor.ledHeight+.022f*cosf(tilt)))));
   if(params.lampLink>=0){
    const float* m=reinterpret_cast<const float*>(params.transforms+(r*3+k)*params.links+params.lampLink);
    lamp=worldNormal(m,sub(params.emitters[l],make_float3(m[3],m[7],m[11])));
   }
   auto ld=sub(lamp,p);float dist=sqrtf(dot(ld,ld));auto direction=mul(ld,1/dist);
   float weight=fmaxf(0,dot(n,direction))/(dist*dist);
   total+=material?fmaxf(0,direction.z)/(dist*dist):weight;
   unsigned sg=0,si=0;float st;
   if(dist>3e-4f && (!material||dot(n,direction)>0) && !trace(add(p,mul(n,1e-4f)),direction,dist-2e-4f,r*3+k,true,sg,si,st))
    visible+=material?fmaxf(0,dot(shadingNormal,direction))/(dist*dist)*materialBRDF(shadingNormal,direction,mul(d,-1),albedo,roughness):weight;
  }
  float led=agv_linescan::LedIrradiance(sensor,pf,pa,pd,(material?1.f:fmaxf(0,-dot(n,down))))*(total>0?visible/total:0);
  if(material){sum+=agv_linescan::MeanElectrons(sensor,1,led,sensor.ambient*albedo,q,u);continue;}
  float reflectance;
  if(g)reflectance=params.geometry[g].reflectance[id];
  else if(params.geometry[0].reflectance[id]>=0)reflectance=params.geometry[0].reflectance[id];
  else {float x=p.x/.1f,y=p.y/.1f;reflectance=(fminf(fabsf(x-roundf(x)),fabsf(y-roundf(y)))<.01f?25.f:210.f)/255.f;}
  sum+=agv_linescan::MeanElectrons(sensor,reflectance,led,sensor.ambient,q,u);
 }
 if(!valid)atomicAdd(params.invalid+r,1u);
 params.pixels[r*params.width+u]=valid?agv_linescan::SensorCode(sensor,sum/3,params.first+r,u):0;
}
