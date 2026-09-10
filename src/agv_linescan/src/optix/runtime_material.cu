#include "runtime_material.h"
#include "verified_tile_file.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
using Json=nlohmann::json;
namespace agv_linescan {
namespace {
void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
struct Inputs {unsigned char *color,*normal,*alpha;float *pigment,*strength,*lut;int4* patches;float4* cracks;int w,h,patch,fw,fh,ncracks;double ratio,length;};
__device__ unsigned char interp8(const unsigned char* p,int w,int h,int channels,int c,float x,float y){
 int u=__float2int_rn(x*32),v=__float2int_rn(y*32);int x0=u>>5,y0=v>>5,fx=u&31,fy=v&31;
 int sum=0;
 for(int j=0;j<2;++j)for(int i=0;i<2;++i){int xx=max(0,min(w-1,x0+i)),yy=max(0,min(h-1,y0+j));sum+=int(p[(yy*w+xx)*channels+c])*(i?fx:32-fx)*(j?fy:32-fy);}
 return (sum+512)>>10;
}
__device__ float interpField(const float* p,int w,int h,float x,float y,bool flipx,bool flipy){
 int u=__float2int_rn(x*32),v=__float2int_rn(y*32),x0=u>>5,y0=v>>5,fx=u&31,fy=v&31;
 float sum=0;
 for(int j=0;j<2;++j)for(int i=0;i<2;++i){int xx=x0+i,yy=y0+j;float value=1;
  if(xx>=0&&xx<w&&yy>=0&&yy<h)value=p[(flipy?h-1-yy:yy)*w+(flipx?w-1-xx:xx)];
  sum+=value*float((i?fx:32-fx)*(j?fy:32-fy))/1024.f;
 }return sum;
}
__device__ float3 unit(float3 n){float length=sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);return make_float3(n.x/length,n.y/length,n.z/length);}
__device__ unsigned char quant(float x){return (unsigned char)fminf(255.f,fmaxf(0.f,x*255.f+.5f));}
__global__ void Generate(Inputs in,const double* coordinates,const int* ids,int count,int stride,unsigned char* output,unsigned* invalid){
 int col=blockIdx.x*blockDim.x+threadIdx.x,row=blockIdx.y*blockDim.y+threadIdx.y;if(col>=stride||row>=stride)return;
 double gx=coordinates[col],gy=coordinates[stride+row],wx=coordinates[2*stride+col],wy=coordinates[3*stride+row];
 float3 rgb=make_float3(0,0,0),normal=rgb;float filled=0;
 for(int k=0;k<count;++k){int id=ids[k];int4 p=in.patches[id];double px=gx-p.x,py=gy-p.y;
  if(px<0||py<0||px>=in.patch||py>=in.patch)continue;
  float a=interp8(in.alpha+size_t(id)*in.patch*in.patch,in.patch,in.patch,1,0,float(px-.5),float(py-.5))/255.f;
  filled=filled*(1-a)+a;
  float u=float((px+p.z)*in.ratio-.5),v=float((py+p.w)*in.ratio-.5);
  float* c=&rgb.x;float* n=&normal.x;
  for(int channel=0;channel<3;++channel){
   c[channel]=c[channel]*(1-a)+in.lut[interp8(in.color,in.w,in.h,3,channel,u,v)]*a;
   n[channel]=n[channel]*(1-a)+(float(interp8(in.normal,in.w,in.h,3,channel,u,v))/255.f)*a;
  }
 }
 if(filled<.99999f){atomicExch(invalid,1u);return;}
 bool mark=false;
 if(wx>=0&&wx<=in.length){
  if(fabs(fabs(wy)-.15)<=.075){rgb=make_float3(.80f+.06f*rgb.x,.56f+.06f*rgb.y,.015f+.06f*rgb.z);mark=true;}
  if(fabs(fabs(wy)-4.5)<=.075){rgb=make_float3(.72f+.12f*rgb.x,.72f+.12f*rgb.y,.72f+.12f*rgb.z);mark=true;}
 }
 normal=unit(make_float3(normal.x*2-1,normal.y*2-1,normal.z*2-1));normal.y=-normal.y;
 if(mark){normal.x*=.25f;normal.y*=.25f;}normal=unit(normal);
 for(int k=0;k<in.ncracks;++k){float4 p=in.cracks[k];double ox=double(p.x)-in.fw*.00025/2,oy=double(p.y)-in.fh*.00025/2;
  if(wx<ox-.00025||wx>ox+in.fw*.00025+.00025||wy<oy-.00025||wy>oy+in.fh*.00025+.00025)continue;
  float x=float((wx-ox)/.00025-.5),y=float((wy-oy)/.00025-.5);
  float pigment=interpField(in.pigment,in.fw,in.fh,x,y,p.z!=0,p.w!=0),strength=interpField(in.strength,in.fw,in.fh,x,y,p.z!=0,p.w!=0);
  rgb.x*=pigment;rgb.y*=pigment;rgb.z*=pigment;normal.x*=strength;normal.y*=strength;
 }
 normal=unit(normal);size_t index=size_t(row)*stride+col,plane=size_t(stride)*stride;
 output[index]=quant((rgb.x*.2126f+rgb.y*.7152f)+rgb.z*.0722f);
 output[plane+2*index]=quant(normal.x*.5f+.5f);output[plane+2*index+1]=quant(normal.y*.5f+.5f);
}
}
struct RuntimeMaterial::Impl {
 Inputs inputs={};Json hashes;VerifiedTileReader reader;std::vector<void*> buffers;size_t bytes=0;int core,gutter,stride,nx,ny;double ox,oy,texel,qx,qy,gsd;
 std::vector<int4> patches;double* coordinates=nullptr;int* ids=nullptr;unsigned char* output=nullptr;unsigned* invalid=nullptr;
 ~Impl(){for(void* p:buffers)cudaFree(p);}
 void* Allocate(size_t n,const void* src=nullptr){void* ptr=nullptr;Check(cudaMalloc(&ptr,n));buffers.push_back(ptr);bytes+=n;if(src)Check(cudaMemcpy(ptr,src,n,cudaMemcpyHostToDevice));return ptr;}
 void* Load(const std::filesystem::path& root,const Json& e,size_t n){
  auto name=e.get<std::string>();if(std::filesystem::path(name).has_parent_path())throw std::runtime_error("recipe payload path must be local");
  std::ifstream f(root/name,std::ios::binary|std::ios::ate);if(!f||size_t(f.tellg())!=n)throw std::runtime_error("recipe payload size mismatch");
  std::vector<unsigned char> data(n);reader.Read(root/name,hashes.at(name).get<std::string>(),data.data(),n);return Allocate(n,data.data());
 }
 void GenerateTile(int ix,int iy,cudaStream_t stream){
  if(ix<0||iy<0||ix>=nx||iy>=ny)throw std::runtime_error("recipe tile outside bounds");
  std::vector<double> xy(4*stride);std::vector<int> selected;
  for(int i=0;i<stride;++i){double x=ox+(ix*core+i-gutter+.5)*texel,y=oy+(iy*core+i-gutter+.5)*texel;xy[i]=(x-qx)/gsd;xy[stride+i]=(y-qy)/gsd;xy[2*stride+i]=x;xy[3*stride+i]=y;}
  for(size_t i=0;i<patches.size();++i){auto p=patches[i];if(xy[0]<p.x+inputs.patch&&xy[stride-1]>=p.x&&xy[stride]<p.y+inputs.patch&&xy[2*stride-1]>=p.y)selected.push_back(i);}
  if(selected.empty()||selected.size()>16)throw std::runtime_error("invalid recipe tile patch coverage");
  Check(cudaMemcpyAsync(coordinates,xy.data(),xy.size()*sizeof(double),cudaMemcpyHostToDevice,stream));
  Check(cudaMemcpyAsync(ids,selected.data(),selected.size()*sizeof(int),cudaMemcpyHostToDevice,stream));
  Check(cudaMemsetAsync(invalid,0,sizeof(unsigned),stream));
  Generate<<<dim3((stride+15)/16,(stride+15)/16),dim3(16,16),0,stream>>>(inputs,coordinates,ids,selected.size(),stride,output,invalid);
  Check(cudaGetLastError());unsigned missing=0;Check(cudaMemcpyAsync(&missing,invalid,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));Check(cudaStreamSynchronize(stream));if(missing)throw std::runtime_error("runtime recipe contains unfilled texels");
 }
};
RuntimeMaterial::RuntimeMaterial(const std::filesystem::path& path,const std::string& digest):impl_(std::make_unique<Impl>()){
 auto& s=*impl_;std::ifstream f(path,std::ios::binary|std::ios::ate);if(!f||f.tellg()<0||f.tellg()>16*1024*1024)throw std::runtime_error("invalid recipe manifest");
 std::vector<unsigned char> content(size_t(f.tellg()));f.seekg(0);f.read((char*)content.data(),content.size());if(!digest.empty())s.reader.Read(path,digest,content.data(),content.size());Json m=Json::parse(content);s.hashes=m.at("payload_sha256");if(m.at("schema")!="agv.material.recipe.probe.v1")throw std::runtime_error("unsupported recipe");auto root=path.parent_path();
 auto& in=s.inputs;in.w=m.at("source_width");in.h=m.at("source_height");in.patch=m.at("patch");in.ratio=m.at("ratio");in.length=m.at("length");in.fw=m.at("field_width");in.fh=m.at("field_height");
 s.core=m.at("core");s.gutter=m.at("gutter");s.nx=m.at("nx");s.ny=m.at("ny");s.ox=m.at("ox");s.oy=m.at("oy");s.texel=m.at("texel");s.qx=m.at("qx");s.qy=m.at("qy");s.gsd=m.at("gsd");s.stride=s.core+2*s.gutter;
 if(in.w<1||in.w>16384||in.h<1||in.h>16384||in.patch<1||in.patch>1024||s.core<2||s.core>4096||s.gutter<1||s.gutter>8||in.fw<1||in.fh<1||in.fw>16384||in.fh>16384)throw std::runtime_error("invalid recipe dimensions");
 for(double v:{in.ratio,in.length,s.ox,s.oy,s.texel,s.qx,s.qy,s.gsd})if(!std::isfinite(v))throw std::runtime_error("nonfinite recipe geometry");
 if(s.nx<1||s.ny<1||s.nx>4096||s.ny>4096||s.texel<=0||s.gsd<=0||in.length<=0||in.ratio<=0)throw std::runtime_error("invalid recipe geometry");
 for(auto p:m.at("placements"))s.patches.push_back(make_int4(p[0],p[1],p[2],p[3]));
 if(s.patches.empty()||s.patches.size()>10000)throw std::runtime_error("invalid recipe patch count");
 in.patches=(int4*)s.Allocate(s.patches.size()*sizeof(int4),s.patches.data());
 in.color=(unsigned char*)s.Load(root,m.at("color"),size_t(in.w)*in.h*3);in.normal=(unsigned char*)s.Load(root,m.at("normal"),size_t(in.w)*in.h*3);
 in.alpha=(unsigned char*)s.Load(root,m.at("alpha"),s.patches.size()*in.patch*in.patch);
 in.pigment=(float*)s.Load(root,m.at("pigment"),size_t(in.fw)*in.fh*4);in.strength=(float*)s.Load(root,m.at("strength"),size_t(in.fw)*in.fh*4);in.lut=(float*)s.Load(root,m.at("lut"),256*4);
 std::vector<float4> cracks;for(auto p:m.at("cracks"))cracks.push_back(make_float4(p[0],p[1],p[2],p[3]));in.ncracks=cracks.size();if(!cracks.empty())in.cracks=(float4*)s.Allocate(cracks.size()*sizeof(float4),cracks.data());
 s.coordinates=(double*)s.Allocate(4*s.stride*sizeof(double));s.ids=(int*)s.Allocate(16*sizeof(int));s.output=(unsigned char*)s.Allocate(size_t(s.stride)*s.stride*3);s.invalid=(unsigned*)s.Allocate(sizeof(unsigned));
}
void RuntimeMaterial::CheckLayout(const Json& m)const{auto& s=*impl_;
 if(m.at("tiles_x")!=s.nx||m.at("tiles_y")!=s.ny||m.at("core_pixels")!=s.core||m.at("gutter_pixels")!=s.gutter||m.at("texel_m")!=s.texel||m.at("origin_xy_m").at(0)!=s.ox||m.at("origin_xy_m").at(1)!=s.oy)throw std::runtime_error("recipe/cache grid mismatch");
}
RuntimeMaterial::~RuntimeMaterial()=default;
size_t RuntimeMaterial::Bytes()const{return impl_->bytes;}
void RuntimeMaterial::Bake(int ix,int iy,cudaArray_t color,cudaArray_t normal,cudaStream_t stream){auto& s=*impl_;s.GenerateTile(ix,iy,stream);size_t plane=size_t(s.stride)*s.stride;
 Check(cudaMemcpy2DToArrayAsync(color,0,0,s.output,s.stride,s.stride,s.stride,cudaMemcpyDeviceToDevice,stream));Check(cudaMemcpy2DToArrayAsync(normal,0,0,s.output+plane,s.stride*2,s.stride*2,s.stride,cudaMemcpyDeviceToDevice,stream));Check(cudaStreamSynchronize(stream));}
void RuntimeMaterial::BakeHost(int ix,int iy,unsigned char* data){auto& s=*impl_;s.GenerateTile(ix,iy,nullptr);Check(cudaMemcpy(data,s.output,size_t(s.stride)*s.stride*3,cudaMemcpyDeviceToHost));}
}
namespace {thread_local std::string error;}
extern "C" const char* recipe_error(){return error.c_str();}
extern "C" void* recipe_create(const char* p){try{return new agv_linescan::RuntimeMaterial(p);}catch(const std::exception& e){error=e.what();return nullptr;}}
extern "C" int recipe_bake(void* p,int x,int y,unsigned char* out){try{static_cast<agv_linescan::RuntimeMaterial*>(p)->BakeHost(x,y,out);return 0;}catch(const std::exception& e){error=e.what();return -1;}}
extern "C" size_t recipe_bytes(void* p){return static_cast<agv_linescan::RuntimeMaterial*>(p)->Bytes();}
extern "C" void recipe_destroy(void* p){delete static_cast<agv_linescan::RuntimeMaterial*>(p);}
