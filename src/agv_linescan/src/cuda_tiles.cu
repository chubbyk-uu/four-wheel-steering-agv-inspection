#include "agv_linescan/cuda_tiles.hpp"
#include <cuda_runtime.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace agv_linescan {
namespace {
using Key=std::pair<int,int>;
using Clock=std::chrono::steady_clock;
void Check(cudaError_t e) {if(e!=cudaSuccess) throw std::runtime_error(std::string("CUDA tiles: ")+cudaGetErrorString(e));}
struct Tile {int x,y,slot;};
struct Domain {float texel,size,minX,maxX,minY,maxY; int baseX,baseY,pixels,stride;};
__device__ float Texel(const uint8_t* pool,int slot,int stride,int x,int y) {
  return pool[size_t(slot)*stride*stride+size_t(y)*stride+x];
}
__global__ void ScanTiles(const float* rays,size_t width,const GridExposure* poses,size_t rows,
                         Domain d,const Tile* tiles,int count,const uint8_t* pool,
                         uint8_t* output,uint32_t* invalid,Radiometry light,uint64_t firstLine,float shadowStart,float shadowEnd) {
  size_t u=blockIdx.x*blockDim.x+threadIdx.x,row=blockIdx.y;
  if(u>=width || row>=rows) return;
  float sum=0; bool valid=true;
  for(int k=0;k<3;++k) {
    const auto& p=poses[row].samples[k];
    float dz=p.down[2]+rays[u]*p.across[2],t=-p.origin[2]/dz;
    float x=p.origin[0]+t*(p.down[0]+rays[u]*p.across[0]);
    float y=p.origin[1]+t*(p.down[1]+rays[u]*p.across[1]);
    if(!(p.origin[2]>0 && t>0 && isfinite(x) && isfinite(y) &&
         x>=d.minX && x<d.maxX && y>=d.minY && y<d.maxY)) {valid=false; continue;}
    int lx=int(floorf(x/d.size)),ly=int(floorf(y/d.size)),slot=-1;
    for(int i=0;i<count;++i) if(tiles[i].x==d.baseX+lx && tiles[i].y==d.baseY+ly) {slot=tiles[i].slot;break;}
    if(slot<0) {valid=false;continue;}
    // Stored texel centers: (i - gutter + .5)*texel. One-pixel gutter
    // supplies both bilinear neighbors at a tile boundary.
    float tx=(x-lx*d.size)/d.texel+.5f,ty=(y-ly*d.size)/d.texel+.5f;
    tx=fminf(float(d.pixels)+.5f,fmaxf(.5f,tx)); ty=fminf(float(d.pixels)+.5f,fmaxf(.5f,ty));
    int ix=int(floorf(tx)),iy=int(floorf(ty)); float fx=tx-ix,fy=ty-iy;
    float a=Texel(pool,slot,d.stride,ix,iy)*(1-fx)+Texel(pool,slot,d.stride,ix+1,iy)*fx;
    float b=Texel(pool,slot,d.stride,ix,iy+1)*(1-fx)+Texel(pool,slot,d.stride,ix+1,iy+1)*fx;
    float reflectance=a*(1-fy)+b*fy;
    if(light.enabled) {
      float dx=x-p.origin[0],dy=y-p.origin[1],dzGround=-p.origin[2];
      // forward = down cross across for the camera basis used by GZ.
      float fw[3]={p.down[1]*p.across[2]-p.down[2]*p.across[1],
        p.down[2]*p.across[0]-p.down[0]*p.across[2],p.down[0]*p.across[1]-p.down[1]*p.across[0]};
      float forward=dx*fw[0]+dy*fw[1]+dzGround*fw[2];
      float across=dx*p.across[0]+dy*p.across[1]+dzGround*p.across[2];
      float depth=dx*p.down[0]+dy*p.down[1]+dzGround*p.down[2];
      float led=LedIrradiance(light,forward,across,depth,-p.down[2]);
      float ambient=light.ambient*((x>=shadowStart && x<shadowEnd)?light.shadowTransmission:1.f);
      float q=(float(u)-float(width-1)*.5f)/(float(width)*.5f);
      sum+=MeanElectrons(light,reflectance/255.f,led,ambient,q,u);
    } else sum+=reflectance;
  }
  output[row*width+u]=!valid?0:light.enabled?SensorCode(light,sum/3,firstLine+row,u):uint8_t(__float2int_rn(sum/3));
  if(!valid) atomicAdd(invalid+row,1u);
}
}
struct CudaTiles::Impl {
  struct Slot {Key key={-1,-1}; int state=0,pins=0;}; // 0 empty, 1 upload in progress, 2 ready
  Radiometry light;
  std::vector<Slot> slots;
  std::vector<Key> wanted;
  std::vector<float> hostRays;
  std::filesystem::path root;
  double ox=0,oy=0,length=0,groundWidth=0,texel=0,size=0;
  int pixels=0,nx=0,ny=0,device=0;
  size_t width=0,capacity=0,tileBytes=0,loads=0,misses=0,bytes=0;
  double maxLoad=0,totalLoad=0;
  std::vector<double> loadTimes;
  std::string deviceName;
  uint8_t *pool=nullptr,*staging=nullptr,*output=nullptr,*hostOutput=nullptr;
  float* rays=nullptr;
  GridExposure *poses=nullptr,*hostPoses=nullptr;
  uint32_t *invalid=nullptr,*hostInvalid=nullptr;
  Tile* descriptors=nullptr;
  cudaStream_t stream=nullptr,upload=nullptr;
  mutable std::mutex mutex;
  std::condition_variable wake,changed;
  std::thread worker;
  bool stop=false;
  std::exception_ptr error;
  ~Impl() {
    {std::lock_guard<std::mutex> lock(mutex);stop=true;} wake.notify_all();
    if(worker.joinable()) worker.join();
    if(stream) cudaStreamSynchronize(stream);
    if(upload) cudaStreamSynchronize(upload);
    cudaFree(pool);cudaFree(output);cudaFree(rays);cudaFree(poses);cudaFree(invalid);cudaFree(descriptors);
    cudaFreeHost(staging);cudaFreeHost(hostOutput);cudaFreeHost(hostPoses);cudaFreeHost(hostInvalid);
    if(stream) cudaStreamDestroy(stream);
    if(upload) cudaStreamDestroy(upload);
  }
  std::array<int,2> Base(std::array<double,2> a) const {
    int x=int(std::llround((a[0]-ox)/size)),y=int(std::llround((a[1]-oy)/size));
    if(std::abs(a[0]-(ox+x*size))>1e-7 || std::abs(a[1]-(oy+y*size))>1e-7)
      throw std::invalid_argument("tile anchor must lie on the metric tile grid");
    return {x,y};
  }
  std::array<double,4> Bounds(const std::vector<GridExposure>& samples,std::array<double,2> a) const {
    if(samples.empty() || samples.size()>capacity) throw std::invalid_argument("invalid tile batch size");
    Base(a);
    std::array<double,4> b={INFINITY,-INFINITY,INFINITY,-INFINITY};
    for(const auto& e:samples) for(const auto& p:e.samples) {
      for(int j=0;j<3;++j) if(!std::isfinite(p.origin[j]) || !std::isfinite(p.across[j]) || !std::isfinite(p.down[j]))
        throw std::invalid_argument("nonfinite tile sampling pose");
      for(float r:{hostRays.front(),hostRays.back()}) {
        double dz=p.down[2]+double(r)*p.across[2];
        if(dz>=-.01 || p.origin[2]<=0) throw std::invalid_argument("tile prefetch requires downward, horizon-free rays");
        double t=-p.origin[2]/dz;
        double x=a[0]+p.origin[0]+t*(p.down[0]+double(r)*p.across[0]);
        double y=a[1]+p.origin[1]+t*(p.down[1]+double(r)*p.across[1]);
        b[0]=std::min(b[0],x);b[1]=std::max(b[1],x);b[2]=std::min(b[2],y);b[3]=std::max(b[3],y);
      }
    }
    return b;
  }
  std::vector<Key> Keys(std::array<double,4> b) const {
    // Tiny conservative margin prevents float rounding at an exact boundary
    // from selecting a tile omitted by the double-precision host footprint.
    int x0=std::max(0,int(std::floor((b[0]-ox-1e-5)/size)));
    int x1=std::min(nx-1,int(std::floor((b[1]-ox+1e-5)/size)));
    int y0=std::max(0,int(std::floor((b[2]-oy-1e-5)/size)));
    int y1=std::min(ny-1,int(std::floor((b[3]-oy+1e-5)/size)));
    std::vector<Key> keys;
    for(int x=x0;x<=x1;++x) for(int y=y0;y<=y1;++y) keys.emplace_back(x,y);
    return keys;
  }
  int Find(Key key) const {
    for(size_t i=0;i<slots.size();++i) if(slots[i].state && slots[i].key==key) return int(i);
    return -1;
  }
  bool Ready(const std::vector<Key>& keys) const {
    for(auto k:keys) {int i=Find(k);if(i<0 || slots[i].state!=2) return false;}
    return true;
  }
  void Worker() {
    try {
      Check(cudaSetDevice(device));
      while(true) {
        int target=-1; Key key;
        {
          std::unique_lock<std::mutex> lock(mutex);
          wake.wait(lock,[&] {
            if(stop) return true;
            for(auto k:wanted) if(Find(k)<0) {
              for(size_t i=0;i<slots.size();++i) {
                const auto& s=slots[i];
                if(s.state==0 || (s.state==2 && !s.pins && std::find(wanted.begin(),wanted.end(),s.key)==wanted.end())) {
                  target=int(i);key=k;return true;
                }
              }
            }
            return false;
          });
          if(stop) break;
          slots[target]={key,1,0};
        }
        auto begin=Clock::now();
        auto file=root/("tile_"+std::to_string(key.first)+"_"+std::to_string(key.second)+".pgm");
        std::ifstream f(file,std::ios::binary); std::string magic; int w=0,h=0,max=0;
        f>>magic>>w>>h>>max;
        if(!f || magic!="P5" || w!=pixels+2 || h!=pixels+2 || max!=255 || f.get()!='\n')
          throw std::runtime_error("invalid tile: "+file.string());
        f.read(reinterpret_cast<char*>(staging),tileBytes);
        if(size_t(f.gcount())!=tileBytes || f.peek()!=std::char_traits<char>::eof())
          throw std::runtime_error("truncated or oversized tile: "+file.string());
        Check(cudaMemcpyAsync(pool+size_t(target)*tileBytes,staging,tileBytes,cudaMemcpyHostToDevice,upload));
        Check(cudaStreamSynchronize(upload));
        double dt=std::chrono::duration<double>(Clock::now()-begin).count();
        {
          std::lock_guard<std::mutex> lock(mutex);
          slots[target].state=2;++loads;bytes+=tileBytes;maxLoad=std::max(maxLoad,dt);totalLoad+=dt;
          if(loadTimes.size()<4096) loadTimes.push_back(dt);
        }
        changed.notify_all();
      }
    } catch(...) {std::lock_guard<std::mutex> lock(mutex);error=std::current_exception();changed.notify_all();}
  }
};
CudaTiles::CudaTiles(const std::vector<float>& rays,const std::string& manifest,size_t capacity,size_t slots,Radiometry light):impl_(new Impl) {
  auto& p=*impl_;light.Validate();p.light=light;
  if(rays.size()<2 || !capacity || capacity>16384 || slots<4 || slots>64) throw std::invalid_argument("invalid tile sampler capacity");
  for(size_t i=0;i<rays.size();++i) if(!std::isfinite(rays[i]) || (i && rays[i]<=rays[i-1])) throw std::invalid_argument("tile rays must increase");
  std::ifstream f(manifest);nlohmann::json c;f>>c;
  if(c.at("schema")!="agv.terrain.tiles.v1" || c.at("gutter_pixels")!=1 || c.at("encoding")!="mono8" ||
      c.at("layout")!="row_y_column_x" || c.at("plane_z_m")!=0) throw std::invalid_argument("unsupported tile manifest");
  p.root=std::filesystem::absolute(manifest).parent_path();p.hostRays=rays;
  p.ox=c.at("origin_x_m");p.oy=c.at("origin_y_m");p.length=c.at("length_m");p.groundWidth=c.at("width_m");
  p.texel=c.at("texel_m");p.pixels=c.at("core_pixels");p.nx=c.at("tiles_x");p.ny=c.at("tiles_y");
  if(!std::isfinite(p.ox) || !std::isfinite(p.oy) || !std::isfinite(p.length) || !std::isfinite(p.groundWidth) ||
      !std::isfinite(p.texel) || p.texel<=0 || p.pixels<32 || p.pixels>8192 || p.length<=0 || p.groundWidth<=0)
    throw std::invalid_argument("invalid terrain dimensions");
  p.size=p.texel*p.pixels;
  if(p.nx!=int(std::ceil(p.length/p.size)) || p.ny!=int(std::ceil(p.groundWidth/p.size))) throw std::invalid_argument("inconsistent terrain tile counts");
  p.width=rays.size();p.capacity=capacity;p.tileBytes=size_t(p.pixels+2)*(p.pixels+2);p.slots.resize(slots);
  Check(cudaGetDevice(&p.device));cudaDeviceProp prop;Check(cudaGetDeviceProperties(&prop,p.device));p.deviceName=prop.name;
  int least,greatest;Check(cudaDeviceGetStreamPriorityRange(&least,&greatest));
  Check(cudaStreamCreateWithPriority(&p.stream,cudaStreamNonBlocking,greatest));
  Check(cudaStreamCreateWithPriority(&p.upload,cudaStreamNonBlocking,least));
  Check(cudaMalloc(&p.pool,slots*p.tileBytes));Check(cudaMallocHost(&p.staging,p.tileBytes));
  Check(cudaMalloc(&p.rays,p.width*sizeof(float)));Check(cudaMemcpy(p.rays,rays.data(),p.width*sizeof(float),cudaMemcpyHostToDevice));
  Check(cudaMalloc(&p.poses,capacity*sizeof(GridExposure)));Check(cudaMallocHost(&p.hostPoses,capacity*sizeof(GridExposure)));
  Check(cudaMalloc(&p.output,capacity*p.width));Check(cudaMallocHost(&p.hostOutput,capacity*p.width));
  Check(cudaMalloc(&p.invalid,capacity*sizeof(uint32_t)));Check(cudaMallocHost(&p.hostInvalid,capacity*sizeof(uint32_t)));
  Check(cudaMalloc(&p.descriptors,slots*sizeof(Tile)));
  p.worker=std::thread([&p]{p.Worker();});
}
CudaTiles::~CudaTiles()=default;
std::array<double,2> CudaTiles::Anchor(double x,double y) const {
  const auto& p=*impl_;
  return {p.ox+std::floor((x-p.ox)/p.size)*p.size,p.oy+std::floor((y-p.oy)/p.size)*p.size};
}
void CudaTiles::Prefetch(const std::vector<GridExposure>& poses,std::array<double,2> a,std::array<double,2> ahead) {
  auto& p=*impl_;auto b=p.Bounds(poses,a);auto keys=p.Keys(b);
  for(int j=0;j<2;++j) {if(!std::isfinite(ahead[j])) throw std::invalid_argument("invalid prefetch vector");
    b[2*j]+=std::min(0.,ahead[j]);b[2*j+1]+=std::max(0.,ahead[j]);}
  auto future=p.Keys(b);
  // Required keys first; remaining tiles ordered nearest the current footprint.
  const double cx=a[0]+poses[0].samples[1].origin[0],cy=a[1]+poses[0].samples[1].origin[1];
  std::sort(future.begin(),future.end(),[&](Key l,Key r) {
    auto d=[&](Key k){return std::hypot(p.ox+(k.first+.5)*p.size-cx,p.oy+(k.second+.5)*p.size-cy);};return d(l)<d(r);});
  for(auto k:future) if(std::find(keys.begin(),keys.end(),k)==keys.end()) keys.push_back(k);
  if(keys.size()>p.slots.size()) throw std::runtime_error("prefetch footprint exceeds bounded tile cache");
  {std::lock_guard<std::mutex> lock(p.mutex);if(p.error) std::rethrow_exception(p.error);p.wanted=std::move(keys);}
  p.wake.notify_one();
}
void CudaTiles::Warm(const std::vector<GridExposure>& poses,std::array<double,2> a) {
  auto& p=*impl_;p.Bounds(poses,a);
  std::unique_lock<std::mutex> lock(p.mutex);
  // Warm the entire prefetch neighborhood before a new segment starts.
  if(!p.changed.wait_for(lock,std::chrono::seconds(30),[&]{return p.error || p.Ready(p.wanted);})) throw std::runtime_error("terrain warmup timeout");
  if(p.error) std::rethrow_exception(p.error);
}
GridBatch CudaTiles::Sample(const std::vector<GridExposure>& poses,std::array<double,2> a,uint64_t firstGlobalLine) {
  auto& p=*impl_;auto keys=p.Keys(p.Bounds(poses,a));auto base=p.Base(a);
  std::vector<Tile> tiles;
  {
    std::lock_guard<std::mutex> lock(p.mutex);if(p.error) std::rethrow_exception(p.error);
    if(!p.Ready(keys)) {++p.misses;throw std::runtime_error("required terrain tile is not ready; continuous scan failed");}
    for(auto k:keys) {int i=p.Find(k);++p.slots[i].pins;tiles.push_back({k.first,k.second,i});}
  }
  auto release=[&] {std::lock_guard<std::mutex> lock(p.mutex);for(auto t:tiles) --p.slots[t.slot].pins;p.wake.notify_one();};
  try {
    size_t n=poses.size();std::copy(poses.begin(),poses.end(),p.hostPoses);
    Check(cudaMemcpyAsync(p.poses,p.hostPoses,n*sizeof(GridExposure),cudaMemcpyHostToDevice,p.stream));
    if(!tiles.empty()) Check(cudaMemcpyAsync(p.descriptors,tiles.data(),tiles.size()*sizeof(Tile),cudaMemcpyHostToDevice,p.stream));
    Check(cudaMemsetAsync(p.invalid,0,n*sizeof(uint32_t),p.stream));
    Domain d={float(p.texel),float(p.size),float(p.ox-a[0]),float(p.ox+p.length-a[0]),float(p.oy-a[1]),
      float(p.oy+p.groundWidth-a[1]),base[0],base[1],p.pixels,p.pixels+2};
    ScanTiles<<<dim3((p.width+255)/256,n),256,0,p.stream>>>(p.rays,p.width,p.poses,n,d,p.descriptors,tiles.size(),p.pool,p.output,p.invalid,p.light,firstGlobalLine,float(double(p.light.shadowStart)-a[0]),float(double(p.light.shadowEnd)-a[0]));
    Check(cudaGetLastError());
    Check(cudaMemcpyAsync(p.hostOutput,p.output,n*p.width,cudaMemcpyDeviceToHost,p.stream));
    Check(cudaMemcpyAsync(p.hostInvalid,p.invalid,n*sizeof(uint32_t),cudaMemcpyDeviceToHost,p.stream));
    Check(cudaStreamSynchronize(p.stream));
    for(size_t r=0;r<n;++r) if(p.hostInvalid[r]) throw std::runtime_error("invalid terrain exposure: batch rejected at global line "+std::to_string(firstGlobalLine+r));
    release();return {{p.hostOutput,p.hostOutput+n*p.width},{p.hostInvalid,p.hostInvalid+n}};
  } catch(...) {cudaStreamSynchronize(p.stream);release();throw;}
}
std::string CudaTiles::Statistics() const {
  const auto& p=*impl_;std::lock_guard<std::mutex> lock(p.mutex);auto times=p.loadTimes;std::sort(times.begin(),times.end());
  return nlohmann::json{{"radiometry_enabled",p.light.enabled},{"radiometry_model","relative_projected_strip_v1"},{"device",p.deviceName},{"tile_loads",p.loads},{"uploaded_bytes",p.bytes},{"required_tile_misses",p.misses},
    {"cache_slots",p.slots.size()},{"gpu_tile_bytes",p.slots.size()*p.tileBytes},{"host_staging_bytes",p.tileBytes},
    {"load_seconds_sum",p.totalLoad},{"load_seconds_max",p.maxLoad},{"load_seconds_p99_first_4096",times.empty()?0:times[size_t((times.size()-1)*.99)]},
    {"sampling_interpolation","bilinear, one-texel gutters; three exposure samples"}}.dump();
}
}
