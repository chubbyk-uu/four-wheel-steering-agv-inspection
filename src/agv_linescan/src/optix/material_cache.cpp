#include "material_cache.h"
#include <openssl/sha.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
using Json=nlohmann::json;
namespace agv_linescan {
namespace {
using Clock=std::chrono::steady_clock;
void Check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(std::string("material cache: ")+cudaGetErrorString(e));}
double Seconds(Clock::duration d){return std::chrono::duration<double>(d).count();}
}
struct MaterialCache::Impl {
 struct Slot {int key=-1,state=0,pins=0;};
 std::filesystem::path root;std::vector<Json> entries;std::vector<Slot> slots;
 std::vector<cudaArray_t> arrays;std::vector<cudaTextureObject_t> objects;
 MaterialTile* deviceTiles=nullptr;int* deviceMap=nullptr;unsigned char* staging=nullptr;
 cudaStream_t upload=nullptr,sampling=nullptr;int device=0,nx=0,ny=0,core=0,gutter=0,stride=0;
 double ox=0,oy=0,texel=0,size=0,zmin=0,zmax=0,ahead=0,behind=0,waitTimeout=.05;
 float rmin=0,rmax=0;std::vector<int> wanted,pinned;
 mutable std::mutex mutex;std::condition_variable changed,wake;std::thread worker;
 bool stop=false,warmed=false;std::exception_ptr error;
 size_t loads=0,bytes=0,misses=0,evictions=0;double maxLoad=0,totalLoad=0,maxWait=0,warmSeconds=0;
 ~Impl(){
  {std::lock_guard<std::mutex> lock(mutex);stop=true;}wake.notify_all();
  if(worker.joinable())worker.join();
  if(upload)cudaStreamSynchronize(upload);
  for(auto t:objects)cudaDestroyTextureObject(t);for(auto a:arrays)cudaFreeArray(a);
  cudaFree(deviceTiles);cudaFree(deviceMap);cudaFreeHost(staging);if(upload)cudaStreamDestroy(upload);
 }
 int Find(int key)const{for(size_t i=0;i<slots.size();++i)if(slots[i].state && slots[i].key==key)return i;return -1;}
 bool Ready(const std::vector<int>& keys)const{for(auto k:keys){int i=Find(k);if(i<0||slots[i].state!=2)return false;}return true;}
 std::vector<int> Keys(std::array<double,4> b)const{
  int x0=std::max(0,int(std::floor((b[0]-ox-1e-5)/size))),x1=std::min(nx-1,int(std::floor((b[1]-ox+1e-5)/size)));
  int y0=std::max(0,int(std::floor((b[2]-oy-1e-5)/size))),y1=std::min(ny-1,int(std::floor((b[3]-oy+1e-5)/size)));
  std::vector<int> keys;for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)keys.push_back(y*nx+x);return keys;
 }
 std::array<double,4> Bounds(const std::vector<GridExposure>& poses)const{
  std::array<double,4> b={INFINITY,-INFINITY,INFINITY,-INFINITY};
  for(const auto& e:poses)for(const auto& p:e.samples)for(float ray:{rmin,rmax})for(double z:{zmin,zmax}){
   double dz=p.down[2]+ray*p.across[2];
   if(dz>=-.01||p.origin[2]<=zmax)throw std::runtime_error("material footprint requires downward rays above declared height bounds");
   double t=(z-p.origin[2])/dz,x=p.origin[0]+t*(p.down[0]+ray*p.across[0]),y=p.origin[1]+t*(p.down[1]+ray*p.across[1]);
   b[0]=std::min(b[0],x);b[1]=std::max(b[1],x);b[2]=std::min(b[2],y);b[3]=std::max(b[3],y);
  }return b;
 }
 void Read(const Json& e,unsigned char* data,size_t count){
  auto name=e.at("file").get<std::string>();auto file=root/name;
  if(std::filesystem::path(name).is_absolute()||std::filesystem::path(name).has_parent_path())throw std::runtime_error("tile path must be a local filename");
  std::ifstream f(file,std::ios::binary);f.read((char*)data,count);
  if(size_t(f.gcount())!=count||f.peek()!=std::char_traits<char>::eof())throw std::runtime_error("tile missing/truncated: "+name);
  unsigned char hash[32];SHA256(data,count,hash);std::ostringstream s;for(auto v:hash)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);
  if(s.str()!=e.at("sha256"))throw std::runtime_error("tile checksum mismatch: "+name);
 }
 void Worker(){try{
  Check(cudaSetDevice(device));size_t plane=size_t(stride)*stride;
  while(true){int slot=-1,key=-1;
   {std::unique_lock<std::mutex> lock(mutex);
    wake.wait(lock,[&]{if(stop)return true;
     for(int k:wanted)if(Find(k)<0)for(size_t i=0;i<slots.size();++i)
      if(!slots[i].state||(slots[i].state==2&&!slots[i].pins&&std::find(wanted.begin(),wanted.end(),slots[i].key)==wanted.end())){slot=i;key=k;return true;}
     return false;});
    if(stop)return;if(slots[slot].state)++evictions;slots[slot]={key,1,0};
   }
   auto start=Clock::now();Read(entries[key].at("color"),staging,plane);Read(entries[key].at("normal"),staging+plane,plane*2);
   Check(cudaMemcpy2DToArrayAsync(arrays[2*slot],0,0,staging,stride,stride,stride,cudaMemcpyHostToDevice,upload));
   Check(cudaMemcpy2DToArrayAsync(arrays[2*slot+1],0,0,staging+plane,stride*2,stride*2,stride,cudaMemcpyHostToDevice,upload));
   Check(cudaStreamSynchronize(upload)); // Ready only after both complete; sampling uses another stream.
   double elapsed=Seconds(Clock::now()-start);
   {std::lock_guard<std::mutex> lock(mutex);slots[slot].state=2;++loads;bytes+=plane*3;maxLoad=std::max(maxLoad,elapsed);totalLoad+=elapsed;}
   changed.notify_all();
  }
 }catch(...){std::lock_guard<std::mutex> lock(mutex);error=std::current_exception();changed.notify_all();}}
};
MaterialCache::MaterialCache(const Json& m,const std::filesystem::path& root,const std::vector<float>& rays):impl_(std::make_unique<Impl>()){
 auto& s=*impl_;s.root=root;s.nx=m.at("tiles_x");s.ny=m.at("tiles_y");s.core=m.at("core_pixels");s.gutter=m.at("gutter_pixels");
 s.texel=m.at("texel_m");s.ox=m.at("origin_xy_m").at(0);s.oy=m.at("origin_xy_m").at(1);
 s.zmin=m.at("height_bounds_m").at(0);s.zmax=m.at("height_bounds_m").at(1);s.ahead=m.value("prefetch_ahead_m",1.5);s.behind=m.value("prefetch_behind_m",.5);s.waitTimeout=m.value("required_wait_timeout_s",.05);
 int count=m.value("cache_slots",32);
 for(double v:{s.texel,s.ox,s.oy,s.zmin,s.zmax,s.ahead,s.behind,s.waitTimeout})if(!std::isfinite(v))throw std::runtime_error("nonfinite tiled material");
 if(s.nx<1||s.ny<1||s.nx>4096||s.ny>4096||size_t(s.nx)*s.ny>1000000||s.core<2||s.core>4096||s.gutter<1||s.gutter>8||s.texel<=0||s.zmin>s.zmax||s.ahead<0||s.behind<0||s.waitTimeout<=0||s.waitTimeout>1||count<2||count>128)throw std::runtime_error("invalid tiled material budget");
 s.stride=s.core+2*s.gutter;s.size=s.core*s.texel;s.slots.resize(count);s.pinned.reserve(count);s.entries.resize(s.nx*s.ny);
 for(const auto& t:m.at("tiles")){int x=t.at("ix"),y=t.at("iy");if(x<0||x>=s.nx||y<0||y>=s.ny||!s.entries[y*s.nx+x].is_null())throw std::runtime_error("duplicate/outside material tile");s.entries[y*s.nx+x]=t;}
 for(const auto& t:s.entries)if(t.is_null())throw std::runtime_error("missing tile manifest entry");
 auto limits=std::minmax_element(rays.begin(),rays.end());s.rmin=*limits.first;s.rmax=*limits.second;
 Check(cudaGetDevice(&s.device));Check(cudaStreamCreateWithFlags(&s.upload,cudaStreamNonBlocking));
 Check(cudaMallocHost((void**)&s.staging,size_t(s.stride)*s.stride*3));
 std::vector<MaterialTile> tiles;
 for(int i=0;i<count;++i){MaterialTile t={};
  for(int c=0;c<2;++c){cudaArray_t array=nullptr;auto format=c?cudaCreateChannelDesc<uchar2>():cudaCreateChannelDesc<unsigned char>();
   Check(cudaMallocArray(&array,&format,s.stride,s.stride));s.arrays.push_back(array);
   cudaResourceDesc resource={};resource.resType=cudaResourceTypeArray;resource.res.array.array=array;
   cudaTextureDesc desc={};desc.normalizedCoords=1;desc.filterMode=cudaFilterModeLinear;desc.readMode=cudaReadModeNormalizedFloat;desc.addressMode[0]=desc.addressMode[1]=cudaAddressModeClamp;
   cudaTextureObject_t object=0;Check(cudaCreateTextureObject(&object,&resource,&desc,nullptr));s.objects.push_back(object);(c?t.normal:t.color)=object;
  }tiles.push_back(t);
 }
 Check(cudaMalloc((void**)&s.deviceTiles,tiles.size()*sizeof(MaterialTile)));Check(cudaMemcpy(s.deviceTiles,tiles.data(),tiles.size()*sizeof(MaterialTile),cudaMemcpyHostToDevice));
 Check(cudaMalloc((void**)&s.deviceMap,size_t(s.nx)*s.ny*sizeof(int)));s.worker=std::thread([&s]{s.Worker();});
}
MaterialCache::~MaterialCache()=default;
void MaterialCache::Bind(ScanParams& p){auto& s=*impl_;p.tileMap=s.deviceMap;p.materialTiles=s.deviceTiles;p.tilesX=s.nx;p.tilesY=s.ny;p.tileCore=s.core;p.tileGutter=s.gutter;p.tileTexel=s.texel;p.textureOriginX=s.ox;p.textureOriginY=s.oy;p.textureSpanX=s.nx*s.size;p.textureSpanY=s.ny*s.size;}
void MaterialCache::Begin(const std::vector<GridExposure>& poses,cudaStream_t stream){
 auto& s=*impl_;s.sampling=stream;auto b=s.Bounds(poses);auto required=s.Keys(b);
 if(required.empty()||required.size()>s.slots.size())throw std::runtime_error("material footprint outside domain or exceeds slot budget");
 double dx=poses.back().samples[1].origin[0]-poses.front().samples[1].origin[0],dy=poses.back().samples[1].origin[1]-poses.front().samples[1].origin[1];
 double norm=std::hypot(dx,dy);if(norm<1e-8){dx=1;dy=0;}else {dx/=norm;dy/=norm;}
 auto extended=b;for(int j=0;j<2;++j){double d=j?dy:dx;extended[2*j]+=std::min(-s.behind*d,s.ahead*d);extended[2*j+1]+=std::max(-s.behind*d,s.ahead*d);}
 auto extras=s.Keys(extended);double cx=((b[0]+b[1])*.5-s.ox)/s.size,cy=((b[2]+b[3])*.5-s.oy)/s.size;
 std::sort(extras.begin(),extras.end(),[&](int a,int b){auto distance=[&](int k){return std::hypot(k%s.nx+.5-cx,k/s.nx+.5-cy);};return distance(a)<distance(b);});
 std::vector<int> want=required;for(int k:extras)if(want.size()<s.slots.size()&&std::find(want.begin(),want.end(),k)==want.end())want.push_back(k);
 auto start=Clock::now();std::vector<int> mapping(s.nx*s.ny,-1);
 {std::unique_lock<std::mutex> lock(s.mutex);if(s.error)std::rethrow_exception(s.error);s.wanted=want;s.wake.notify_one();
  bool cold=!s.warmed;if(!cold&&!s.Ready(required))++s.misses;
  const auto& waitFor=cold?want:required;
  if(!s.changed.wait_for(lock,std::chrono::duration<double>(cold?10:s.waitTimeout),[&]{return s.error||s.Ready(waitFor);}))throw std::runtime_error("material prefetch timeout");
  if(s.error)std::rethrow_exception(s.error);
  for(int key:required){int slot=s.Find(key);++s.slots[slot].pins;s.pinned.push_back(slot);mapping[key]=slot;}
  double wait=Seconds(Clock::now()-start);if(cold)s.warmSeconds=wait;else s.maxWait=std::max(s.maxWait,wait);s.warmed=true;
 }
 Check(cudaMemcpyAsync(s.deviceMap,mapping.data(),mapping.size()*sizeof(int),cudaMemcpyHostToDevice,stream));
 // Pageable upload may stage synchronously; complete before the temporary mapping disappears.
 Check(cudaStreamSynchronize(stream));
}
void MaterialCache::End()noexcept{auto& s=*impl_;if(s.sampling)cudaStreamSynchronize(s.sampling);{std::lock_guard<std::mutex> lock(s.mutex);for(int i:s.pinned)--s.slots[i].pins;s.pinned.clear();}s.wake.notify_one();}
size_t MaterialCache::Bytes()const{const auto& s=*impl_;return s.slots.size()*(size_t(s.stride)*s.stride*3+sizeof(MaterialTile))+size_t(s.nx)*s.ny*sizeof(int);}
std::string MaterialCache::Statistics()const{auto& s=*impl_;std::lock_guard<std::mutex> lock(s.mutex);return Json{{"cache_slots",s.slots.size()},{"tile_loads",s.loads},{"evictions",s.evictions},{"required_tile_misses_after_warm",s.misses},{"bytes_read",s.bytes},{"load_seconds_max",s.maxLoad},{"load_seconds_total",s.totalLoad},{"batch_wait_seconds_max",s.maxWait},{"cold_warm_seconds",s.warmSeconds},{"device_payload_bytes",Bytes()},{"slot_pins",s.pinned.size()}}.dump();}
}
