#include "agv_linescan/cuda_grid.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <stdexcept>

namespace agv_linescan {
namespace {
void Check(cudaError_t code) {
  if (code!=cudaSuccess) throw std::runtime_error(std::string("CUDA grid: ")+cudaGetErrorString(code));
}
__global__ void Scan(const float *rays, size_t width, const GridExposure *poses,
                     size_t rows, GridParameters c, uint8_t *image, uint32_t *invalid) {
  size_t u=blockIdx.x*blockDim.x+threadIdx.x, row=blockIdx.y;
  if (u>=width || row>=rows) return;
  float sum=0; bool valid=true;
  for (int i=0;i<3;++i) {
    const auto &p=poses[row].samples[i];
    float dz=p.down[2]+rays[u]*p.across[2];
    float t=-p.origin[2]/dz;
    float x=p.origin[0]+t*(p.down[0]+rays[u]*p.across[0]);
    float y=p.origin[1]+t*(p.down[1]+rays[u]*p.across[1]);
    bool hit=p.origin[2]>0 && t>0 && isfinite(x) && isfinite(y) &&
        fmaxf(fabsf(x),fabsf(y))<=c.extent*.5f;
    valid=valid && hit;
    float dx=fabsf(x-roundf(x/c.spacing)*c.spacing);
    float dy=fabsf(y-roundf(y/c.spacing)*c.spacing);
    if (hit) sum+=fminf(dx,dy)<=c.lineWidth*.5f ? c.dark : c.light;
  }
  image[row*width+u]=!valid?0:static_cast<uint8_t>(__float2int_rn(sum/3));
  if (!valid) atomicAdd(invalid+row,1u);
}
}
struct CudaGrid::Impl {
  float *rays=nullptr;
  GridExposure *poses=nullptr, *hostPoses=nullptr;
  uint8_t *pixels=nullptr, *hostPixels=nullptr;
  uint32_t *invalid=nullptr, *hostInvalid=nullptr;
  cudaStream_t stream=nullptr;
  size_t width=0,capacity=0;
  GridParameters parameters;
  std::string device;
  ~Impl() {
    if (stream) cudaStreamSynchronize(stream);
    cudaFree(rays); cudaFree(poses); cudaFree(pixels); cudaFree(invalid);
    cudaFreeHost(hostPoses); cudaFreeHost(hostPixels); cudaFreeHost(hostInvalid);
    if (stream) cudaStreamDestroy(stream);
  }
};
CudaGrid::CudaGrid(const std::vector<float> &rays,GridParameters c,size_t capacity):impl_(new Impl) {
  if (rays.size()<2 || rays.size()>65536 || !capacity || capacity>16384 ||
      !std::isfinite(c.spacing) || c.spacing<=0 || !std::isfinite(c.extent) || c.extent<=0 ||
      !std::isfinite(c.lineWidth) || c.lineWidth<=0 || c.lineWidth>=c.spacing)
    throw std::invalid_argument("invalid CUDA grid dimensions");
  for (float r:rays) if (!std::isfinite(r)) throw std::invalid_argument("invalid ray");
  auto &p=*impl_; p.width=rays.size(); p.capacity=capacity; p.parameters=c;
  int device; Check(cudaGetDevice(&device));
  cudaDeviceProp prop; Check(cudaGetDeviceProperties(&prop,device)); p.device=prop.name;
  Check(cudaStreamCreateWithFlags(&p.stream,cudaStreamNonBlocking));
  Check(cudaMalloc(&p.rays,p.width*sizeof(float)));
  Check(cudaMalloc(&p.poses,capacity*sizeof(GridExposure)));
  Check(cudaMalloc(&p.pixels,capacity*p.width));
  Check(cudaMalloc(&p.invalid,capacity*sizeof(uint32_t)));
  Check(cudaMallocHost(&p.hostPoses,capacity*sizeof(GridExposure)));
  Check(cudaMallocHost(&p.hostPixels,capacity*p.width));
  Check(cudaMallocHost(&p.hostInvalid,capacity*sizeof(uint32_t)));
  Check(cudaMemcpy(p.rays,rays.data(),p.width*sizeof(float),cudaMemcpyHostToDevice));
}
CudaGrid::~CudaGrid()=default;
const std::string &CudaGrid::Device() const {return impl_->device;}
GridBatch CudaGrid::Sample(const std::vector<GridExposure> &poses) {
  auto &p=*impl_; const size_t rows=poses.size();
  if (rows>p.capacity) throw std::invalid_argument("CUDA batch capacity exceeded");
  if (!rows) return {};
  for (const auto &e:poses) for (const auto &s:e.samples) for (int j=0;j<3;++j)
    if (!std::isfinite(s.origin[j]) || !std::isfinite(s.across[j]) || !std::isfinite(s.down[j]))
      throw std::invalid_argument("nonfinite CUDA pose");
  std::copy(poses.begin(),poses.end(),p.hostPoses);
  Check(cudaMemcpyAsync(p.poses,p.hostPoses,rows*sizeof(GridExposure),cudaMemcpyHostToDevice,p.stream));
  Check(cudaMemsetAsync(p.invalid,0,rows*sizeof(uint32_t),p.stream));
  Scan<<<dim3((p.width+255)/256,rows),256,0,p.stream>>>(p.rays,p.width,p.poses,rows,p.parameters,p.pixels,p.invalid);
  Check(cudaGetLastError());
  Check(cudaMemcpyAsync(p.hostPixels,p.pixels,rows*p.width,cudaMemcpyDeviceToHost,p.stream));
  Check(cudaMemcpyAsync(p.hostInvalid,p.invalid,rows*sizeof(uint32_t),cudaMemcpyDeviceToHost,p.stream));
  Check(cudaStreamSynchronize(p.stream));
  for(size_t r=0;r<rows;++r) if(p.hostInvalid[r]) throw std::runtime_error("invalid grid exposure: entire batch rejected");
  return {{p.hostPixels,p.hostPixels+rows*p.width},{p.hostInvalid,p.hostInvalid+rows}};
}
}  // namespace agv_linescan
