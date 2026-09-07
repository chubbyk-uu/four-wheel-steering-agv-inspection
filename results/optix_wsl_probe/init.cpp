#include <cstdio>
#include <cuda.h>
#include <optix.h>
#include <optix_stubs.h>
#include <optix_function_table_definition.h>
int main() {
 CUresult c=cuInit(0); printf("cuInit=%d\n",int(c));
 if(c!=CUDA_SUCCESS) return 1;
 OptixResult r=optixInit(); printf("optixInit=%d (%s)\n",int(r),optixGetErrorName(r));
 if(r!=OPTIX_SUCCESS) return 2;
 CUdevice d; CUcontext ctx; if(cuDeviceGet(&d,0)!=CUDA_SUCCESS || cuCtxCreate(&ctx,0,d)!=CUDA_SUCCESS) return 3;
 OptixDeviceContext context=nullptr; OptixDeviceContextOptions options={};
 r=optixDeviceContextCreate(ctx,&options,&context); printf("optixDeviceContextCreate=%d\n",int(r));
 if(context) optixDeviceContextDestroy(context); cuCtxDestroy(ctx); return r==OPTIX_SUCCESS?0:4;
}
