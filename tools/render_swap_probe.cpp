// Optional, process-local profiling. Counts actual GLX swaps per drawable.
#include <GL/glx.h>
#include <dlfcn.h>
#include <time.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <mutex>
extern "C" void glXSwapBuffers(Display* display,GLXDrawable drawable){
 using Swap=void(*)(Display*,GLXDrawable);static auto real=reinterpret_cast<Swap>(dlsym(RTLD_NEXT,"glXSwapBuffers"));
 if(!real)std::abort();real(display,drawable);
 const char* dir=std::getenv("AGV_RENDER_PROBE_DIR");if(!dir)return;
 static std::mutex lock;std::lock_guard<std::mutex> guard(lock);
 static FILE* file=nullptr;
 if(!file){char path[4096];std::snprintf(path,sizeof(path),"%s/swaps_%d.csv",dir,int(getpid()));file=std::fopen(path,"w");}
 if(file){timespec t;clock_gettime(CLOCK_MONOTONIC,&t);std::fprintf(file,"%.9f,%lu\n",double(t.tv_sec)+t.tv_nsec*1e-9,drawable);std::fflush(file);}
}
