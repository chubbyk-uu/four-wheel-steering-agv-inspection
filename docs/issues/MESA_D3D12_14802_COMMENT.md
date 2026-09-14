I found a likely root cause in the shared command-signature cache and reproduced signature-related memory growth with indirect **draws** on WSL2. I have not tested the original application's indirect-compute workload, but both paths use this cache.

In `d3d12_get_cmd_signature()`, `key` is already a `const d3d12_cmd_signature_key *`, but lookup passes `&key`:

```diff
-   struct hash_entry *entry = _mesa_hash_table_search(ctx->cmd_signature_cache, &key);
+   struct hash_entry *entry = _mesa_hash_table_search(ctx->cmd_signature_cache, key);
```

The hash/equality callbacks read the structure contents. Passing the pointer's address causes incorrect lookups. The miss path creates another command signature, then inserts the correct copied key. When insertion replaces an existing entry with that key, the previous wrapper and its COM reference are not released; cache destruction only sees the surviving entry.

The same lookup is present in [26.2.2](https://gitlab.freedesktop.org/mesa/mesa/-/blob/mesa-26.2.2/src/gallium/drivers/d3d12/d3d12_cmd_signature.cpp#L68) and [main at a655bee9ba8de9ddc1a0a4e861ee67c47d13cf55](https://gitlab.freedesktop.org/mesa/mesa/-/blob/a655bee9ba8de9ddc1a0a4e861ee67c47d13cf55/src/gallium/drivers/d3d12/d3d12_cmd_signature.cpp#L68), checked on 2026-09-14. This is source inspection only; I have not built or runtime-tested those newer versions.

Tested environment: Ubuntu 24.04 / WSL2, Mesa `25.2.8-0ubuntu0.24.04.2`, D3D12 renderer on NVIDIA RTX 5080, host driver 616.92.

The standalone reproducer below repeatedly draws one fixed triangle to a 32x32 GLX pbuffer. No ROS, Gazebo, CUDA or external assets are required. It checks GL errors and verifies foreground/background pixels. Comparing equal pre-readback draw counts:

| Mode | RSS at draw 2001 (KiB) | RSS at draw 10001 (KiB) |
| --- | ---: | ---: |
| Original, indirect | 173048 | 196648 |
| Original, direct | 164924 | 164924 |
| Privately corrected library, indirect | 187768 | 187768 |

Original indirect draws grow by approximately 2.95 KiB/draw; direct draws and corrected indirect draws stay flat. All rendering checks pass. The first final readback adds a one-time allocation, so it is excluded from these intervals.

For diagnosis only, the correction was made to a private copy of the installed binary at the single lookup-argument instruction (load the key pointer instead of taking its stack-slot address). It is the assembly equivalent of the diff above, **not a source-built patch validation**. The original system library was untouched; preloading the unmodified library still reproduced growth. In the original application's separate short stationary test, growth fell from about 2.38 MiB/s to approximately flat. A source-built fix and broader regression testing remain to be done.

Save the source below as `probe_mesa_indirect_memory.cpp` and run with an X display and OpenGL 4.3 available:

```sh
g++ -O2 -Wall -Wextra probe_mesa_indirect_memory.cpp -o /tmp/mesa_probe -lGL -lX11
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA /tmp/mesa_probe indirect 12000
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA /tmp/mesa_probe direct 12000
```

Adjust the adapter selector for other hardware. The reproducer uses Linux `/proc/self/statm` for RSS and GLX, so it is not directly portable to the original Windows setup.

<details>
<summary>Standalone reproducer source</summary>

```cpp
// Standalone diagnostic: no ROS, Gazebo, OptiX or project assets.
// Build: g++ -O2 tools/probe_mesa_indirect_memory.cpp -o /tmp/mesa_probe -lGL -lX11
// Run: GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA /tmp/mesa_probe indirect 24000
// Use "direct" for the same triangle without indirect commands.
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <unistd.h>

long rssKiB() {
  std::ifstream f("/proc/self/statm");long size=0,resident=0;f>>size>>resident;
  return resident*sysconf(_SC_PAGESIZE)/1024;
}

int main(int argc,char**argv) {
  const bool indirect=argc<2 || !std::strcmp(argv[1],"indirect");
  if(argc>2 && std::atoi(argv[2])<=0)return 1;
  if(argc>1 && !indirect && std::strcmp(argv[1],"direct"))return 1;
  const int draws=argc>2?std::atoi(argv[2]):24000;
  auto display=XOpenDisplay(nullptr);if(!display){std::fprintf(stderr,"X display unavailable\n");return 2;}
  int attrs[]={GLX_X_RENDERABLE,True,GLX_DRAWABLE_TYPE,GLX_PBUFFER_BIT,
               GLX_RENDER_TYPE,GLX_RGBA_BIT,None},count=0;
  auto configs=glXChooseFBConfig(display,DefaultScreen(display),attrs,&count);
  if(!configs || !count)return 3;
  auto create=(PFNGLXCREATECONTEXTATTRIBSARBPROC)glXGetProcAddressARB((const GLubyte*)"glXCreateContextAttribsARB");
  if(!create)return 3;
  int contextAttrs[]={GLX_CONTEXT_MAJOR_VERSION_ARB,4,GLX_CONTEXT_MINOR_VERSION_ARB,3,
                     GLX_CONTEXT_PROFILE_MASK_ARB,GLX_CONTEXT_CORE_PROFILE_BIT_ARB,None};
  auto context=create(display,configs[0],nullptr,True,contextAttrs);
  int surfaceAttrs[]={GLX_PBUFFER_WIDTH,32,GLX_PBUFFER_HEIGHT,32,None};
  auto surface=glXCreatePbuffer(display,configs[0],surfaceAttrs);
  if(!context || !glXMakeContextCurrent(display,surface,surface,context))return 4;
  std::printf("renderer=%s mode=%s draws=%d\n",glGetString(GL_RENDERER),indirect?"indirect":"direct",draws);
  glViewport(0,0,32,32);glClearColor(0,0,0,0);
  const char* vertex="#version 330\nvoid main(){vec2 p[3]=vec2[3](vec2(-.5,-.5),vec2(.5,-.5),vec2(0,.5));gl_Position=vec4(p[gl_VertexID],0,1);}";
  const char* fragment="#version 330\nout vec4 c;void main(){c=vec4(1);}";
  auto vs=glCreateShader(GL_VERTEX_SHADER);glShaderSource(vs,1,&vertex,nullptr);glCompileShader(vs);
  auto fs=glCreateShader(GL_FRAGMENT_SHADER);glShaderSource(fs,1,&fragment,nullptr);glCompileShader(fs);
  auto program=glCreateProgram();glAttachShader(program,vs);glAttachShader(program,fs);glLinkProgram(program);
  GLint linked=0;glGetProgramiv(program,GL_LINK_STATUS,&linked);if(!linked)return 5;
  glDeleteShader(vs);glDeleteShader(fs);glUseProgram(program);
  GLuint vao=0,buffer=0;glGenVertexArrays(1,&vao);glBindVertexArray(vao);
  unsigned args[4]={3,1,0,0};glGenBuffers(1,&buffer);glBindBuffer(GL_DRAW_INDIRECT_BUFFER,buffer);
  glBufferData(GL_DRAW_INDIRECT_BUFFER,sizeof(args),args,GL_STATIC_DRAW);
  for(int i=0;i<draws;++i) {
    glClear(GL_COLOR_BUFFER_BIT);
    if(indirect)glDrawArraysIndirect(GL_TRIANGLES,nullptr);else glDrawArrays(GL_TRIANGLES,0,3);
    glFinish();auto error=glGetError();if(error){std::fprintf(stderr,"GL error=%u\n",error);return 6;}
    if(i%2000==0){std::printf("draws=%d rss_kib=%ld\n",i+1,rssKiB());std::fflush(stdout);}
    usleep(1000);
  }
  unsigned char center[4]={},corner[4]={};
  glReadPixels(16,16,1,1,GL_RGBA,GL_UNSIGNED_BYTE,center);
  glReadPixels(0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,corner);
  bool rendered=center[0]==255 && center[1]==255 && center[2]==255 &&
                corner[0]==0 && corner[1]==0 && corner[2]==0 && glGetError()==GL_NO_ERROR;
  std::printf("end draws=%d rss_kib=%ld rendered=%s\n",draws,rssKiB(),rendered?"true":"false");
  glDeleteBuffers(1,&buffer);glDeleteVertexArrays(1,&vao);glDeleteProgram(program);
  glXMakeContextCurrent(display,None,None,nullptr);glXDestroyPbuffer(display,surface);
  glXDestroyContext(display,context);XFree(configs);XCloseDisplay(display);
  return rendered?0:7;
}
```

</details>
