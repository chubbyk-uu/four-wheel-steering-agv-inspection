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
