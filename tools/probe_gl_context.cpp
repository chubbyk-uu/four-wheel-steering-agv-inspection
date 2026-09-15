// Standalone diagnostic: does creating a GLX context succeed right now?
// Mirrors the surface format the Gazebo GUI asks for (window, double buffered,
// 24-bit depth, 8-bit stencil), because that is the request seen failing in the
// GUI abort log, not the pbuffer path the memory probe uses.
// Build: g++ -O2 tools/probe_gl_context.cpp -o /tmp/agv_gl_context -lGL -lX11
// Exit: 0 created, 2 no display, 3 no fbconfig, 4 no context, 5 not current.
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <cstdio>
#include <ctime>

static double now(){timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}

int main(){
  const double t0=now();
  auto display=XOpenDisplay(nullptr);
  if(!display){std::printf("FAIL stage=display elapsed=%.3f\n",now()-t0);return 2;}
  int attrs[]={GLX_X_RENDERABLE,True,GLX_DRAWABLE_TYPE,GLX_WINDOW_BIT,
               GLX_RENDER_TYPE,GLX_RGBA_BIT,GLX_DOUBLEBUFFER,True,
               GLX_DEPTH_SIZE,24,GLX_STENCIL_SIZE,8,None},count=0;
  auto configs=glXChooseFBConfig(display,DefaultScreen(display),attrs,&count);
  if(!configs||!count){std::printf("FAIL stage=fbconfig elapsed=%.3f\n",now()-t0);return 3;}
  auto visual=glXGetVisualFromFBConfig(display,configs[0]);
  if(!visual){std::printf("FAIL stage=visual elapsed=%.3f\n",now()-t0);return 3;}
  XSetWindowAttributes swa{};
  swa.colormap=XCreateColormap(display,RootWindow(display,visual->screen),visual->visual,AllocNone);
  auto window=XCreateWindow(display,RootWindow(display,visual->screen),0,0,64,64,0,visual->depth,
                            InputOutput,visual->visual,CWColormap,&swa);
  auto context=glXCreateNewContext(display,configs[0],GLX_RGBA_TYPE,nullptr,True);
  if(!context){std::printf("FAIL stage=context elapsed=%.3f\n",now()-t0);return 4;}
  if(!glXMakeCurrent(display,window,context)){std::printf("FAIL stage=makecurrent elapsed=%.3f\n",now()-t0);return 5;}
  std::printf("OK renderer=%s version=%s direct=%d elapsed=%.3f\n",
              glGetString(GL_RENDERER),glGetString(GL_VERSION),
              glXIsDirect(display,context),now()-t0);
  glXMakeCurrent(display,None,nullptr);glXDestroyContext(display,context);
  XDestroyWindow(display,window);XCloseDisplay(display);
  return 0;
}
