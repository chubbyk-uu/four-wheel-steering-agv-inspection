// Standalone diagnostic: hold GPU memory so another process can be asked to
// create a context while the device is under pressure. Allocates 64 MiB
// textures until the requested budget is reached, reports what stuck, then
// waits for a line on stdin before releasing.
// Build: g++ -O2 tools/probe_gl_hog.cpp -o /tmp/agv_gl_hog -lGL -lX11
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <cstdio>
#include <cstdlib>
#include <vector>

int main(int argc,char**argv){
  const int wanted=argc>1?std::atoi(argv[1]):16;      // 64 MiB units
  auto display=XOpenDisplay(nullptr);if(!display){std::printf("FAIL display\n");return 2;}
  int attrs[]={GLX_X_RENDERABLE,True,GLX_DRAWABLE_TYPE,GLX_PBUFFER_BIT,
               GLX_RENDER_TYPE,GLX_RGBA_BIT,None},count=0;
  auto configs=glXChooseFBConfig(display,DefaultScreen(display),attrs,&count);
  if(!configs||!count){std::printf("FAIL fbconfig\n");return 3;}
  auto context=glXCreateNewContext(display,configs[0],GLX_RGBA_TYPE,nullptr,True);
  int surfaceAttrs[]={GLX_PBUFFER_WIDTH,32,GLX_PBUFFER_HEIGHT,32,None};
  auto surface=glXCreatePbuffer(display,configs[0],surfaceAttrs);
  if(!context||!glXMakeContextCurrent(display,surface,surface,context)){std::printf("FAIL context\n");return 4;}
  std::printf("renderer=%s\n",glGetString(GL_RENDERER));
  std::vector<GLuint> textures;std::vector<unsigned char> pixels(4096ull*4096*4,0x5a);
  int held=0;
  for(int i=0;i<wanted;++i){
    GLuint t=0;glGenTextures(1,&t);glBindTexture(GL_TEXTURE_2D,t);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,4096,4096,0,GL_RGBA,GL_UNSIGNED_BYTE,pixels.data());
    GLenum e=glGetError();
    if(e!=GL_NO_ERROR){glDeleteTextures(1,&t);std::printf("stopped at %d units, gl error 0x%x\n",i,e);break;}
    glFinish();textures.push_back(t);held=i+1;
  }
  std::printf("held_mib=%d units=%d\n",held*64,held);fflush(stdout);
  char line[64];if(!std::fgets(line,sizeof(line),stdin)){}
  glDeleteTextures((GLsizei)textures.size(),textures.data());glFinish();
  std::printf("released\n");
  return 0;
}
