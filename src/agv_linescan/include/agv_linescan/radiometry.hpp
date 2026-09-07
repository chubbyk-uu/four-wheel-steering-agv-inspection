#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#ifdef __CUDACC__
#define AGV_HD __host__ __device__
#else
#define AGV_HD
#endif
namespace agv_linescan {
// Relative, engineered strip-light footprint, not an absolute lux/ray-traced lamp.
// Texture codes are interpreted as linear diffuse reflectance (code / 255).
struct Radiometry {
  bool enabled=false, noise=true;
  float exposure=20e-6f, cameraHeight=.8370535714f;
  float ledHeight=.30f, ledForward=.25f, ledPeak=40;
  float halfWidth=.85f, halfDepth=.08f;
  float ambient=1, shadowTransmission=.1f;
  float shadowStart=40, shadowEnd=50;
  float vignette=.15f, prnu=.01f, readNoise=3;
  float referenceIrradiance=41; // fixed calibration reference; does not track current LED power
  float referenceElectrons=10000, fullWell=12000, gain=1, black=4;
  uint32_t seed=1729;
  void Validate() const {
    const float values[]={exposure,cameraHeight,ledHeight,ledForward,ledPeak,halfWidth,halfDepth,
      ambient,shadowTransmission,shadowStart,shadowEnd,vignette,prnu,readNoise,referenceIrradiance,referenceElectrons,fullWell,gain,black};
    for(float x:values) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite radiometry parameter");
    if(exposure<=0 || cameraHeight<=0 || ledHeight<=0 || halfWidth<=0 || halfDepth<=0 ||
       ledPeak<0 || ambient<0 || shadowTransmission<0 || shadowTransmission>1 || shadowEnd<shadowStart ||
       vignette<0 || vignette>=1 || prnu<0 || prnu>.2f || readNoise<0 || referenceIrradiance<=0 || referenceElectrons<=0 ||
       fullWell<=0 || gain<=0 || black<0 || black>=255)
      throw std::invalid_argument("invalid radiometry parameter");
  }
};
AGV_HD inline uint32_t RadiometryHash(uint32_t v) {
  v^=v>>16;v*=0x7feb352du;v^=v>>15;v*=0x846ca68bu;return v^(v>>16);
}
AGV_HD inline float Uniform(uint32_t v) {return ((RadiometryHash(v)>>8)+.5f)*(1.f/16777216.f);}
// Coordinates of a surface point relative to the camera in its forward/across/down axes.
// Perspective projection moves/broadens the finite light band as mounting height changes.
AGV_HD inline float LedIrradiance(const Radiometry& c,float forward,float across,float down,float normalCosine) {
  float depth=down-(c.cameraHeight-c.ledHeight);
  if(depth<=0 || normalCosine<=0) return 0;
  float scale=depth/c.ledHeight;
  float center=c.ledForward*(1-scale);
  float a=across/(c.halfWidth*scale),b=(forward-center)/(c.halfDepth*scale);
  float a2=a*a,a4=a2*a2,b2=b*b;
  return c.ledPeak*expf(-2*(a4*a4+b2*b2))*normalCosine/(scale*scale);
}
AGV_HD inline float MeanElectrons(const Radiometry& c,float reflectance,float led,float ambient,float q,uint32_t column) {
  float sensitivity=(1-c.vignette*q*q)*(1+c.prnu*(2*Uniform(column^c.seed)-1));
  return reflectance*(led+ambient)/c.referenceIrradiance*(c.exposure/20e-6f)*c.referenceElectrons*sensitivity;
}
AGV_HD inline uint8_t SensorCode(const Radiometry& c,float electrons,uint64_t line,uint32_t column) {
  // Shot noise uses a Gaussian approximation; read noise shares the summed variance.
  if(c.noise) {
    uint32_t key=RadiometryHash(uint32_t(line)^RadiometryHash(uint32_t(line>>32))^c.seed)^RadiometryHash(column+1);
    float normal=sqrtf(-2*logf(Uniform(key)))*cosf(6.28318530718f*Uniform(key^0x9e3779b9u));
    electrons+=sqrtf(fmaxf(0,electrons)+c.readNoise*c.readNoise)*normal;
  }
  electrons=fminf(c.fullWell,fmaxf(0,electrons));
  float code=c.black+(255-c.black)*c.gain*electrons/c.fullWell;
  return uint8_t(fminf(255,fmaxf(0,floorf(code+.5f))));
}
}
#undef AGV_HD
