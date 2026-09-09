#pragma once
#include "agv_linescan/radiometry.hpp"
#include <yaml-cpp/yaml.h>
namespace agv_linescan {
inline Radiometry LoadRadiometry(const YAML::Node& config) {
  Radiometry c;
  c.exposure=config["exposure_s"].as<float>();
  c.cameraHeight=config["nominal_width_m"].as<float>()*config["focal_length_m"].as<float>()/
      (config["width"].as<float>()*config["pixel_pitch_m"].as<float>());
  c.ledHeight=config["led_height_m"].as<float>();c.ledForward=config["led_forward_offset_m"].as<float>();
  c.ledLength=config["led_length_m"].as<float>();
  const auto r=config["radiometry"];
  if(r) {
    c.enabled=r["enabled"].as<bool>(false);c.noise=r["noise"].as<bool>(true);
#define LOAD(field,key) c.field=r[key].as<float>(c.field)
    LOAD(ledPeak,"led_peak_relative");LOAD(halfWidth,"led_half_width_m");LOAD(halfDepth,"led_half_depth_m");
    LOAD(ambient,"ambient_relative");LOAD(shadowTransmission,"shadow_transmission");
    LOAD(shadowStart,"shadow_start_x_m");LOAD(shadowEnd,"shadow_end_x_m");
    LOAD(vignette,"vignette_edge_loss");LOAD(prnu,"prnu_fraction");LOAD(readNoise,"read_noise_electrons");
    LOAD(referenceIrradiance,"reference_irradiance_relative");
    LOAD(referenceElectrons,"reference_electrons");LOAD(fullWell,"full_well_electrons");
    LOAD(gain,"gain");LOAD(black,"black_level_dn");
#undef LOAD
    c.seed=r["seed"].as<uint32_t>(c.seed);
  }
  c.Validate();return c;
}
}
