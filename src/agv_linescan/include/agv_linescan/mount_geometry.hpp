#pragma once
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace agv_linescan {
struct MountPivot { double x; double z; };

inline MountPivot LoadMountPivot(const YAML::Node &config) {
  const auto mount=config["mount_flex"];
  if(!mount || !mount["pivot_x_m"] || !mount["pivot_z_m"])
    throw std::runtime_error("mount_flex pivot is required");
  MountPivot pivot{mount["pivot_x_m"].as<double>(),mount["pivot_z_m"].as<double>()};
  if(!std::isfinite(pivot.x) || !std::isfinite(pivot.z))
    throw std::runtime_error("mount_flex pivot must be finite");
  return pivot;
}
}
