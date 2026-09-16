#pragma once
#include <gz/math/Vector3.hh>
#include <optional>
namespace agv_linescan {
// Velocity of the wheel material at the contact point relative to a static road.
// Reject undefined normals; absence of evidence must not become zero slip.
inline std::optional<gz::math::Vector3d> ContactTangentVelocity(
    const gz::math::Vector3d &linear,const gz::math::Vector3d &angular,
    const gz::math::Vector3d &arm,gz::math::Vector3d normal) {
  if(!linear.IsFinite() || !angular.IsFinite() || !arm.IsFinite() || !normal.IsFinite() || normal.Length()==0)return std::nullopt;
  normal.Normalize();const auto material=linear+angular.Cross(arm);
  return material-normal*material.Dot(normal);
}
}
