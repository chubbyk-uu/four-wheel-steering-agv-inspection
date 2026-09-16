#pragma once
#include <cmath>
#include <optional>
namespace agv_linescan {
struct CameraEncoderInterval {
  double lines=0, encoder=0, travel=0, ratio=0;
};
// Camera-centre horizontal travel against nominal encoder distance over one
// pose-tag interval, the same quantities tools/audit_scan_slip.py divides so
// the online alert and the offline audit cannot disagree about what one is.
// An interval that cannot be judged returns nothing rather than a ratio of
// zero, which would read as a total stall.
inline std::optional<CameraEncoderInterval> MeasureCameraEncoder(
    double firstLine, double lastLine, double spacing, double fromX, double toX) {
  const double lines = lastLine - firstLine;
  if (!(lines > 0) || !(spacing > 0) || !std::isfinite(fromX) || !std::isfinite(toX))
    return std::nullopt;
  const double encoder = lines*spacing, travel = std::abs(toX - fromX);
  if (!(encoder > 0) || !std::isfinite(travel)) return std::nullopt;
  return CameraEncoderInterval{lines, encoder, travel, travel/encoder};
}
}
