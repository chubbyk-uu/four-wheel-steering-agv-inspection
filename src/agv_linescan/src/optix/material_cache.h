#pragma once
#include "shared.h"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <memory>
namespace agv_linescan {
class MaterialCache {
 public:
  MaterialCache(const nlohmann::json&,const std::filesystem::path&,const std::vector<float>&);
  ~MaterialCache();
  void Bind(ScanParams&);
  void Begin(const std::vector<GridExposure>&,cudaStream_t,bool prewarm=false);
  void End() noexcept;
  std::string Statistics() const;
  size_t Bytes() const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
