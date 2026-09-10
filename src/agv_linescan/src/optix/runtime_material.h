#pragma once
#include <cuda_runtime.h>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
namespace agv_linescan {
// Experimental deterministic GPU replay of the archived ConcreteQuilt recipe.
class RuntimeMaterial {
 public:
  explicit RuntimeMaterial(const std::filesystem::path& recipe, const std::string& digest="");
  void CheckLayout(const nlohmann::json& material)const;
  ~RuntimeMaterial();
  void Bake(int ix,int iy,cudaArray_t color,cudaArray_t normal,cudaStream_t stream);
  void BakeHost(int ix,int iy,unsigned char* output);
  size_t Bytes()const;
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
