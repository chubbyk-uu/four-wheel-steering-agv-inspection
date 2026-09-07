#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <iomanip>
inline nlohmann::json ReadJson(const std::filesystem::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("cannot open "+p.string());nlohmann::json j;f>>j;return j;}
inline std::string Sha256(const std::filesystem::path& p){
 std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("missing mesh");
 std::string bytes((std::istreambuf_iterator<char>(f)),{});unsigned char h[SHA256_DIGEST_LENGTH];SHA256((const unsigned char*)bytes.data(),bytes.size(),h);
 std::ostringstream o;for(auto c:h)o<<std::hex<<std::setfill('0')<<std::setw(2)<<int(c);return o.str();
}
// Independent strict OBJ reader: positive indices, triangulated faces, identity transform.
inline std::vector<float3> LoadScene(const std::filesystem::path& path){
 auto m=ReadJson(path);if(m.at("schema")!="agv.shared.static_scene.v1"||m.at("units")!="m"||m.at("frame")!="world"||m.at("transform")!="identity_world_baked")throw std::runtime_error("unsupported scene");
 std::vector<float3> result;
 for(const auto&a:m.at("assets")){
  auto file=path.parent_path()/a.at("mesh").get<std::string>();if(Sha256(file)!=a.at("sha256"))throw std::runtime_error("mesh checksum mismatch");
  std::ifstream f(file);std::string line;std::vector<float3> vertices;size_t count=0;
  while(std::getline(f,line)){
   std::istringstream s(line);std::string kind;s>>kind;
   if(kind=="v"){float3 v;if(!(s>>v.x>>v.y>>v.z)||!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.z))throw std::runtime_error("bad vertex");vertices.push_back(v);}
   else if(kind=="f"){
    for(int i=0;i<3;++i){std::string token;if(!(s>>token))throw std::runtime_error("bad face");size_t n=std::stoul(token.substr(0,token.find('/')));if(!n||n>vertices.size())throw std::runtime_error("bad index");result.push_back(vertices[n-1]);}
    std::string extra;if(s>>extra)throw std::runtime_error("nontriangle face");++count;
   }else if(!kind.empty()&&kind[0]!='#'&&kind!="vt"&&kind!="vn"&&kind!="mtllib"&&kind!="usemtl")throw std::runtime_error("unsupported OBJ directive");
  }
  if(count!=a.at("triangles").get<size_t>())throw std::runtime_error("triangle count mismatch");
 }
 if(result.empty())throw std::runtime_error("empty scene");return result;
}
