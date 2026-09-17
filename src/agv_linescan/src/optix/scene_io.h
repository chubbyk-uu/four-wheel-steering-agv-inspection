#pragma once
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <iomanip>
#include <array>
#include <charconv>
#include <cmath>
#include <memory>
#include <string_view>
#include <openssl/evp.h>
inline nlohmann::json ReadJson(const std::filesystem::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("cannot open "+p.string());nlohmann::json j;f>>j;return j;}
inline std::string Sha256(const std::filesystem::path& p){
 std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("missing mesh");
 // Bounded streaming avoids istreambuf_iterator's byte-wise allocation/copy.
 std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),EVP_MD_CTX_free);
 if(!ctx||EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("SHA256 init failed");
 std::array<char,64*1024> buffer;
 while(f){f.read(buffer.data(),buffer.size());if(f.gcount()&&EVP_DigestUpdate(ctx.get(),buffer.data(),f.gcount())!=1)throw std::runtime_error("SHA256 update failed");}
 if(!f.eof())throw std::runtime_error("mesh read failed");
 unsigned char h[SHA256_DIGEST_LENGTH];unsigned length=0;
 if(EVP_DigestFinal_ex(ctx.get(),h,&length)!=1||length!=SHA256_DIGEST_LENGTH)throw std::runtime_error("SHA256 final failed");
 std::ostringstream o;for(auto c:h)o<<std::hex<<std::setfill('0')<<std::setw(2)<<int(c);return o.str();
}
// Independent strict OBJ reader: positive indices, triangulated faces, identity transform.
// Token views and from_chars avoid a locale-aware stringstream per OBJ line.
inline std::string_view ObjToken(std::string_view& line){
 auto begin=line.find_first_not_of(" \t\r\n\v\f");
 if(begin==line.npos){line={};return {};}
 line.remove_prefix(begin);auto end=line.find_first_of(" \t\r\n\v\f");
 auto token=line.substr(0,end);line=end==line.npos?std::string_view{}:line.substr(end);return token;
}
inline float ObjFloat(std::string_view token){
 if(!token.empty()&&token.front()=='+')token.remove_prefix(1);
 if(token.empty())throw std::runtime_error("bad vertex");
 float value=0;auto parsed=std::from_chars(token.data(),token.data()+token.size(),value);
 if(token.empty()||parsed.ec!=std::errc{}||parsed.ptr!=token.data()+token.size()||!std::isfinite(value))throw std::runtime_error("bad vertex");
 return value;
}
inline std::vector<float3> LoadScene(const std::filesystem::path& path){
 auto m=ReadJson(path);if(m.at("schema")!="agv.shared.static_scene.v1"||m.at("units")!="m"||m.at("frame")!="world"||m.at("transform")!="identity_world_baked")throw std::runtime_error("unsupported scene");
 std::vector<float3> result;
 for(const auto&a:m.at("assets")){
  auto file=path.parent_path()/a.at("mesh").get<std::string>();if(Sha256(file)!=a.at("sha256"))throw std::runtime_error("mesh checksum mismatch");
  std::ifstream f(file);if(!f)throw std::runtime_error("missing mesh");std::string line;std::vector<float3> vertices;size_t count=0;
  while(std::getline(f,line)){
   std::string_view rest(line);auto kind=ObjToken(rest);
   if(kind=="v"){float3 v;v.x=ObjFloat(ObjToken(rest));v.y=ObjFloat(ObjToken(rest));v.z=ObjFloat(ObjToken(rest));vertices.push_back(v);}
   else if(kind=="f"){
    for(int i=0;i<3;++i){auto token=ObjToken(rest);auto index=token.substr(0,token.find('/'));size_t n=0;
     if(index.empty())throw std::runtime_error("bad index");
     auto parsed=std::from_chars(index.data(),index.data()+index.size(),n);
     if(index.empty()||parsed.ec!=std::errc{}||parsed.ptr!=index.data()+index.size()||!n||n>vertices.size())throw std::runtime_error("bad index");result.push_back(vertices[n-1]);}
    if(!ObjToken(rest).empty())throw std::runtime_error("nontriangle face");++count;
   }else if(!kind.empty()&&kind[0]!='#'&&kind!="vt"&&kind!="vn"&&kind!="mtllib"&&kind!="usemtl")throw std::runtime_error("unsupported OBJ directive");
  }
  if(!f.eof())throw std::runtime_error("mesh read failed");
  if(count!=a.at("triangles").get<size_t>())throw std::runtime_error("triangle count mismatch");
 }
 if(result.empty())throw std::runtime_error("empty scene");return result;
}
