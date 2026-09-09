#pragma once
#include <openssl/sha.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace agv_linescan {
// Worker-local, process-lifetime verification memo. Road assets are immutable
// during capture. Replacements or metadata changes force a fresh SHA256 check;
// this does not monitor files already resident on the GPU.
class VerifiedTileReader {
 public:
  struct Result {double read_seconds,hash_seconds;bool hashed;};
  Result Read(const std::filesystem::path& path,const std::string& digest,
              unsigned char* data,size_t count) {
    using Clock=std::chrono::steady_clock;
    const auto start=Clock::now();
    struct File {int fd;~File(){if(fd>=0)::close(fd);}} file{::open(path.c_str(),O_RDONLY|O_CLOEXEC)};
    struct stat before{},after{};
    if(file.fd<0 || ::fstat(file.fd,&before) || !S_ISREG(before.st_mode) ||
       before.st_size<0 || size_t(before.st_size)!=count)
      throw std::runtime_error("tile missing/truncated: "+path.filename().string());
    size_t done=0;
    while(done<count) {
      auto n=::read(file.fd,data+done,count-done);
      if(n<0 && errno==EINTR)continue;
      if(n<=0)throw std::runtime_error("tile read failed: "+path.filename().string());
      done+=size_t(n);
    }
    if(::fstat(file.fd,&after) || !Same(before,after))
      throw std::runtime_error("tile changed during read");
    const auto readEnd=Clock::now();
    auto previous=verified_.find(path.string());
    bool hashed=previous==verified_.end() || previous->second.digest!=digest || !Same(previous->second.identity,after);
    if(hashed) {
      unsigned char hash[32];SHA256(data,count,hash);std::ostringstream text;
      for(auto v:hash)text<<std::hex<<std::setw(2)<<std::setfill('0')<<int(v);
      if(text.str()!=digest)throw std::runtime_error("tile checksum mismatch: "+path.filename().string());
      if(::fstat(file.fd,&after) || !Same(before,after))
        throw std::runtime_error("tile changed during verification");
      verified_[path.string()]={after,digest};
    }
    return {std::chrono::duration<double>(readEnd-start).count(),
            hashed?std::chrono::duration<double>(Clock::now()-readEnd).count():0.,hashed};
  }
 private:
  static bool Same(const struct stat& a,const struct stat& b) {
    return a.st_dev==b.st_dev && a.st_ino==b.st_ino && a.st_size==b.st_size &&
      a.st_mtim.tv_sec==b.st_mtim.tv_sec && a.st_mtim.tv_nsec==b.st_mtim.tv_nsec &&
      a.st_ctim.tv_sec==b.st_ctim.tv_sec && a.st_ctim.tv_nsec==b.st_ctim.tv_nsec;
  }
  struct Verified {struct stat identity;std::string digest;};
  std::unordered_map<std::string,Verified> verified_;
};
}
