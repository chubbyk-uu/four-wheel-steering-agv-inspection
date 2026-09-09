#include "verified_tile_file.h"
#include <gtest/gtest.h>
#include <fstream>
#include <vector>
using namespace agv_linescan;
class TileFileTest:public ::testing::Test {
 protected:
 std::filesystem::path dir,path;VerifiedTileReader reader;std::vector<unsigned char> data=std::vector<unsigned char>(4);
 const std::string digest="88d4266fd4e6338d13b845fcf289579d209c897823b9217da3e161936f031589";
 void SetUp()override {
  char pattern[]="/tmp/agv_tile_file_XXXXXX";auto result=::mkdtemp(pattern);ASSERT_NE(result,nullptr);
  dir=result;path=dir/"tile.raw";Write(path,"abcd");
 }
 void TearDown()override{std::filesystem::remove_all(dir);}
 void Write(const std::filesystem::path& p,const std::string& value){std::ofstream(p,std::ios::binary)<<value;}
 auto Read(){return reader.Read(path,digest,data.data(),data.size());}
};
TEST_F(TileFileTest, UnchangedReloadReusesVerificationButStillReadsBytes) {
 EXPECT_TRUE(Read().hashed);data.assign(4,0);EXPECT_FALSE(Read().hashed);
 EXPECT_EQ(data,(std::vector<unsigned char>{'a','b','c','d'}));
}
TEST_F(TileFileTest, SameSizeCorruptionWithRestoredMtimeMustFail) {
 Read();auto time=std::filesystem::last_write_time(path);Write(path,"abce");
 std::filesystem::last_write_time(path,time);
 EXPECT_THROW(Read(),std::runtime_error);
}
TEST_F(TileFileTest, ReplacementWithSameSizeAndMtimeMustFail) {
 Read();auto time=std::filesystem::last_write_time(path);Write(dir/"new.raw","abce");
 std::filesystem::last_write_time(dir/"new.raw",time);std::filesystem::rename(dir/"new.raw",path);
 EXPECT_THROW(Read(),std::runtime_error);
}
TEST_F(TileFileTest, IdenticalReplacementIsVerifiedAgain) {
 Read();Write(dir/"new.raw","abcd");std::filesystem::rename(dir/"new.raw",path);
 EXPECT_TRUE(Read().hashed);
}
TEST_F(TileFileTest, ManifestDigestChangeAndTruncationCannotReuseMemo) {
 Read();EXPECT_THROW(reader.Read(path,std::string(64,'0'),data.data(),4),std::runtime_error);
 Write(path,"abc");EXPECT_THROW(Read(),std::runtime_error);
 std::filesystem::remove(path);EXPECT_THROW(Read(),std::runtime_error);
}
