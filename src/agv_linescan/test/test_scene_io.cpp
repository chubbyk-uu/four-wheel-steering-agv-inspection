#include <gtest/gtest.h>
#include <chrono>
// Exercise the independent CPU reader without a CUDA context or GPU dependency.
struct float3 {float x,y,z;};
#include "scene_io.h"
class SceneIoTest:public ::testing::Test {
 protected:
 std::filesystem::path dir;
 void SetUp()override{dir=std::filesystem::temp_directory_path()/("agv_scene_io_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::filesystem::create_directory(dir);}
 void TearDown()override{std::filesystem::remove_all(dir);}
 std::filesystem::path scene(const std::string& text,size_t triangles=1){
  std::ofstream(dir/"mesh.obj")<<text;
  nlohmann::json m={{"schema","agv.shared.static_scene.v1"},{"units","m"},{"frame","world"},{"transform","identity_world_baked"},{"assets",nlohmann::json::array({{{"mesh","mesh.obj"},{"sha256",Sha256(dir/"mesh.obj")},{"triangles",triangles}}})}};
  std::ofstream(dir/"manifest.json")<<m;return dir/"manifest.json";
 }
 const std::string vertices="v 0 0 0\nv 1 0 0\nv 0 1 0\n";
};
TEST_F(SceneIoTest, PreservesDecimalExponentSignedZeroAndCornerOrder){
 auto v=LoadScene(scene("# comment\n \tv +1.25 -0 2e-3\r\nv 3 4 5\nv 6 7 8\nvt 0 0\nvn 0 0 1\nmtllib road.mtl\nusemtl road\nf 3/3/1 1/1/1 2/2/1\n"));
 ASSERT_EQ(v.size(),3u);EXPECT_EQ(v[0].x,6.f);EXPECT_EQ(v[1].x,1.25f);EXPECT_TRUE(std::signbit(v[1].y));EXPECT_EQ(v[1].z,.002f);EXPECT_EQ(v[2].z,5.f);
}
TEST_F(SceneIoTest, RejectsBadFaces){
 for(const auto& f:{"f 0 2 3","f -1 2 3","f 4 2 3","f 1x 2 3","f 1 2","f 1 2 3 1","f /1 2 3"})EXPECT_THROW(LoadScene(scene(vertices+f+"\n")),std::runtime_error);
}
TEST_F(SceneIoTest, RejectsBadVerticesAndCounts){
 for(const auto& v:{"v nan 0 0","v inf 0 0","v 1e99 0 0","v 1x 0 0","v 1 2"})EXPECT_THROW(LoadScene(scene(std::string(v)+"\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")),std::runtime_error);
 EXPECT_THROW(LoadScene(scene(vertices+"f 1 2 3\n",2)),std::runtime_error);
 EXPECT_THROW(LoadScene(scene(vertices+"g unexpected\nf 1 2 3\n")),std::runtime_error);
}
TEST_F(SceneIoTest, ChecksContentAndStreamsMultipleChunks){
 std::ofstream(dir/"payload",std::ios::binary)<<"abc";
 EXPECT_EQ(Sha256(dir/"payload"),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
 std::string bytes(150000,'q');std::ofstream(dir/"payload",std::ios::binary)<<bytes;
 unsigned char hash[32];SHA256((const unsigned char*)bytes.data(),bytes.size(),hash);std::ostringstream expected;for(auto c:hash)expected<<std::hex<<std::setfill('0')<<std::setw(2)<<int(c);
 EXPECT_EQ(Sha256(dir/"payload"),expected.str());
 auto path=scene(vertices+"f 1 2 3\n");std::ofstream(dir/"mesh.obj",std::ios::app)<<"# tampered\n";EXPECT_THROW(LoadScene(path),std::runtime_error);
 EXPECT_THROW(Sha256(dir/"missing"),std::runtime_error);
}
