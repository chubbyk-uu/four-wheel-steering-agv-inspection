#include <gtest/gtest.h>
#include "../src/optix/runtime_material.h"
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <atomic>
#include <thread>
using Json=nlohmann::json;
class RecipeTest:public testing::Test{
 protected:
 std::filesystem::path root;Json m;
 void SetUp()override{
  root=std::filesystem::temp_directory_path()/("agv_recipe_test_"+std::to_string(getpid()));std::filesystem::create_directory(root);
  m={{"schema","agv.material.recipe.probe.v1"},{"source_width",16},{"source_height",16},{"patch",8},{"ratio",1.},{"length",10.},
   {"field_width",1},{"field_height",1},{"core",4},{"gutter",1},{"nx",1},{"ny",1},{"ox",1.},{"oy",1.},{"texel",.25},{"qx",0.},{"qy",0.},{"gsd",1.},
   {"placements",{{0,0,0,0}}},{"cracks",Json::array()}};
  std::vector<unsigned char> color(16*16*3,128),normal=color,alpha(64,255);for(size_t i=2;i<normal.size();i+=3)normal[i]=255;
  float lut[256];for(int i=0;i<256;++i)lut[i]=float(i)/255;float one=1;
  File("color",color.data(),color.size());File("normal",normal.data(),normal.size());File("alpha",alpha.data(),alpha.size());File("lut",lut,sizeof(lut));File("pigment",&one,4);File("strength",&one,4);Save();
 }
 void TearDown()override{std::filesystem::remove_all(root);}
 void File(const std::string& name,const void* data,size_t size){auto file=name+".raw";std::ofstream out(root/file,std::ios::binary);out.write((const char*)data,size);unsigned char hash[32];SHA256((const unsigned char*)data,size,hash);std::ostringstream hex;for(auto b:hash)hex<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);m[name]=file;m["payload_sha256"][file]=hex.str();}
 void Save(){std::ofstream(root/"recipe.json")<<m.dump();}
};
TEST_F(RecipeTest,DeterministicGenerationAndDomainGuard){agv_linescan::RuntimeMaterial material(root/"recipe.json");std::vector<unsigned char> a(108),b(108);material.BakeHost(0,0,a.data());material.BakeHost(0,0,b.data());
 EXPECT_EQ(a,b);for(int i=0;i<36;++i){EXPECT_EQ(a[i],128);
 EXPECT_EQ(a[36+2*i],128);
 EXPECT_EQ(a[37+2*i],127);}EXPECT_THROW(material.BakeHost(-1,0,a.data()),std::runtime_error);
 EXPECT_THROW(material.BakeHost(1,0,a.data()),std::runtime_error);}
TEST_F(RecipeTest,RejectsChangedPayload){std::ofstream(root/"color.raw",std::ios::binary|std::ios::in).put(1);
 EXPECT_THROW(agv_linescan::RuntimeMaterial material(root/"recipe.json"),std::runtime_error);}
TEST_F(RecipeTest,RejectsWrongManifestDigest){EXPECT_THROW(agv_linescan::RuntimeMaterial material(root/"recipe.json",std::string(64,'0')),std::runtime_error);}
TEST_F(RecipeTest,RejectsMissingAlphaCoverage){std::vector<unsigned char> zero(64,0);File("alpha",zero.data(),zero.size());Save();agv_linescan::RuntimeMaterial material(root/"recipe.json");std::vector<unsigned char> a(108);
 EXPECT_THROW(material.BakeHost(0,0,a.data()),std::runtime_error);}
TEST_F(RecipeTest,RejectsCacheGridMismatch){agv_linescan::RuntimeMaterial material(root/"recipe.json");Json grid={{"tiles_x",1},{"tiles_y",1},{"core_pixels",4},{"gutter_pixels",1},{"texel_m",.25},{"origin_xy_m",{1.,1.}}};
 EXPECT_NO_THROW(material.CheckLayout(grid));grid["origin_xy_m"]={0.,1.};
 EXPECT_THROW(material.CheckLayout(grid),std::runtime_error);}
TEST_F(RecipeTest,ReleasesScratchGuardAfterFailedBake){agv_linescan::RuntimeMaterial material(root/"recipe.json");
 std::vector<unsigned char> expected(108),after(108);material.BakeHost(0,0,expected.data());
 // A throwing bake must not leave the single-caller flag latched; otherwise the
 // material cache would reject every later tile after one transient failure.
 EXPECT_THROW(material.BakeHost(1,0,after.data()),std::runtime_error);
 EXPECT_NO_THROW(material.BakeHost(0,0,after.data()));EXPECT_EQ(after,expected);}
TEST_F(RecipeTest,ConcurrentBakeIsRejectedNeverCorrupted){agv_linescan::RuntimeMaterial material(root/"recipe.json");
 std::vector<unsigned char> expected(108);material.BakeHost(0,0,expected.data());
 // Invariant test: every concurrent outcome is either the exact tile or the
 // single-caller rejection. It never asserts on thread interleaving.
 std::atomic<int> rejected{0},wrong{0},unexpected{0},ready{0};
 auto worker=[&]{++ready;while(ready.load()<2){}
  std::vector<unsigned char> local(108);
  for(int i=0;i<400;++i){
   try{material.BakeHost(0,0,local.data());if(local!=expected)++wrong;}
   catch(const std::runtime_error& e){
    if(std::string(e.what())=="runtime recipe scratch buffers are single-caller")++rejected;else ++unexpected;}}};
 std::thread a(worker),b(worker);a.join();b.join();
 EXPECT_EQ(wrong.load(),0);EXPECT_EQ(unexpected.load(),0);
 RecordProperty("concurrent_rejections",rejected.load());
 std::vector<unsigned char> final(108);EXPECT_NO_THROW(material.BakeHost(0,0,final.data()));EXPECT_EQ(final,expected);}
TEST_F(RecipeTest,OptionalPaintUsesWorldCoordinatesAndDoesNotAccumulateOverlaps){
 Json polygon={{"vertices",{{1.,1.},{1.5,1.},{1.5,1.5},{1.,1.5}}},{"color","white"}};
 m["inspection_paint"]={polygon,polygon};Save();agv_linescan::RuntimeMaterial material(root/"recipe.json");
 std::vector<unsigned char> a(108);material.BakeHost(0,0,a.data());
 for(int y=0;y<6;++y)for(int x=0;x<6;++x)EXPECT_EQ(a[y*6+x],(x>=1&&x<=2&&y>=1&&y<=2)?199:128);
}
TEST_F(RecipeTest,RejectsNonConvexOrClockwisePaint){
 m["inspection_paint"]={{{"vertices",{{1.,1.},{1.,2.},{2.,2.},{2.,1.}}},{"color","white"}}};Save();
 EXPECT_THROW(agv_linescan::RuntimeMaterial material(root/"recipe.json"),std::runtime_error);
}
