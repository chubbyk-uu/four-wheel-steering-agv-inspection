#include <gtest/gtest.h>
#include "../src/optix/runtime_material.h"
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
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
