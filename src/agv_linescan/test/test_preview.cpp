#include "agv_linescan/preview.hpp"
#include <gtest/gtest.h>
TEST(Preview, IntegratesThinLinesInsteadOfSkippingThem){
 std::vector<uint8_t> input(64*64,255);
 for(size_t y=0;y<64;++y)input[y*64+7]=0;
 auto out=agv_linescan::AreaPreview(input,64,64);
 ASSERT_EQ(out.size(),4u);EXPECT_EQ(out[0],247);EXPECT_EQ(out[2],247);EXPECT_EQ(out[1],255);EXPECT_EQ(out[3],255);
}
TEST(Preview, PartialBinUsesActualArea){
 std::vector<uint8_t> input(35*3,90);auto out=agv_linescan::AreaPreview(input,35,3);
 ASSERT_EQ(out.size(),2u);EXPECT_EQ(out[0],90);EXPECT_EQ(out[1],90);
}
