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
TEST(Preview, DisplayTransferPreservesEndpointsAndLinearAveraging){
 std::vector<uint8_t> raw{4,255};const auto original=raw;
 auto preview=agv_linescan::AreaPreview(raw,2,1,2);
 agv_linescan::DisplaySrgb(preview,4);
 EXPECT_EQ(preview[0],188);EXPECT_EQ(raw,original);
 std::vector<uint8_t> ramp(256);for(size_t i=0;i<256;++i)ramp[i]=i;
 agv_linescan::DisplaySrgb(ramp,4);
 EXPECT_EQ(ramp[0],0);EXPECT_EQ(ramp[4],0);EXPECT_EQ(ramp[255],255);
 EXPECT_GT(ramp[52],110);EXPECT_LT(ramp[52],130);
 EXPECT_TRUE(std::is_sorted(ramp.begin(),ramp.end()));
 EXPECT_THROW(agv_linescan::DisplaySrgb(ramp,255),std::invalid_argument);
}
