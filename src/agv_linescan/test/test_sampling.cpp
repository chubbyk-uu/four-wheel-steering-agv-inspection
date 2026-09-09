#include <gtest/gtest.h>
#include "agv_linescan/sampling.hpp"
using namespace agv_linescan;
TEST(Trigger, MultipleLinesPerPhysicsStep) {
  Trigger trigger(1.2/4096);
  trigger.Update(0,0);
  auto events=trigger.Update(.001,(20./3.6)*.001);
  ASSERT_EQ(events.size(),18u);
  EXPECT_NEAR(events.front().time,52.734375e-6,1e-12);
  EXPECT_LT(events.back().time,.001);
}
TEST(Trigger, ReverseAndStop) {
  Trigger trigger(.01); trigger.Update(0,0);
  auto a=trigger.Update(1,-.025);
  EXPECT_EQ(a.size(),2u); EXPECT_EQ(a[0].direction,-1);
  EXPECT_TRUE(trigger.Update(2,-.025).empty());
  auto b=trigger.Update(3,-.04); ASSERT_EQ(b.size(),2u);
  EXPECT_NEAR(b.front().distance,-.03,1e-12);
  EXPECT_THROW(trigger.Update(4,0),std::runtime_error);
  trigger.Reset(); EXPECT_TRUE(trigger.Update(0,0).empty());
}
TEST(Optics, NominalRaysAndOverscan) {
  Optics lens(4096,5120,7e-6,.016,1.2,{0,1,0,.04});
  EXPECT_NEAR(lens.height,0.6696428571428571,1e-12);
  for (size_t i=0;i<4096;++i) {
    double renderedRay=((lens.source[i]+.5)*2/5120-1)*lens.span;
    double q=(double(i)-2047.5)/2048;
    EXPECT_NEAR(renderedRay,lens.scale*(q+.04*q*q*q),1e-12);
  }
}
TEST(Optics, RejectFoldedMapping) {
  EXPECT_THROW(Optics(4096,5120,7e-6,.020,1.2,{0,1,0,-1}),std::invalid_argument);
}
TEST(Pose, ExposureTimesRemainDistinct) {
  gz::math::Pose3d a(0,0,1,0,0,0), b(.005,0,1,0,0,.001);
  EXPECT_NEAR(Interpolate(a,b,.25).Pos().X(),.00125,1e-12);
  EXPECT_NEAR(Interpolate(a,b,.75).Rot().Yaw(),.00075,1e-9);
  EXPECT_THROW(Interpolate(a,b,1.1),std::runtime_error);
}

TEST(ScanMotion, ContactTransientToleranceKeepsLowSpeedAndSlipRejection) {
  // Recorded 10 km/h slab crossing: front/rear spread peaks at .015424 m/s.
  for(double direction:{-1.,1.}){
    EXPECT_GT(WheelSpeedSpreadLimit(direction*2.77778,.01,.01),.015424);
    EXPECT_LT(WheelSpeedSpreadLimit(direction*2.77778,.01,.01),.04); // Larger disagreement still rejects.
    EXPECT_LT(WheelSpeedSpreadLimit(direction*.1,.01,.01),.015424); // Low speed retains strict floor.
  }
  EXPECT_DOUBLE_EQ(WheelSpeedSpreadLimit(2.77778,.01,0),.01); // Legacy config.
  EXPECT_THROW(WheelSpeedSpreadLimit(0,.01,.2),std::invalid_argument);
}


#include "agv_linescan/scan_motion.hpp"
TEST(ScanMotion, CurvedMotionAndEquivalentReverseWheelBranches) {
  std::array<double,4> speed{},angle{};
  std::array<double,4> x={.65,.65,-.65,-.65},y={.47,-.47,.47,-.47};
  for(size_t i=0;i<4;++i){double vx=.5-.06*y[i],vy=.02+.06*x[i];speed[i]=std::hypot(vx,vy);angle[i]=std::atan2(vy,vx);}
  speed[2]*=-1;angle[2]+=M_PI;
  auto v=FitScanVelocity(speed,angle,1.3,.94);
  EXPECT_NEAR(v.vx,.5,1e-12);EXPECT_NEAR(v.vy,.02,1e-12);EXPECT_NEAR(v.wz,.06,1e-12);EXPECT_LT(v.residual,1e-12);
}
TEST(ScanMotion, ProjectedDistanceAndPureLateralMotion) {
  ProjectedEncoder encoder;std::array<double,4> p{},angle{};angle.fill(.1);encoder.Update(p,angle);
  p.fill(1);EXPECT_NEAR(encoder.Update(p,angle),std::cos(.1),1e-12);
  encoder.Reset();p.fill(0);angle.fill(M_PI/2);encoder.Update(p,angle);p.fill(1);
  EXPECT_NEAR(encoder.Update(p,angle),0,1e-12);
}

TEST(Trigger, PauseRetainsTwoThousandLinesAndFractionalPitch) {
  const double pitch=1.5/4096;
  Trigger trigger(pitch);trigger.Update(0,0);
  auto first=trigger.Update(1,2000.25*pitch);
  ASSERT_EQ(first.size(),2000u);
  for(int t=2;t<=60;++t) EXPECT_TRUE(trigger.Update(t,2000.25*pitch).empty());
  auto remaining=trigger.Update(61,4096*pitch);
  ASSERT_EQ(remaining.size(),2096u);
  EXPECT_NEAR(remaining.front().distance-first.back().distance,pitch,1e-12);
  EXPECT_GT(remaining.front().time,60.);
  EXPECT_NEAR(remaining.back().distance,1.5,1e-10);
}
TEST(Trigger, SubPulseRestJitterDoesNotChooseOrReverseScanDirection) {
  Trigger trigger(.01);trigger.Update(0,0);
  EXPECT_TRUE(trigger.Update(1,-.0001).empty());
  auto first=trigger.Update(2,.025);ASSERT_EQ(first.size(),2u);
  EXPECT_NEAR(first.front().distance,.01,1e-12);
  EXPECT_TRUE(trigger.Update(3,.02499).empty());
  EXPECT_TRUE(trigger.Update(4,.025).empty());
  auto last=trigger.Update(5,.03);ASSERT_EQ(last.size(),1u);
  EXPECT_NEAR(last.front().distance,.03,1e-12);
  EXPECT_THROW(trigger.Update(6,.015),std::runtime_error);
}
