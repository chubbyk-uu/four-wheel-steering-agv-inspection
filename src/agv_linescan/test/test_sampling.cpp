#include <gtest/gtest.h>
#include "agv_linescan/flight_recorder.hpp"
#include "agv_linescan/mount_geometry.hpp"
#include "agv_linescan/sampling.hpp"
using namespace agv_linescan;
TEST(MountGeometry, ReadsSharedPivotAndRejectsMissingValues) {
  auto config=YAML::Load("mount_flex: {pivot_x_m: 1.025, pivot_z_m: 0.16}");
  auto pivot=LoadMountPivot(config);
  EXPECT_DOUBLE_EQ(pivot.x,1.025);
  EXPECT_DOUBLE_EQ(pivot.z,.16);
  EXPECT_THROW(LoadMountPivot(YAML::Load("mount_flex: {pivot_x_m: 1.0}")),std::runtime_error);
}
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
TEST(Trigger, PositionModeRetraceDoesNotDuplicateOrClosePartialImage) {
  const double pitch=1.5/4096;
  for(double sign:{1.,-1.}) {
    Trigger trigger(pitch,true);trigger.Update(0,0);
    auto first=trigger.Update(1,sign*2000.25*pitch);ASSERT_EQ(first.size(),2000u);
    EXPECT_TRUE(trigger.Update(2,sign*1980.25*pitch).empty());
    EXPECT_TRUE(trigger.Update(3,sign*1999.9*pitch).empty());
    EXPECT_TRUE(trigger.Update(4,sign*2000.25*pitch).empty());
    auto last=trigger.Update(5,sign*4096*pitch);ASSERT_EQ(last.size(),2096u);
    EXPECT_NEAR(last.front().distance-first.back().distance,sign*pitch,1e-12);
    EXPECT_GT(last.front().time,4.);
    EXPECT_NEAR(trigger.MaxRetrace(),20*pitch,1e-12);
    EXPECT_EQ(trigger.RetraceEpisodes(),1u);
    trigger.Reset();EXPECT_EQ(trigger.RetraceEpisodes(),0u);
  }
}

TEST(WheelEncoder, FractionalPhaseSurvivesTurnsPauseAndReverseScan) {
  WheelEncoderScale scale(2000,4,64,149,.2);
  EXPECT_NEAR(scale.linesPerTurn,3436.241610738255,1e-9);
  for(double sign:{-1.,1.}) {
    Trigger t(scale.spacing,true); t.Update(0,0);
    size_t lines=0;
    for(int i=1;i<=149;++i) {
      const double distance=sign*i*2*std::acos(-1.)*.2;
      lines+=t.Update(2*i-1,distance).size();
      EXPECT_TRUE(t.Update(2*i,distance).empty()); // stationary pause, no reset
    }
    EXPECT_EQ(lines,512000u); // no per-revolution rounding loss
  }
}
TEST(WheelEncoder, PhysicalDiameterChangesRawRowsNotCountsPerTurn) {
  WheelEncoderScale scale(2000,4,64,149,.2);
  for(double actualDiameter:{.39,.4,.42}) {
    Trigger t(scale.spacing,true);t.Update(0,0);
    size_t lines=0;
    // One metre of independent pure-rolling ground motion, integrated in steps.
    for(int i=1;i<=1000;++i)
      lines+=t.Update(i*.001,(i*.001)/(actualDiameter/2)*.2).size();
    EXPECT_NEAR(double(lines),scale.linesPerTurn/(std::acos(-1.)*actualDiameter),1.);
  }
  EXPECT_THROW(WheelEncoderScale(0,4,64,149,.2),std::invalid_argument);
  EXPECT_THROW(WheelEncoderScale(2000,4,63,149,.2),std::invalid_argument);
  EXPECT_THROW(WheelEncoderScale(2000,4,64,0,.2),std::invalid_argument);
}

TEST(FlightRecorder, KeepsTheSpanAndNeverExceedsCapacity) {
  FlightRecorder recorder(2000,2.0);
  FlightSample sample;
  for(int i=0;i<6000;++i){sample.simTime=i*.001;sample.residual=i*1e-6;recorder.Push(sample);}
  EXPECT_LE(recorder.size(),recorder.capacity());
  auto rows=recorder.Snapshot();
  ASSERT_FALSE(rows.empty());
  // Oldest first, and the whole window is within the declared span of the newest.
  EXPECT_LT(rows.front().simTime,rows.back().simTime);
  EXPECT_LE(rows.back().simTime-rows.front().simTime,recorder.span()+1e-9);
  EXPECT_NEAR(rows.back().simTime,5.999,1e-9);
}

TEST(FlightRecorder, ACoarserStepStillKeepsTheSameSimulatedWindow) {
  // The count limit must not be read as a duration: at 10 ms per step, 2000
  // samples would be 20 s, and only the span keeps the window at 2 s.
  FlightRecorder recorder(2000,2.0);
  FlightSample sample;
  for(int i=0;i<1000;++i){sample.simTime=i*.01;recorder.Push(sample);}
  auto rows=recorder.Snapshot();
  EXPECT_LE(rows.back().simTime-rows.front().simTime,2.0+1e-9);
  EXPECT_LE(rows.size(),size_t(201));
}

TEST(FlightRecorder, RejectsDegenerateBounds) {
  EXPECT_THROW(FlightRecorder(0,2.0),std::invalid_argument);
  EXPECT_THROW(FlightRecorder(10,0.0),std::invalid_argument);
}

TEST(ResidualStats, CountsCrossingsAndEstimatesQuantiles) {
  ResidualStats stats(.03);
  for(int i=0;i<990;++i)stats.Add(.001);
  for(int i=0;i<9;++i)stats.Add(.02);      // above half the limit, below it
  stats.Add(.05);                           // above the limit
  EXPECT_EQ(stats.samples(),1000u);
  EXPECT_EQ(stats.overHalf(),10u);          // the nine plus the one over the limit
  EXPECT_EQ(stats.overLimit(),1u);
  EXPECT_DOUBLE_EQ(stats.max(),.05);
  EXPECT_LT(stats.Quantile(.5),.01);
  EXPECT_GT(stats.Quantile(.999),.015);
}

TEST(FlightRecorder, ALateGuardCanCorrectTheStepItRejected) {
  // The polarity check runs after the sample is stored. Without this the dump
  // would show that guard passing on the very step that failed it.
  FlightRecorder recorder(16,2.0);
  EXPECT_EQ(recorder.Newest(),nullptr);
  FlightSample sample;sample.simTime=.001;recorder.Push(sample);
  ASSERT_NE(recorder.Newest(),nullptr);
  EXPECT_EQ(recorder.Newest()->passPolarity,1);
  recorder.Newest()->passPolarity=0;
  EXPECT_EQ(recorder.Snapshot().back().passPolarity,0);
}

TEST(ContactStats, CountsOnlyCaptureAndKeepsDropoutEpisodes) {
  CaptureContactStats stats;FlightSample sample;
  for(auto &contact:sample.contact)contact.available=1;
  stats.Add(sample); // armed state is intentionally excluded
  sample.capturing=1;
  sample.suspensionVel[0]=.012;
  stats.Add(sample);stats.Add(sample);
  sample.contact[0].points=1;sample.contact[0].stored=1;
  sample.contact[0].maxDepth=.0003f;
  sample.contact[0].point[0].normal[0]=1;sample.contact[0].point[0].normal[2]=1;
  stats.Add(sample);
  sample.contact[0].points=0;sample.contact[0].stored=0;stats.Add(sample);
  const auto &wheel=stats.wheels()[0];
  EXPECT_EQ(wheel.samples,4u);EXPECT_EQ(wheel.noContact,3u);
  EXPECT_EQ(wheel.noContactEpisodes,2u);EXPECT_EQ(wheel.longestNoContact,2u);
  EXPECT_NEAR(wheel.maxDepth,.0003,1e-8);EXPECT_NEAR(wheel.maxNormalTiltRad,M_PI/4,1e-12);
  EXPECT_EQ(wheel.suspensionRateHistogram[2],4u);
}

#include "agv_linescan/contact_kinematics.hpp"
TEST(ContactKinematics, SeparatesRollingSlipAndNormalCompression) {
  using gz::math::Vector3d;
  const Vector3d arm(0,0,-.2),normal(0,0,1);
  auto rolling=agv_linescan::ContactTangentVelocity(Vector3d(1,0,.03),Vector3d(0,5,0),arm,normal);
  ASSERT_TRUE(rolling);EXPECT_NEAR(rolling->Length(),0,1e-12);
  auto slip=agv_linescan::ContactTangentVelocity(Vector3d(.9,0,.03),Vector3d(0,5,0),arm,normal);
  ASSERT_TRUE(slip);EXPECT_NEAR(slip->X(),-.1,1e-12);EXPECT_NEAR(slip->Z(),0,1e-12);
  EXPECT_FALSE(agv_linescan::ContactTangentVelocity(Vector3d::Zero,Vector3d::Zero,arm,Vector3d::Zero));
}
