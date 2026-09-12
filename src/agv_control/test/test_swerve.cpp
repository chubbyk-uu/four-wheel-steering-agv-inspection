#include "agv_control/swerve.hpp"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
using namespace agv;
TEST(Kinematics, ReconstructAllModes){
 Config cfg;Controller c(cfg); Four zero{};
 for(Twist t: {Twist{1,0,0},Twist{0,1,0},Twist{.4,.3,0},Twist{0,0,.3},Twist{.2,.3,.2},Twist{-1,0,0}}){
  auto r=c.allocate(t,zero,zero);
  for(size_t i=0;i<4;++i){
   EXPECT_NEAR(r.speed[i]*std::cos(r.angle[i]),t.x-t.yaw*(i%2==0?1:-1)*Config{}.track/2,1e-9);
   EXPECT_NEAR(r.speed[i]*std::sin(r.angle[i]),t.y+t.yaw*(i<2?1:-1)*Config{}.wheelbase/2,1e-9);
  }
 }
}
TEST(Kinematics, MechanicalLimitsAndReverse){
 Config cfg;Controller c(cfg);Four a; a.fill(cfg.soft_lower+4*pi/180);
 auto r=c.allocate({1,0,0},a,a);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],-pi,1e-9);EXPECT_LT(r.speed[i],0);}
 for(int deg=-180;deg<=180;++deg){double h=deg*pi/180;auto q=c.allocate({std::cos(h),std::sin(h),0},a,a);
  for(double angle:q.angle){EXPECT_GE(angle,cfg.soft_lower);EXPECT_LE(angle,cfg.soft_upper);}}
}
TEST(Kinematics, ZeroHoldsAndLimits){
 Controller c;Four a{1,2,-1,-2};auto r=c.allocate({},a,a);EXPECT_EQ(r.angle,a);
 // Read the cap rather than restating it. This request is dominated by the
 // lateral limit, so add one that the vehicle top speed alone has to hold.
 const double cap=Config{}.max_speed;
 auto q=c.allocate({20,20,3},a,a);for(double v:q.speed)EXPECT_LE(std::abs(v),cap+1e-9);
 auto f=c.allocate({20,0,0},a,a);
 for(double v:f.speed)EXPECT_LE(std::abs(v),cap+1e-9);
 EXPECT_NEAR(Controller::max_abs(f.speed),cap,1e-9);
 EXPECT_THROW(c.allocate({NAN,0,0},a,a),std::invalid_argument);
}
TEST(Transitions, StopBeforeLateralSteering){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<300;++n){r=c.update({.5,0,0},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_EQ(c.mode(),Mode::Drive);ASSERT_GT(v[0],.4);
 auto before=a;r=c.update({0,.5,0},a,v,.01);
 EXPECT_EQ(c.mode(),Mode::Brake);EXPECT_EQ(r.angle,before);
 // Measured wheel motion prevents advancing to ALIGN even after commands reach zero.
 for(int n=0;n<100;++n)r=c.update({0,.5,0},a,v,.01);
 EXPECT_EQ(c.mode(),Mode::Brake);EXPECT_EQ(r.angle,before);
 v.fill(0);
 bool saw_align=false;
 for(int n=0;n<500;++n){r=c.update({0,.5,0},a,v,.01);if(c.mode()==Mode::Align){saw_align=true;EXPECT_LT(Controller::max_abs(r.speed),1e-9);}a=r.angle;v=r.speed;}
 EXPECT_TRUE(saw_align);EXPECT_EQ(c.mode(),Mode::Drive);EXPECT_GT(std::abs(v[0]),.4);
}
TEST(Transitions, SmallChangeContinuousReverseBrakes){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<200;++n){r=c.update({.5,0,0},a,v,.01);a=r.angle;v=r.speed;}
 r=c.update({.5,.02,0},a,v,.01);EXPECT_EQ(c.mode(),Mode::Drive);
 r=c.update({-.5,0,0},a,v,.01);EXPECT_EQ(c.mode(),Mode::Brake);
}
TEST(Transitions, SteeringRateAndAcceleration){
 Controller c;Four a{},v{},old_rate{};Target r;
 for(int n=0;n<400;++n){r=c.update({0,.5,0},a,v,.01);
  for(size_t i=0;i<4;++i){double rate=(r.angle[i]-a[i])/.01;EXPECT_LE(std::abs(rate),Config{}.rate+1e-8);EXPECT_LE(std::abs(rate-old_rate[i]),Config{}.steer_accel*.01+1e-8);old_rate[i]=rate;}
  a=r.angle;v=r.speed;
 }
 EXPECT_EQ(c.mode(),Mode::Drive);
}
TEST(Transitions, HighSpeedSmallAngleRequiresBraking){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<900;++n){r=c.update({5,0,0},a,v,.01);a=r.angle;v=r.speed;}
 // An excessive request is capped at the vehicle top speed before it may
 // change direction. Read the cap rather than restating it, or a change to the
 // platform spec silently turns this into a different test.
 ASSERT_NEAR(v[0],Config{}.max_speed,1e-9);
 r=c.update({5,.25,0},a,v,.01);EXPECT_EQ(c.mode(),Mode::Brake);
}
TEST(Transitions, RestartWhileMovingBrakesBeforeSteering){
 Controller c;Four a{},v{.4,.4,.4,.4};
 auto r=c.update({0,.5,0},a,v,.01);
 EXPECT_EQ(c.mode(),Mode::Brake);EXPECT_EQ(r.angle,a);
 EXPECT_NEAR(r.speed[0],.4-Config{}.decel*.01,1e-9);
 c.reset();r=c.update({},a,v,.01);EXPECT_EQ(c.mode(),Mode::Brake);
}
TEST(Transitions, AlignmentNeedsActualWheelAngles){
 Controller c;Four a{},v{};
 for(int n=0;n<500;++n){auto r=c.update({0,.5,0},a,v,.01);EXPECT_EQ(Controller::max_abs(r.speed),0);}
 EXPECT_EQ(c.mode(),Mode::Align);
}
TEST(Configuration, RejectInvalidAndTime){
 Config p;p.rate=0;EXPECT_THROW(Controller c(p),std::invalid_argument);
 Controller c;Four a{};EXPECT_THROW(c.update({},a,a,0),std::invalid_argument);
 EXPECT_THROW(c.update({},a,a,.2),std::invalid_argument);
 a[0]=NAN;EXPECT_THROW(c.update({},a,a,.01),std::invalid_argument);
}
TEST(Kinematics, ExplicitLimitReconfigurationSelectsInterior){
 Config cfg;Controller c(cfg);Four a; a.fill(cfg.soft_upper-pi/180);
 for(double vx:{-1.0,1.0}) {
  auto r=c.allocate({vx,0,0},a,a,true);
  for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],0,1e-9);EXPECT_NEAR(r.speed[i],vx,1e-9);}
 }
 // The branch this placement leaves beside a limit is lateral +pi/2. Continuity
 // holds a wheel there; only a latched limit approach moves it to the partner.
 Four crabbing;crabbing.fill(pi/2);
 const auto held=c.allocate({0,.5,0},crabbing,crabbing);
 const auto moved=c.allocate({0,.5,0},crabbing,crabbing,true);
 for(size_t i=0;i<4;++i){
  EXPECT_NEAR(held.angle[i],pi/2,1e-9);EXPECT_NEAR(held.speed[i],.5,1e-9);
  EXPECT_NEAR(moved.angle[i],-pi/2,1e-9);EXPECT_NEAR(moved.speed[i],-.5,1e-9);
 }
 Four previous{};
 for(int deg=-15;deg<=15;++deg) {
  const double h=deg*pi/180;
  auto r=c.allocate({std::cos(h),std::sin(h),0},previous,previous);
  for(double angle:r.angle)EXPECT_NEAR(angle,h,1e-9);
  previous=r.angle;
 }
}
TEST(Transitions, DirectReverseNeverTurnsWheels){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<200;++n){r=c.update({.5,0,0},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_GT(v[0],.4);
 bool stopped=false;
 for(int n=0;n<400;++n){
  r=c.update({-.5,0,0},a,v,.01);
  for(double angle:r.angle)EXPECT_NEAR(angle,0,1e-12);
  if(std::abs(r.speed[0])<1e-9)stopped=true;
  if(r.speed[0]<0){EXPECT_TRUE(stopped);}
  a=r.angle;v=r.speed;
 }
 EXPECT_EQ(c.mode(),Mode::Drive);EXPECT_LT(v[0],-.49);
}
TEST(Transitions, DiagonalReverseKeepsWheelAxes){
 for(double degrees:{-90.,-75.,-45.,-15.,15.,45.,75.,90.}) {
  Controller c;Four a{},v{};Target r;
  const double h=degrees*pi/180;
  const Twist forward{.4*std::cos(h),.4*std::sin(h),0};
  for(int n=0;n<500;++n){r=c.update(forward,a,v,.01);a=r.angle;v=r.speed;}
  ASSERT_EQ(c.mode(),Mode::Drive);
  const auto before=a;
  bool zero_crossing=false;
  for(int n=0;n<400;++n){
   r=c.update({-forward.x,-forward.y,0},a,v,.01);
   for(size_t i=0;i<4;++i)EXPECT_NEAR(r.angle[i],before[i],1e-8);
   if(Controller::max_abs(r.speed)<1e-9)zero_crossing=true;
   if(r.speed[0]*v[0]<0){EXPECT_TRUE(zero_crossing);}
   a=r.angle;v=r.speed;
  }
  EXPECT_EQ(c.mode(),Mode::Drive);
  EXPECT_TRUE(zero_crossing);
  EXPECT_NEAR(v[0]*std::cos(a[0]),-forward.x,1e-8);
  EXPECT_NEAR(v[0]*std::sin(a[0]),-forward.y,1e-8);
 }
}
TEST(Kinematics, ReverseWholeTwistNegatesEveryWheelSpeed){
 Controller c;Four z{};
 for(Twist t:{Twist{.3,.2,.4},Twist{0,0,.4},Twist{.1,-.3,-.2}}){
  const auto f=c.allocate(t,z,z,true);
  const auto b=c.allocate({-t.x,-t.y,-t.yaw},f.angle,f.angle);
  for(size_t i=0;i<4;++i){EXPECT_NEAR(b.angle[i],f.angle[i],1e-9);EXPECT_NEAR(b.speed[i],-f.speed[i],1e-9);}
 }
}
TEST(Kinematics, AtanBranchCutIsNotMechanicalDiscontinuity){
 Controller c;Four a;a.fill(-179*pi/180);
 auto r=c.allocate({.3*std::cos(-181*pi/180),.3*std::sin(-181*pi/180),0},a,a);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],-181*pi/180,1e-9);EXPECT_GT(r.speed[i],0);}
}
TEST(Kinematics, ZeroSpeedModuleHoldsThroughNoise){
 Controller c;Four a{.7,0,0,0};
 for(double noise:{-.001,-.00001,0.,.00001,.001}) {
  auto r=c.allocate({.4*Config{}.track/2+noise,-.4*Config{}.wheelbase/2,.4},a,a);
  EXPECT_DOUBLE_EQ(r.angle[0],a[0]);EXPECT_DOUBLE_EQ(r.speed[0],0);
  EXPECT_GT(Controller::max_abs(r.speed),.1);
 }
}
TEST(Transitions, ContinuousHeadingSweepReconfiguresBeforeLimit){
 for(double direction:{-1.,1.}) {
  Config cfg;Controller c(cfg);Four a{},v{};Target r;
  int limit_stops=0;double least_margin=1e9;Mode old_mode=Mode::Hold;
  for(int n=0;n<7000;++n){
   const double heading=direction*.2*n*.01;
   r=c.update({.25*std::cos(heading),.25*std::sin(heading),0},a,v,.01);
   if(old_mode==Mode::Drive && c.mode()==Mode::Brake && c.reason()=="LIMIT_RECONFIGURE") {
    ++limit_stops;for(double angle:a)EXPECT_GT(cfg.margin(angle),cfg.limit_reserve);
   }
   for(size_t i=0;i<4;++i){
    EXPECT_LE(std::abs(r.angle[i]-a[i]),cfg.rate*.01+1e-9);
    least_margin=std::min(least_margin,cfg.margin(r.angle[i]));
   }
   if(c.mode()==Mode::Align){EXPECT_LT(Controller::max_abs(r.speed),.025);}
   old_mode=c.mode();a=r.angle;v=r.speed;
  }
  EXPECT_GE(limit_stops,3);EXPECT_GT(least_margin,cfg.limit_reserve);
 }
}
TEST(Transitions, PureSpinDoesNotAccumulateSteeringTravel){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<400;++n){r=c.update({0,0,.3},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_EQ(c.mode(),Mode::Drive);const auto before=a;
 for(int n=0;n<2000;++n){
  r=c.update({0,0,.3},a,v,.01);
  EXPECT_EQ(c.mode(),Mode::Drive);
  for(size_t i=0;i<4;++i)EXPECT_NEAR(r.angle[i],before[i],1e-8);
  a=r.angle;v=r.speed;
 }
}
TEST(Transitions, AlignmentRequiresActualSteeringToSettle){
 Controller c;Four a{},v{},rates{.2,.2,.2,.2};Target r;
 for(int n=0;n<100;++n){r=c.update({.5,0,0},a,v,.01,rates);a=r.angle;v=r.speed;}
 EXPECT_EQ(c.mode(),Mode::Align);EXPECT_DOUBLE_EQ(Controller::max_abs(v),0);
 rates.fill(0);
 for(int n=0;n<100;++n){r=c.update({.5,0,0},a,v,.01,rates);a=r.angle;v=r.speed;}
 EXPECT_EQ(c.mode(),Mode::Drive);
}
TEST(Kinematics, OrdinaryStoppedChoiceIsShortestNotForcedHome){
 Controller c;Four a;a.fill(-170*pi/180);
 const double heading=-160*pi/180;
 const auto r=c.allocate({.3*std::cos(heading),.3*std::sin(heading),0},a,a,false,true);
 for(double angle:r.angle)EXPECT_NEAR(angle,heading,1e-9);
}
TEST(Kinematics, StationaryShortestChoiceMustLeaveSegmentMargin){
 // Ten degrees from the +pi/2 branch, the short turn is 17 times cheaper than
 // its partner and still refused: a stopped vehicle must not start there.
 Config cfg;Controller c(cfg);Four a;a.fill(80*pi/180);
 const auto r=c.allocate({0,.3,0},a,a,false,true);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],-pi/2,1e-9);EXPECT_NEAR(r.speed[i],-.3,1e-9);}
}
TEST(Kinematics, SpinRecoveryNeedsNoLongUnwindUnderAsymmetricLimits){
 Config cfg;cfg.wheelbase=1.3;cfg.track=.94;cfg.limit_reserve=10*pi/180;Controller c(cfg);Four zero{};
 for(double direction:{-1.,1.}) {
  const auto direct=c.allocate({0,0,direction*.3},zero,zero,false,true);
  const auto straight=c.allocate({.4,0,0},direct.angle,direct.angle,false,true);
  for(size_t i=0;i<4;++i) {
   EXPECT_LT(std::abs(direct.angle[i]),pi/2);
   EXPECT_NEAR(straight.angle[i],0,1e-9);
  }
  // Crab, spin, then recover. Under +-190 deg two wheels came out of the spin at
  // 125.87 deg, whose 54.13 deg branch at 180 deg had only 5 deg of margin, so
  // the only legal straight branch was 0 deg: a 126 deg turn mid-mission. Here
  // both straight branches are 95 deg clear and every turn stays at 54.13 deg.
  Four zero{};
  const auto lateral=c.allocate({0,direction*.4,0},zero,zero,false,true);
  const auto spin=c.allocate({0,0,direction*.3},lateral.angle,lateral.angle,false,true);
  const auto recovered=c.allocate({.4,0,0},spin.angle,spin.angle,false,true);
  for(size_t i=0;i<4;++i) {
   // Both lateral branches drive the same axis; only one clears the margin.
   EXPECT_NEAR(lateral.angle[i],-pi/2,1e-9);
   EXPECT_NEAR(lateral.speed[i]*std::sin(lateral.angle[i]),direction*.4,1e-9);
   EXPECT_NEAR(std::abs(spin.angle[i]-lateral.angle[i]),35.8698235177*pi/180,1e-9);
   EXPECT_NEAR(std::abs(recovered.angle[i]-spin.angle[i]),54.1301764823*pi/180,1e-9);
   EXPECT_NEAR(recovered.speed[i]*std::cos(recovered.angle[i]),.4,1e-9);
   EXPECT_GE(cfg.margin(recovered.angle[i]),cfg.segment_margin);
  }
 }
}
TEST(Kinematics, StoppedVehicleDoesNotStartASegmentBesideTheSteeringLimit){
 // A branch within the segment margin of a limit is kinematically identical to
 // its partner but leaves no travel for a pass of cross-track and heading
 // correction: parking 5 deg from the limit cost a 100 m pass a 180 deg
 // reconfiguration 15 mm before the end of the scan. Re-steering is free while
 // stopped, so the interior branch is taken even though it is the longer turn.
 // Moving the span off centre relocates that branch from straight ahead to
 // lateral +pi/2 but cannot remove it: some direction always owns it.
 Config cfg;cfg.wheelbase=1.3;cfg.track=.94;cfg.limit_reserve=3*pi/180;
 Controller c(cfg);
 for(double sign:{-1.,1.}) {
  Four crabbing;crabbing.fill(pi/2);
  const auto stopped_start=c.allocate({0,sign*.4,0},crabbing,crabbing,false,true);
  const auto driving=c.allocate({0,sign*.4,0},crabbing,crabbing,false,false);
  for(size_t i=0;i<4;++i) {
   // Continuity alone still prefers the near-limit branch, which is how a wheel
   // reaches it in the first place; only the stopped case refuses it.
   EXPECT_NEAR(driving.angle[i],pi/2,1e-9);
   EXPECT_LT(cfg.margin(driving.angle[i]),cfg.segment_margin);
   EXPECT_NEAR(stopped_start.angle[i],-pi/2,1e-9);
   EXPECT_GE(cfg.margin(stopped_start.angle[i]),cfg.segment_margin);
   EXPECT_NEAR(stopped_start.speed[i]*std::sin(stopped_start.angle[i]),sign*.4,1e-9);
   EXPECT_GT(std::abs(stopped_start.angle[i]-crabbing[i]),std::abs(driving.angle[i]-crabbing[i]));
  }
  // Dynamic braking margin still triggers well before the hard endpoint.
  Four a{},v{};Mode old=Mode::Hold;bool stopped=false;
  for(int k=0;k<3000;++k){
   double h=sign*.2*k*.01;
   auto out=c.update({.25*std::cos(h),.25*std::sin(h),0},a,v,.01);
   if(old==Mode::Drive && c.mode()==Mode::Brake && c.reason()=="LIMIT_RECONFIGURE")stopped=true;
   for(double angle:out.angle)EXPECT_GT(cfg.margin(angle),0);
   old=c.mode();a=out.angle;v=out.speed;
  }
  EXPECT_TRUE(stopped);
 }
}
TEST(Transitions, OneWheelCrossesZeroWithoutStoppingWholeVehicle){
 Config cfg;Controller c(cfg);Four a{},v{};Target r;
 const double yaw=.3, x=yaw*cfg.track/2+.05, y=-yaw*cfg.wheelbase/2;
 for(int n=0;n<500;++n){r=c.update({x,y,yaw},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_EQ(c.mode(),Mode::Drive);
 for(int n=0;n<1000;++n){
  r=c.update({x-.0001*n,y,yaw},a,v,.01);
  EXPECT_EQ(c.mode(),Mode::Drive);
  EXPECT_NEAR(r.angle[0],0,1e-7);
  a=r.angle;v=r.speed;
 }
 EXPECT_NEAR(v[0],-.0499,.001);EXPECT_NEAR(v[1],yaw*cfg.track-.0499,.001);
}

TEST(DriveLimits, SeparateAccelerationAndDecelerationBothSigns){
 for(double sign : {-1.,1.}) {
  Config cfg;cfg.accel=.8;cfg.decel=1.;Controller c(cfg);Four a{},v{};
  for(int k=0;k<150;++k){auto r=c.update({sign,0,0},a,v,.01);
   for(size_t i=0;i<4;++i)EXPECT_LE(std::abs(r.speed[i]-v[i]),.008+1e-10);
   a=r.angle;v=r.speed;}
  ASSERT_NEAR(v[0],sign,1e-8);
  auto slower=c.update({sign*.2,0,0},a,v,.01);
  EXPECT_NEAR(slower.speed[0],sign*.99,1e-8);
  a=slower.angle;v=slower.speed;
  auto stop=c.update({},a,v,.01);
  EXPECT_EQ(c.mode(),Mode::Brake);EXPECT_NEAR(stop.speed[0],sign*.98,1e-8);
  a=stop.angle;v=stop.speed;
  for(int k=0;k<350;++k){auto r=c.update({-sign,0,0},a,v,.01);
   for(size_t i=0;i<4;++i){
    EXPECT_GE(v[i]*r.speed[i],-1e-12);
    double rate=std::abs(r.speed[i])>std::abs(v[i])?.8:1.;
    EXPECT_LE(std::abs(r.speed[i]-v[i]),rate*.01+1e-10);
   }
   a=r.angle;v=r.speed;}
  EXPECT_NEAR(v[0],-sign,1e-8);
 }
}
TEST(DriveLimits, RejectInvalidDeceleration){
 Config c;for(double value:std::array<double,4>{0.,-1.,INFINITY,NAN}){c.decel=value;EXPECT_THROW(Controller{c},std::invalid_argument);}
}
TEST(Configuration, RejectSteeringLimitsThatCannotCoverEveryDirection){
 // A span below pi+2*segment_margin leaves some direction with no branch a
 // stopped vehicle may start on, whichever way the span is placed.
 Config narrow;narrow.soft_lower=-110*pi/180;narrow.soft_upper=105*pi/180;
 EXPECT_LT(narrow.span(),pi+2*narrow.segment_margin);
 EXPECT_THROW(Controller{narrow},std::invalid_argument);
 // Straight ahead is the mechanical zero and must stay directly commandable.
 for(auto pair:{std::pair<double,double>{10*pi/180,240*pi/180},{-370*pi/180,-10*pi/180}}) {
  Config offset;offset.soft_lower=pair.first;offset.soft_upper=pair.second;
  EXPECT_GE(offset.span(),pi+2*offset.segment_margin);
  EXPECT_THROW(Controller{offset},std::invalid_argument);
 }
 // The shipped placement is the widest guaranteed margin its span can give.
 Config shipped;
 for(double shift=-95*pi/180;shift<=95*pi/180;shift+=pi/180) {
  Config moved;moved.soft_lower=shipped.soft_lower+shift;moved.soft_upper=shipped.soft_upper+shift;
  if(moved.soft_lower>=0||moved.soft_upper<=0)continue;
  EXPECT_LE(std::min(moved.margin(0),moved.margin(-pi)),shipped.margin(0)+1e-12)<<shift;
 }
}


TEST(Alignment, BriefPassiveRollingKeepsDrivesZeroButSustainedMotionBrakes) {
 Config cfg;Controller c(cfg);Four a{1,1,1,1},v{};
 c.update({.1,0,0},a,v,.01);ASSERT_EQ(c.mode(),Mode::Align);
 v.fill(.03);auto out=c.update({.1,0,0},a,v,.01);
 EXPECT_EQ(c.mode(),Mode::Align);for(auto speed:out.speed)EXPECT_DOUBLE_EQ(speed,0);
 v.fill(0);c.update({.1,0,0},a,v,.01);EXPECT_EQ(c.mode(),Mode::Align);
 v.fill(.03);for(int i=0;i<7;++i)c.update({.1,0,0},a,v,.01);
 EXPECT_EQ(c.mode(),Mode::Brake);
 Controller significant(cfg);v.fill(0);significant.update({.1,0,0},a,v,.01);
 v.fill(.051);significant.update({.1,0,0},a,v,.01);EXPECT_EQ(significant.mode(),Mode::Brake);
}

TEST(Configuration, DefaultsMatchShippedPlatformAndPolicy) {
 Config c;
 auto platform=YAML::LoadFile(std::string(AGV_CONFIG_ROOT)+"/agv_description/config/platform.yaml");
 auto policy=YAML::LoadFile(std::string(AGV_CONFIG_ROOT)+"/agv_bringup/config/motion.yaml")["swerve_controller"]["ros__parameters"];
 for(auto pair: {std::pair<const char*,double>{"wheelbase",c.wheelbase},{"track",c.track},
   {"wheel_radius",c.radius},{"steer_soft_lower",c.soft_lower},
   {"steer_soft_upper",c.soft_upper},{"steer_rate",c.rate},
   {"steer_accel",c.steer_accel},{"max_speed",c.max_speed},{"max_yaw_rate",c.max_yaw},
   {"drive_accel",c.accel},{"drive_decel",c.decel},{"max_lateral_speed",c.max_lateral_speed}}) {
  EXPECT_NEAR(platform[pair.first].as<double>(),pair.second,1e-12)<<pair.first;
 }
 EXPECT_FALSE(policy["max_lateral_speed"]); // One shared limit for allocator and tracker.
 EXPECT_NEAR(policy["steering_limit_reserve"].as<double>(),c.limit_reserve,1e-12);
 EXPECT_NEAR(policy["steering_segment_margin"].as<double>(),c.segment_margin,1e-12);
 // Cross-package invariant: the travel a stopped vehicle keeps in hand must
 // cover the largest wheel deviation the tracker is allowed to ask for, where
 // the lateral command and the yaw rate are each bounded by the same ratio.
 auto tracking=YAML::LoadFile(std::string(AGV_CONFIG_ROOT)+"/agv_mission/config/tracking.yaml");
 const double ratio=tracking["cross_command_ratio"].as<double>();
 EXPECT_GE(c.segment_margin,std::atan(ratio)+c.limit_reserve);
 // Every direction has a branch clear of that margin iff the span is at least
 // pi+2m; the shipped span carries 95 deg against a 20 deg margin.
 EXPECT_GE(c.span(),pi+2*c.segment_margin);
 EXPECT_NEAR((c.span()-pi)/2,95*pi/180,1e-12);
 // Straight ahead is held for a whole 110 s pass, so both of its branches, not
 // just one, must clear the margin. That is what the placement buys.
 for(double branch:{0.,-pi})EXPECT_GE(c.margin(branch),c.segment_margin);
 // Hard limits keep 5 deg beyond the software limit on each side; with the 3 deg
 // dynamic reserve that is the same 8 deg of mechanical reserve as before.
 const double lower=platform["steer_hard_lower"].as<double>();
 const double upper=platform["steer_hard_upper"].as<double>();
 EXPECT_NEAR(c.soft_lower-lower+c.limit_reserve,8*pi/180,1e-12);
 EXPECT_NEAR(upper-c.soft_upper+c.limit_reserve,8*pi/180,1e-12);
 EXPECT_NEAR(upper-lower,380*pi/180,1e-12);
 // Every direction the planner drives a segment along must clear that margin,
 // including a reverse pass starting from wheels already parked at -180 deg.
 Controller allocator(c);
 for(auto request:{Twist{.4,0,0},Twist{-.4,0,0},Twist{0,.4,0},Twist{0,-.4,0}}) {
  Four parked;parked.fill(-pi);
  const auto stopped=allocator.allocate(request,parked,parked,false,true);
  for(size_t i=0;i<4;++i)EXPECT_GE(c.margin(stopped.angle[i]),c.segment_margin);
 }
 // The vehicle's top speed and the fastest scan a mission may request are
 // different numbers. A scan commanded at the top speed would leave the tracker
 // no authority to accelerate, only to brake, so the rated inspection speed has
 // to clear the top speed by at least that authority.
 const double rated=platform["rated_scan_speed"].as<double>();
 EXPECT_NEAR(c.max_speed,15/3.6,1e-12);   // 15 km/h vehicle top speed
 EXPECT_NEAR(rated,10/3.6,1e-12);         // 10 km/h rated inspection speed
 EXPECT_LT(rated,c.max_speed);
 EXPECT_LE(rated+tracking["max_position_feedback_m_s"].as<double>(),c.max_speed);
 EXPECT_NEAR(std::atan(c.wheelbase/c.track)*180/pi,54.1301764823,1e-6);
}
