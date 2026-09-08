#include "agv_control/swerve.hpp"
#include <gtest/gtest.h>
using namespace agv;
TEST(Kinematics, ReconstructAllModes){
 Controller c; Four zero{};
 for(Twist t: {Twist{1,0,0},Twist{0,1,0},Twist{.4,.3,0},Twist{0,0,.5},Twist{.2,.3,.2},Twist{-1,0,0}}){
  auto r=c.allocate(t,zero,zero);
  for(size_t i=0;i<4;++i){
   EXPECT_NEAR(r.speed[i]*std::cos(r.angle[i]),t.x-t.yaw*(i%2==0?1:-1)*.55/2,1e-9);
   EXPECT_NEAR(r.speed[i]*std::sin(r.angle[i]),t.y+t.yaw*(i<2?1:-1)*.65/2,1e-9);
  }
 }
}
TEST(Kinematics, MechanicalLimitsAndReverse){
 Controller c;Four a; a.fill(184*pi/180);
 auto r=c.allocate({1,0,0},a,a);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],pi,1e-9);EXPECT_LT(r.speed[i],0);}
 for(int deg=-180;deg<=180;++deg){double h=deg*pi/180;auto q=c.allocate({std::cos(h),std::sin(h),0},a,a);
  for(double angle:q.angle)EXPECT_LE(std::abs(angle),185*pi/180);}
}
TEST(Kinematics, ZeroHoldsAndLimits){
 Controller c;Four a{1,2,-1,-2};auto r=c.allocate({},a,a);EXPECT_EQ(r.angle,a);
 auto q=c.allocate({20,20,3},a,a);for(double v:q.speed)EXPECT_LE(std::abs(v),10/3.6+1e-9);
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
  for(size_t i=0;i<4;++i){double rate=(r.angle[i]-a[i])/.01;EXPECT_LE(std::abs(rate),1.2+1e-8);EXPECT_LE(std::abs(rate-old_rate[i]),3.0*.01+1e-8);old_rate[i]=rate;}
  a=r.angle;v=r.speed;
 }
 EXPECT_EQ(c.mode(),Mode::Drive);
}
TEST(Transitions, HighSpeedSmallAngleRequiresBraking){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<900;++n){r=c.update({5,0,0},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_NEAR(v[0],10/3.6,1e-9);  // An excessive request is capped before changing direction.
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
 Controller c;Four a; a.fill(pi);
 for(double vx:{-1.0,1.0}) {
  auto r=c.allocate({vx,0,0},a,a,true);
  for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],0,1e-9);EXPECT_NEAR(r.speed[i],vx,1e-9);}
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
  if(r.speed[0]<0)EXPECT_TRUE(stopped);
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
   if(r.speed[0]*v[0]<0)EXPECT_TRUE(zero_crossing);
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
 Controller c;Four a;a.fill(179*pi/180);
 auto r=c.allocate({.3*std::cos(181*pi/180),.3*std::sin(181*pi/180),0},a,a);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],181*pi/180,1e-9);EXPECT_GT(r.speed[i],0);}
}
TEST(Kinematics, ZeroSpeedModuleHoldsThroughNoise){
 Controller c;Four a{.7,0,0,0};
 for(double noise:{-.001,-.00001,0.,.00001,.001}) {
  auto r=c.allocate({.4*.55/2+noise,-.4*.65/2,.4},a,a);
  EXPECT_DOUBLE_EQ(r.angle[0],a[0]);EXPECT_DOUBLE_EQ(r.speed[0],0);
  EXPECT_GT(Controller::max_abs(r.speed),.1);
 }
}
TEST(Transitions, ContinuousHeadingSweepReconfiguresBeforeLimit){
 for(double direction:{-1.,1.}) {
  Controller c;Four a{},v{};Target r;
  int limit_stops=0;double maximum_angle=0;Mode old_mode=Mode::Hold;
  for(int n=0;n<7000;++n){
   const double heading=direction*.2*n*.01;
   r=c.update({.25*std::cos(heading),.25*std::sin(heading),0},a,v,.01);
   if(old_mode==Mode::Drive && c.mode()==Mode::Brake && c.reason()=="LIMIT_RECONFIGURE") {
    ++limit_stops;EXPECT_LT(Controller::max_abs(a),180*pi/180);
   }
   for(size_t i=0;i<4;++i){
    EXPECT_LE(std::abs(r.angle[i]-a[i]),1.2*.01+1e-9);
    maximum_angle=std::max(maximum_angle,std::abs(r.angle[i]));
   }
   if(c.mode()==Mode::Align)EXPECT_LT(Controller::max_abs(r.speed),.025);
   old_mode=c.mode();a=r.angle;v=r.speed;
  }
  EXPECT_GE(limit_stops,3);EXPECT_LT(maximum_angle,180*pi/180);
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
 Controller c;Four a;a.fill(170*pi/180);
 const double heading=160*pi/180;
 const auto r=c.allocate({.3*std::cos(heading),.3*std::sin(heading),0},a,a,false,true);
 for(double angle:r.angle)EXPECT_NEAR(angle,heading,1e-9);
}
TEST(Kinematics, StationaryShortestChoiceMustLeaveLimitReserve){
 Controller c;Four a;a.fill(160*pi/180);
 const auto r=c.allocate({.3,0,0},a,a,false,true);
 for(size_t i=0;i<4;++i){EXPECT_NEAR(r.angle[i],0,1e-9);EXPECT_GT(r.speed[i],0);}
}
TEST(Transitions, OneWheelCrossesZeroWithoutStoppingWholeVehicle){
 Controller c;Four a{},v{};Target r;
 for(int n=0;n<500;++n){r=c.update({.16,-.13,.4},a,v,.01);a=r.angle;v=r.speed;}
 ASSERT_EQ(c.mode(),Mode::Drive);
 for(int n=0;n<1000;++n){
  r=c.update({.16-.0001*n,-.13,.4},a,v,.01);
  EXPECT_EQ(c.mode(),Mode::Drive);
  EXPECT_NEAR(r.angle[0],0,1e-7);
  a=r.angle;v=r.speed;
 }
 EXPECT_NEAR(v[0],-.0499,.001);EXPECT_NEAR(v[1],.1701,.001);
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
