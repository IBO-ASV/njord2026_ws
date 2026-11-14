#include <gtest/gtest.h>
#include <limits>
#include "thruster_driver/velocity_control.hpp"
using njord::thruster_driver::VelocityControl;

TEST(VelocityControl, DefaultsPreserveIndependentProportionalAxes)
{
  VelocityControl c;
  const auto output=c.step({1.,-.4,.2},{.5,-.1,.1},.02,{20.,30.,10.},{0.,0.,0.},{100.,100.,50.});
  EXPECT_NEAR(output[0],10.,1e-9); EXPECT_NEAR(output[1],-9.,1e-9); EXPECT_NEAR(output[2],1.,1e-9);
}
TEST(VelocityControl, RejectsNonfiniteVelocity)
{
  VelocityControl c;
  EXPECT_THROW(c.step({std::numeric_limits<double>::quiet_NaN(),0,0},{0,0,0},.02,
    {1,1,1},{0,0,0},{10,10,10}),std::invalid_argument);
}
TEST(VelocityControl, SaturationDoesNotAccumulateAnUnboundedIntegral)
{
  VelocityControl c;
  for(int i=0;i<500;++i)c.step({10,0,0},{0,0,0},.02,{20,20,20},{10,10,10},{2,2,2});
  auto output=c.step({0,0,0},{0,0,0},.02,{20,20,20},{10,10,10},{2,2,2});
  EXPECT_NEAR(output[0],0.,1e-9);
}
TEST(VelocityControl, IntegralRemovesConstantDragAndResetRemovesStoredForce)
{
  VelocityControl c; double velocity=0;
  for(int i=0;i<2000;++i) {
    auto output=c.step({1,0,0},{velocity,0,0},.02,{10,10,10},{5,5,5},{30,30,30});
    velocity += .02*(output[0]-4*velocity)/10.;
  }
  EXPECT_NEAR(velocity,1.,.001);
  c.reset();
  EXPECT_DOUBLE_EQ(c.step({0,0,0},{0,0,0},.02,{10,10,10},{5,5,5},{30,30,30})[0],0.);
}
TEST(VelocityControl, ReferenceFilterLimitsInitialStepAndResets)
{
  VelocityControl c;
  auto output=c.step({1,0,0},{0,0,0},.02,{10,10,10},{0,0,0},{100,100,100},.15);
  EXPECT_GT(output[0],0.); EXPECT_LT(output[0],2.);
  c.reset();
  EXPECT_DOUBLE_EQ(c.step({0,0,0},{0,0,0},.02,{10,10,10},{0,0,0},{100,100,100},.15)[0],0.);
}
