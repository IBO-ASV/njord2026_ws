#include <gtest/gtest.h>
#include "thruster_controller/node.hpp"

TEST(ThrusterStartup, DeclaresAllStartupParametersBeforeInstallingLiveValidation)
{
  if(!rclcpp::ok())rclcpp::init(0,nullptr);
  std::string urdf="<robot>";
  for(const std::string id:{"FR","FL","RR","RL"})
    urdf += "<joint name='"+id+"'><parent link='base_link'/><child link='thruster_"+id+"'/><origin xyz='0.2 0.25 0'/></joint>";
  urdf += "</robot>";
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("robot_description",urdf),rclcpp::Parameter("duty_resolution",1000),
    rclcpp::Parameter("control.i.surge",55.0),rclcpp::Parameter("control.reference_tau_sec",.15)});
  auto node=std::make_shared<njord::thruster_controller::ThrusterControllerNode>(options);
  EXPECT_EQ(node->get_parameter("duty_resolution").as_int(),1000);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("control.p.surge",40.0)).successful);

  // These parameters are changed by the velocity autotuner while it runs.
  // Keep the dynamic allowlist and the controller's actual update path aligned.
  auto tuning_update = node->set_parameters_atomically({
    rclcpp::Parameter("control.ff.surge",12.0),
    rclcpp::Parameter("control.d.surge",3.0),
    rclcpp::Parameter("control.d.filter_tau_sec",0.25),
    rclcpp::Parameter("safety.max_thruster_thrust_n",20.0)});
  EXPECT_TRUE(tuning_update.successful) << tuning_update.reason;
  EXPECT_DOUBLE_EQ(node->get_parameter("control.ff.surge").as_double(),12.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("control.d.surge").as_double(),3.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("control.d.filter_tau_sec").as_double(),0.25);
  EXPECT_DOUBLE_EQ(node->get_parameter("safety.max_thruster_thrust_n").as_double(),20.0);

  auto over_limit = node->set_parameter(
    rclcpp::Parameter("safety.max_thruster_thrust_n",41.0));
  EXPECT_FALSE(over_limit.successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("safety.max_thruster_thrust_n").as_double(),20.0);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("duty_resolution",500)).successful);
  node.reset();rclcpp::shutdown();
}
