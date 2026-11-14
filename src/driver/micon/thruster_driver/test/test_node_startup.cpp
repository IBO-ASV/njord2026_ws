#include <gtest/gtest.h>
#include "thruster_driver/node.hpp"

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
  auto node=std::make_shared<njord::thruster_driver::ThrusterDriverNode>(options);
  EXPECT_EQ(node->get_parameter("duty_resolution").as_int(),1000);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("control.p.surge",40.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("duty_resolution",500)).successful);
  node.reset();rclcpp::shutdown();
}
