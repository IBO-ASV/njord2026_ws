#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class DynamicGoalCheckerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(DynamicGoalCheckerTest, ProjectedGoalsAndTolerancesChangeAfterConfigure)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dynamic_goal_checker_test");
  auto checker = loader.createSharedInstance("robot::SelectiveHeadingGoalChecker");
  checker->initialize(node, "checker", nullptr);
  geometry_msgs::msg::Pose query, goal, tolerance;
  geometry_msgs::msg::Twist velocity, velocity_tolerance;
  goal.orientation.w = 1.0;
  query.orientation.z = std::sin(0.2);
  query.orientation.w = std::cos(0.2);
  ASSERT_TRUE(checker->isGoalReached(query, goal, velocity));  // Position-only initially.
  node->set_parameters({
    rclcpp::Parameter("checker.heading_required_goal_xs", std::vector<double>{0.0}),
    rclcpp::Parameter("checker.heading_required_goal_ys", std::vector<double>{0.0}),
    rclcpp::Parameter("checker.yaw_goal_tolerance", 0.0872664626)});
  EXPECT_FALSE(checker->isGoalReached(query, goal, velocity));
  ASSERT_TRUE(checker->getTolerances(tolerance, velocity_tolerance));
  EXPECT_DOUBLE_EQ(tolerance.orientation.w, 0.0872664626);
  query.orientation.z = 0.0;
  query.orientation.w = 1.0;
  EXPECT_TRUE(checker->isGoalReached(query, goal, velocity));
  // SetParameters can briefly expose unequal arrays; never accept in between.
  node->set_parameter(rclcpp::Parameter("checker.heading_required_goal_xs", std::vector<double>{0.0, 2.0}));
  EXPECT_FALSE(checker->isGoalReached(query, goal, velocity));
}
