#include <gtest/gtest.h>

#include "status_lights/heartbeat_monitor.hpp"

TEST(HeartbeatMonitor, TracksAgeAndTimeout)
{
  status_lights::HeartbeatMonitor monitor;
  const rclcpp::Time start(1000000000LL, RCL_ROS_TIME);
  EXPECT_FALSE(monitor.isAlive("driver", start, 1.0));
  monitor.update("driver", start);
  EXPECT_TRUE(monitor.isAlive("driver", start, 1.0));
  EXPECT_TRUE(monitor.isAlive("driver", rclcpp::Time(1900000000LL, RCL_ROS_TIME), 1.0));
  EXPECT_FALSE(monitor.isAlive("driver", rclcpp::Time(2100000000LL, RCL_ROS_TIME), 1.0));
}
