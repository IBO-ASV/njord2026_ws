#include <rclcpp/rclcpp.hpp>

#include <chrono>

class Zed2iSdkStubNode : public rclcpp::Node
{
public:
  explicit Zed2iSdkStubNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_sdk_node", options)
  {
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        RCLCPP_FATAL(
          get_logger(),
          "mode:=sdk was requested, but the ZED SDK CMake package was not found at build time. "
          "Install the Stereolabs ZED SDK and rebuild this workspace, or launch mode:=cpu.");
        rclcpp::shutdown();
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zed2iSdkStubNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 1;
}
