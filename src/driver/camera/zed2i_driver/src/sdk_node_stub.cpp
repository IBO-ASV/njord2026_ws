#include <rclcpp/rclcpp.hpp>

#include <chrono>

namespace zed2i_driver
{

class SdkNode : public rclcpp::Node
{
public:
  explicit SdkNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_sdk_node", options)
  {
    const bool enable_gpu_perception = declare_parameter<bool>("enable_gpu_perception", false);
    if (enable_gpu_perception) {
      RCLCPP_FATAL(
        get_logger(),
        "enable_gpu_perception=true requires a zed2i_driver build with the ZED SDK, CUDA, and "
        "TensorRT. This build contains only the SDK stub.");
    }
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

}  // namespace zed2i_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed2i_driver::SdkNode)
