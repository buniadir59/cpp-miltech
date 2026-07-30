#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/ros_names.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

/* gimbal_driver_node відповідає за вертикальне наведення -
піднімає, опускає або втримує лінію наведення.

Нода підписується на /gimbal/cmd topic, отримує GimbalCommand: UP, DOWN, CENTER;
і логує її

gimbal_driver_node отримав: direction=UP target_y=180 error_y=60
*/

namespace {

const char* gimbal_direction_to_str(int8_t direction)
{
  switch (direction) {
    case -1:
      return "DOWN";
    case 0:
      return "CENTER";
    case 1:
      return "UP";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class GimbalDriverNode final : public rclcpp::Node {
public:
  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    subscription_ = create_subscription<antidrone_turret::msg::GimbalCommand>(
      antidrone_turret::ros_names::kGimbalCommandTopic, 10, [this](const antidrone_turret::msg::GimbalCommand& gimbal_cmd) {
        on_gimbal_cmd(gimbal_cmd);
      });
      
    RCLCPP_INFO(get_logger(), "subscribed to %s", antidrone_turret::ros_names::kGimbalCommandTopic);
  };

private:
  void on_gimbal_cmd(antidrone_turret::msg::GimbalCommand cmd)
  {
    RCLCPP_INFO(get_logger(),  // gimbal_driver_node отримав: direction=UP target_y=180 error_y=60
                "received command: direction=%s, target_y=%.1f, error_y=%.1f",
                gimbal_direction_to_str(cmd.direction),
                cmd.target_y,
                cmd.error_y);
  }

  rclcpp::Subscription<antidrone_turret::msg::GimbalCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
