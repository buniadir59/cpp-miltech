#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/ros_names.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

/*
yaw_servo_driver_node - це сервопривід повороту навколо вертикальної осі.
Нода отримує ServoCommand з LEFT, RIGHT, CENTER
і логує її
yaw_servo_driver_node отримав: direction=RIGHT target_x=420 error_x=100
*/

// subscribe to /servo/cmd topic

namespace {

const char* servo_direction_to_str(int8_t direction)
{
  switch (direction) {
    case -1:
      return "LEFT";
    case 0:
      return "CENTER";
    case 1:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class YawServoDriverNode final : public rclcpp::Node {
public:
  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    subscription_ = create_subscription<antidrone_turret::msg::ServoCommand>(
      antidrone_turret::ros_names::kServoCommandTopic, 10, [this](const antidrone_turret::msg::ServoCommand& servo_cmd) {
        on_servo_cmd(servo_cmd);
      });

    RCLCPP_INFO(get_logger(), "subscribed to %s", antidrone_turret::ros_names::kServoCommandTopic);
  };

private:
  void on_servo_cmd(antidrone_turret::msg::ServoCommand cmd)
  {
    RCLCPP_INFO(get_logger(),  // gimbal_driver_node отримав: direction=UP target_y=180 error_y=60
                "received command: direction=%s target_y=%.1f error_y=%.1f",
                servo_direction_to_str(cmd.direction),
                cmd.target_x,
                cmd.error_x);
  }

  rclcpp::Subscription<antidrone_turret::msg::ServoCommand>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
