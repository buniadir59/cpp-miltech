#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/turret_controller_logic.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/ros_names.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <optional>
#include <string>
#include <chrono>

// subcribe to:
//      /actuator/status
//      /perception/target
// publish to:
//      /servo/cmd
//      /gimbal/cmd topics
//      /turret/status message
// call
//      /actuator/trigger service

/* контролер спочатку вирішує, куди навести турель, а команду пострілу надсилає окремо -
 тоді, коли ціль достатньо близько, розпізнавання достатньо надійне, а актуатор  READY */

/* callback для /perception/target має виконувати приблизно таку послідовність:
ROS Target
   ↓ конвертація
чистий C++ TargetObservation
   ↓
turret controller logic
   ↓
TurretDecision
   ├── publish TurretStatus завжди
   ├── publish GimbalCommand, якщо ACTION_TRACK
   ├── publish ServoCommand, якщо ACTION_TRACK
   └── async service request, якщо TRIGGER_REQUESTED
    */

namespace {

std::uint8_t to_message_trigger_state(const antidrone_turret::TriggerDecision decision)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TriggerDecision = antidrone_turret::TriggerDecision;

  return decision == TriggerDecision::kTriggerRequested
           ? TurretStatus::TRIGGER_REQUESTED
           : (decision == TriggerDecision::kTriggerReloading ? TurretStatus::TRIGGER_RELOADING : TurretStatus::TRIGGER_SKIP);
}

std::uint8_t to_message_target_state(const antidrone_turret::TargetState state)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using TargetState = antidrone_turret::TargetState;
  return state == TargetState::kTargetLocked
           ? TurretStatus::TARGET_LOCKED
           : (state == TargetState::kTargetLowConfidence ? TurretStatus::TARGET_LOW_CONFIDENCE : TurretStatus::TARGET_NONE);
}

std::uint8_t to_message_gimbal_direction(const antidrone_turret::GimbalDirection dir)
{
  using GimbalCommand = antidrone_turret::msg::GimbalCommand;
  using GimbalDirection = antidrone_turret::GimbalDirection;
  return dir == GimbalDirection::kDown ? GimbalCommand::DOWN : (dir == GimbalDirection::kUp ? GimbalCommand::UP : GimbalCommand::CENTER);
}

std::uint8_t to_message_servo_direction(const antidrone_turret::ServoDirection dir)
{
  using ServoCommand = antidrone_turret::msg::ServoCommand;
  using ServoDirection = antidrone_turret::ServoDirection;
  return dir == ServoDirection::kLeft ? ServoCommand::LEFT : (dir == ServoDirection::kRight ? ServoCommand::RIGHT : ServoCommand::CENTER);
}

std::uint8_t to_message_action(const antidrone_turret::Action action)
{
  using TurretStatus = antidrone_turret::msg::TurretStatus;
  using Action = antidrone_turret::Action;
  return action == Action::kActionTrack ? TurretStatus::ACTION_TRACK : TurretStatus::ACTION_IDLE;
}

auto to_message_actuator_state(const antidrone_turret::msg::ActuatorStatus& msg) -> antidrone_turret::ActuatorState
{
  if (msg.state == antidrone_turret::msg::ActuatorStatus::READY) {
    return antidrone_turret::ActuatorState::kReady;
  }

  return antidrone_turret::ActuatorState::kReloading;
}
}  // namespace

class TurretControlNode final : public rclcpp::Node {
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuator_status_subscription_;
  rclcpp::Subscription<antidrone_turret::msg::Target>::SharedPtr target_subscription_;
  rclcpp::Publisher<antidrone_turret::msg::TurretStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Publisher<antidrone_turret::msg::ServoCommand>::SharedPtr servo_cmd_publisher_;
  rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedPtr actuator_client;

  antidrone_turret::ControllerConfig config_;
  bool trigger_request_pending_{false};
  std::optional<antidrone_turret::ActuatorState> actuator_state;

public:
  TurretControlNode()
    : Node("turret_control_node")
  {
    config_.confidence_threshold = declare_parameter<double>("confidence_threshold", 0.8);
    config_.max_distance_m = declare_parameter<double>("max_distance_m", 30.0);

    status_publisher_ = create_publisher<antidrone_turret::msg::TurretStatus>(antidrone_turret::ros_names::kTurretStatusTopic, 10);
    gimbal_cmd_publisher_ = create_publisher<antidrone_turret::msg::GimbalCommand>(antidrone_turret::ros_names::kGimbalCommandTopic, 10);
    servo_cmd_publisher_ = create_publisher<antidrone_turret::msg::ServoCommand>(antidrone_turret::ros_names::kServoCommandTopic, 10);

    actuator_client = create_client<antidrone_turret::srv::TriggerActuator>(antidrone_turret::ros_names::kTriggerService);

    actuator_status_subscription_ = create_subscription<antidrone_turret::msg::ActuatorStatus>(
      antidrone_turret::ros_names::kActuatorStatusTopic, 10, [this](const antidrone_turret::msg::ActuatorStatus& status) {
        on_actuator_status(status);
      });

    target_subscription_ = create_subscription<antidrone_turret::msg::Target>(
      antidrone_turret::ros_names::kTargetTopic, 10, [this](const antidrone_turret::msg::Target& target) { on_target_perception(target); });
    RCLCPP_INFO(get_logger(),
                "publishing %s %s %s and subscribed to %s %s",
                antidrone_turret::ros_names::kTurretStatusTopic,
                antidrone_turret::ros_names::kGimbalCommandTopic,
                antidrone_turret::ros_names::kServoCommandTopic,
                antidrone_turret::ros_names::kTargetTopic,
                antidrone_turret::ros_names::kActuatorStatusTopic);
  }

private:
  void on_actuator_status(const antidrone_turret::msg::ActuatorStatus& status_msg)
  {
    actuator_state = to_message_actuator_state(status_msg);
  }

  void publish_gimbal(antidrone_turret::GimbalCmd& cmd)
  {
    auto msg = antidrone_turret::msg::GimbalCommand{};
    msg.target_y = cmd.target_y;
    msg.error_y = cmd.error_y;
    msg.direction = to_message_gimbal_direction(cmd.direction);
    gimbal_cmd_publisher_->publish(msg);
  }

  void publish_servo(antidrone_turret::ServoCmd& cmd)
  {
    auto msg = antidrone_turret::msg::ServoCommand{};
    msg.target_x = cmd.target_x;
    msg.error_x = cmd.error_x;
    msg.direction = to_message_servo_direction(cmd.direction);
    servo_cmd_publisher_->publish(msg);
  }

  void publish_status(antidrone_turret::TurretDecision& decision)
  {
    auto msg = antidrone_turret::msg::TurretStatus{};
    msg.target_state = to_message_target_state(decision.target_state);
    msg.action = to_message_action(decision.action);
    msg.trigger_state = to_message_trigger_state(decision.trigger_decision);
    msg.confidence = decision.confidence;
    msg.distance_m = decision.distance_m;
  }

  /*
callback  /perception/target :
 - converts ROS Target =>   TargetObservation
  - calls turret controller logic => decision
  - based on decision
    ├── publish GimbalCommand & ServoCommand, if ACTION_TRACK
    └── async service request, if TRIGGER_REQUESTED
  - publish Turret Status always
    */
  void on_target_perception(const antidrone_turret::msg::Target& target)
  {
    // ROS Target =>  C++ logic TargetObservation
    const antidrone_turret::TargetObservation tgt{target.visible, target.x, target.y, target.distance_m, target.confidence};

    // call C++ logic
    antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, actuator_state, config_);

    if (decision.action == antidrone_turret::Action::kActionTrack) {
      //`ACTION_TRACK` - публікувати `GimbalCommand` і `ServoCommand`.
      publish_gimbal(decision.gimbal);
      publish_servo(decision.servo);
    }
    using namespace std::chrono_literals;  // Дозволяє використовувати суфікси s, ms, ns
    if (decision.trigger_decision == antidrone_turret::TriggerDecision::kTriggerRequested) {
      // not to repeat async request if previous is pending
      if (!trigger_request_pending_) {
        /*         if (!actuator_client->wait_for_service(3s)) {
                  RCLCPP_ERROR(get_logger(), "service %s is not available", antidrone_turret::ros_names::kTriggerService);
                  rclcpp::shutdown();
                  return;  // TODO ??
                } */
        trigger_request_pending_ = true;
        // call `/actuator/trigger`
        auto request = std::make_shared<antidrone_turret::srv::TriggerActuator::Request>();
        request->confidence = decision.confidence;
        request->distance_m = decision.distance_m;

        actuator_client->async_send_request(request, [this](rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedFuture future) {
          const auto response = future.get();
          RCLCPP_INFO(get_logger(), "accepted=%s trigger_count=%u", response->accepted ? "true" : "false", response->trigger_count);
          trigger_request_pending_ = false;
          rclcpp::shutdown();
        });
      }
    }

    publish_status(decision);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControlNode>());
  rclcpp::shutdown();
  return 0;
}