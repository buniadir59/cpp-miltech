/*
payload_action_node
надає /payload/trigger;
після прийнятого запиту публікує /payload/enemy_down;
повертає accepted=true.
*/

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/scenario.hpp"

namespace {

using EnemyDown = underground_world::msg::EnemyDown;
using PayloadTrigger = underground_world::srv::PayloadTrigger;

constexpr auto kEnemyDownTopic = "/payload/enemy_down";
constexpr auto kPayloadTriggerService = "/payload/trigger";

}  // namespace

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("actuator_node")
  {
    enemy_down_publisher_ = create_publisher<EnemyDown>(kEnemyDownTopic, 10);
    payload_trigger_service_ =
      create_service<PayloadTrigger>(kPayloadTriggerService,
                                     [this](const std::shared_ptr<PayloadTrigger::Request> request,
                                            std::shared_ptr<PayloadTrigger::Response> response) { on_trigger(request, response); });

    RCLCPP_INFO(get_logger(), "serving %s and publishing %s", kPayloadTriggerService, kEnemyDownTopic);
  }

private:

  auto publish_enemy_down(const underground_world::Contact& contact) const
  {
    auto msg = EnemyDown{};
    msg.contact_id = contact.id;
    msg.x = contact.position.x;
    msg.y = contact.position.y;

    enemy_down_publisher_->publish(msg);
  }

  void on_trigger(const std::shared_ptr<PayloadTrigger::Request>& request, const std::shared_ptr<PayloadTrigger::Response>& response)
  {
    underground_world::Contact contact{request->contact_id, {request->x, request->y}};
    std::string reason = "Contact processed id#" + std::to_string(request->contact_id);
    response->accepted = true;
    response->reason = reason;

    RCLCPP_INFO(get_logger(), "trigger accepted reason=%s", reason.c_str());

    publish_enemy_down(contact);
  }


  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_publisher_;
  rclcpp::Service<PayloadTrigger>::SharedPtr payload_trigger_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
