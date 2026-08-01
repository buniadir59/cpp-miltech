#include <format>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <string>

#include "underground_world/state_qos.hpp"
#include <underground_world/scenario.hpp>
#include "mission_explorer/explorer_core.hpp"
#include "underground_world/msg/cell_observation.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

using LocalScan = underground_world::msg::LocalScan;
using MoveCommand = underground_world::msg::MoveCommand;
using PayloadTrigger = underground_world::srv::PayloadTrigger;

/* ROS-обгортка:
subscription на /robot/local_scan; бажано також читає /robot/result;
publisher /robot/cmd_move;
publisher /student/status;
client /payload/trigger;
перетворення ROS-повідомлень у внутрішні типи.
після кожної дії чекає нового scan.
 */
/*
отримає local scan-> конвертує для кора -> викликає кор і отримує decision
-> calls trigger if needed (waits for result?)-> move command ->next scan
 */
namespace {

using MoveCommand = underground_world::msg::MoveCommand;
using StudentStatus = underground_world::msg::StudentStatus;
using PayloadTrigger = underground_world::srv::PayloadTrigger;
using LocalScan = underground_world::msg::LocalScan;
using RobotResult = underground_world::msg::RobotResult;
using CellObservation = underground_world::msg::CellObservation;
using CellKind = underground_world::CellKind;

using Direction = mission_explorer::Direction;
using State = mission_explorer::State;
using ScanObservation = mission_explorer::ScanObservation;

constexpr auto kLocalScanTopic = "/robot/local_scan";
constexpr auto kPayloadTriggerService = "/payload/trigger";
constexpr auto kCommandMoveTopic = "/robot/cmd_move";
constexpr auto kStudentStatusTopic = "/student/status";
constexpr auto kRobotResultTopic = "/robot/result";

CellKind str_to_cellkind(const std::string& cell_type)
{
  if (cell_type.length() == 1) {
    switch (cell_type[0]) {
      case '#':
        return CellKind::Wall;
      case '.':
        return CellKind::Free;
      case 'C':
        return CellKind::Contact;
      case 'x':
        return CellKind::ProcessedContact;
      case 'S':
        return CellKind::Start;
    }
  }

  throw std::invalid_argument{"Invalid cell_type: '" + cell_type + "'"};
}

auto convert_scan(const LocalScan& msg) -> ScanObservation
{
  ScanObservation scan_obs{};
  scan_obs.robot_position = {msg.robot_x, msg.robot_y};
  scan_obs.cells.reserve(msg.cells.size());
  for (const auto& cell : msg.cells) {
    mission_explorer::ObservedCell cell_obs = {{cell.x, cell.y}, str_to_cellkind(cell.cell_type), cell.contact_id};
    scan_obs.cells.push_back(cell_obs);
  }
  return scan_obs;
}

auto to_message_direction(Direction direction)
{
  switch (direction) {
    case Direction::Up:
      return MoveCommand::UP;
    case Direction::Down:
      return MoveCommand::DOWN;
    case Direction::Left:
      return MoveCommand::LEFT;
    case Direction::Right:
      return MoveCommand::RIGHT;
  }
  throw std::invalid_argument{"Invalid direction in decision: " + std::to_string(static_cast<int>(direction))};
}

auto to_message_state(State state)
{
  switch (state) {
    case State::Exploring:
      return StudentStatus::EXPLORING;
    case State::Returning:
      return StudentStatus::RETURNING;
    case State::Engaging:
      return StudentStatus::ENGAGING;
    case State::Done:
      return StudentStatus::DONE;
    case State::Failed:
      return StudentStatus::FAILED;
  }
  throw std::invalid_argument{"Invalid state in decision: " + std::to_string(static_cast<int>(state))};
}

auto to_string_direction(Direction direction)
{
  switch (direction) {
    case Direction::Up:
      return "UP";
    case Direction::Down:
      return "DOWN";
    case Direction::Left:
      return "LEFT";
    case Direction::Right:
      return "RIGHT";
  }
  return "???";
}

auto to_string_state(State state)
{
  switch (state) {
    case State::Exploring:
      return "EXPLORING";
    case State::Returning:
      return "RETURNING";
    case State::Engaging:
      return "ENGAGING";
    case State::Done:
      return "DONE";
    case State::Failed:
      return "FAILED";
  }
  return "??";
}
}  // namespace

/* Після запуску underground_world_node один раз публікує стартові
/robot/local_scan, /robot/metrics і /robot/result, щоб система рішення могла
зробити перший крок не наосліп. Далі публікація відбувається після подій,
які змінюють стан світу: застосованої команди /robot/cmd_move або отриманої
події /payload/enemy_down. Постійного фонового потоку стану немає. */

class MissionExplorerNode final : public rclcpp::Node {
  rclcpp::Publisher<MoveCommand>::SharedPtr move_publisher_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_publisher_;
  rclcpp::Client<PayloadTrigger>::SharedPtr payload_trigger_client_;
  rclcpp::Subscription<LocalScan>::SharedPtr local_scan_subscriber_;
  rclcpp::Subscription<RobotResult>::SharedPtr robot_result_subscriber_;

  bool scenario_name_published_ = false;
  bool mission_finished_ = false;
  mission_explorer::ExplorerCore expl_core_{};

public:
  MissionExplorerNode()
    : Node("mission_explorer_node")
  {
    const auto qos = rclcpp::QoS{10};
    const auto state_qos = underground_world::make_state_qos();
    move_publisher_ = create_publisher<MoveCommand>(kCommandMoveTopic, qos);
    status_publisher_ = create_publisher<StudentStatus>(kStudentStatusTopic, qos);

    local_scan_subscriber_ =
      create_subscription<LocalScan>(kLocalScanTopic, state_qos, [this](const LocalScan::SharedPtr msg) { on_local_scan_msg(*msg); });

    robot_result_subscriber_ = create_subscription<RobotResult>(
      kRobotResultTopic, state_qos, [this](const RobotResult::SharedPtr msg) { on_robot_result_msg(*msg); });

    payload_trigger_client_ = create_client<PayloadTrigger>(kPayloadTriggerService);
  }

private:
  void on_robot_result_msg(const RobotResult& msg)
  {
    if (!scenario_name_published_) {
      scenario_name_published_ = true;
      RCLCPP_INFO(get_logger(), "Started scenario: %s", msg.scenario_name.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "RobotResult msg received: %s, steps=%d out of %d reason=%s",
                msg.mission_result.c_str(),
                msg.steps_taken,
                msg.max_steps,
                msg.reason.c_str());

    if (msg.mission_result == "SUCCESS") {
      mission_finished_ = true;
      publish_status(State::Done);
    }
  }

  void publish_status(State state)
  {
    auto msg = StudentStatus{};
    msg.state = to_message_state(state);
    status_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "State: %s", to_string_state(state));
  }

  void publish_move(Direction direction)
  {
    auto msg = MoveCommand{};
    msg.direction = to_message_direction(direction);
    move_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Move: %s", to_string_direction(direction));
  }

  void log_scan(const LocalScan& msg)
  {
    int y_saved = -1;
    std::string s = "";
    for (const auto& cell : msg.cells) {
      if (cell.y != y_saved) {
        s += '\n';
        y_saved = cell.y;
      }
      s += cell.cell_type;
    }

    RCLCPP_INFO(get_logger(), "LocalScan msg received: R(%d, %d) %s", msg.robot_x, msg.robot_y, s.c_str());
  }

  void on_local_scan_msg(const LocalScan& msg)
  {
    ScanObservation scan = convert_scan(msg);
    log_scan(msg);

    if (mission_finished_) {
      expl_core_.update_map(scan);
#ifdef PUBLISH_MAP
      RCLCPP_INFO(get_logger(), "Known map:\n%s", expl_core_.map_to_string().c_str());
#endif
      return;
    }

    mission_explorer::ExplorerDecision decision = expl_core_.processScan(scan);
    if (decision.state == State::Engaging) {
      if (!decision.contact.has_value()) {
        throw std::invalid_argument{"Engaging decision doesnt have contact"};
      }

      auto request = std::make_shared<PayloadTrigger::Request>();
      request->contact_id = decision.contact->id;
      request->x = decision.contact->position.x;
      request->y = decision.contact->position.y;
      payload_trigger_client_->async_send_request(request, [this](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "recieved response: accepted=%s, reason=%s", response->accepted ? "true" : "false", response->reason.c_str());
      });
    }
    else if (decision.state == State::Exploring || decision.state == State::Returning) {
      publish_move(decision.direction);
    }

    publish_status(decision.state);

#ifdef PUBLISH_MAP
    RCLCPP_INFO(get_logger(), "Known map:\n%s", expl_core_.map_to_string().c_str());
#endif
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExplorerNode>();
  try {
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Shutdown due to error in node: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
