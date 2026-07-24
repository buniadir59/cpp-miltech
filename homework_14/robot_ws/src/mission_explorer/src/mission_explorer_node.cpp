#include "mission_explorer/explorer_core.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"


#include "rclcpp/rclcpp.hpp"

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

constexpr auto kLocalScanTopic = "/robot/local_scan";
constexpr auto kPayloadTriggerService = "/payload/trigger";
constexpr auto kCommandMoveTopic = "/robot/cmd_move";
constexpr auto kStudentStatusTopic = "/student/status";
constexpr auto kRobotResultTopic = "/robot/result";

}  // namespace

/* Після запуску underground_world_node один раз публікує стартові 
/robot/local_scan, /robot/metrics і /robot/result, щоб система рішення могла 
зробити перший крок не наосліп. Далі публікація відбувається після подій, 
які змінюють стан світу: застосованої команди /robot/cmd_move або отриманої 
події /payload/enemy_down. Постійного фонового потоку стану немає. */
   using MoveCommand = underground_world::msg::MoveCommand;  
using StudentStatus = underground_world::msg::StudentStatus;
using PayloadTrigger = underground_world::srv::PayloadTrigger;
using LocalScan = underground_world::msg::LocalScan;
using RobotResult = underground_world::msg::RobotResult;

class MissionExplorerNode final : public rclcpp::Node {
   rclcpp::Publisher<MoveCommand>::SharedPtr cmd_move_publisher_;
rclcpp::Publisher<StudentStatus>::SharedPtr student_status_publisher_;
rclcpp::Client<PayloadTrigger>::SharedPtr payload_trier_client_;
rclcpp::Subscription<LocalScan>::SharedPtr local_scan_subscriber_;
rclcpp::Subscription<RobotResult>::SharedPtr robot_result_subscriber_;

   public:
   MissionExplorerNode() : Node("mission_explorer_node") {

   cmd_move_publisher_ = create_publisher<MoveCommand>(kCommandMoveTopic, 10); 
   student_status_publisher_ = create_publisher<StudentStatus>(kStudentStatusTopic, 10);

//publisher /student/status;
//client /payload/trigger;
//subscription на /robot/local_scan; бажано також читає /robot/result;
   }

};

/*  auto MissionExplorerNode::convertScan(
    const underground_world::msg::LocalScan& message) ->mission_explorer:: ScanObservation; */