#include "mission_explorer/explorer_core.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"


#include "rclcpp/rclcpp.hpp"

using LocalScan = underground_world::msg::LocalScan;
using MoveCommand = underground_world::msg::MoveCommand;
using PayloadTrigger = underground_world::srv::PayloadTrigger;

/* ROS-обгортка:
subscription на /robot/local_scan; бажано також читає /robot/result; TODO
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

 auto MissionExplorerNode::convertScan(
    const underground_world::msg::LocalScan& message) ->mission_explorer:: ScanObservation;