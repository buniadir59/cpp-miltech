#pragma once


#include <vector>
#include <string>

/* Чистий C++ без ROS:

зберігає відомі клітинки карти;
окремо зберігає відвідані позиції;
пам’ятає батьківську клітинку для повернення;
приймає черговий LocalScan;
повертає одне рішення:
EngageContact;
Move;
Done;
Failed.
 */

 /* mission_explorer_core містить алгоритм:

карту;
DFS;
visited;
parent;
вибір наступного кроку;
внутрішні Position, Cell, Decision.
 */
namespace mission_explorer {

    enum class CellType {
  Wall,
  Free,
  Start,
  Contact,
  ProcessedContact,
};

std::string cell_type_to_string(CellType type);

    struct Position {
  int x{};
  int y{};

  auto operator<=>(const Position&) const = default;
};

struct Contact {
  int id = 0;
  Position position;
};

struct CellObservation {
  Position position;
  CellType type{};
  int contact_id{};
};

struct ScanObservation {
  Position robot_position;
  std::vector<CellObservation> cells;
};

struct ExplorerDecision {
  Position new_position;
  Position enemy;
  bool action;
};

class ExplorerCore { 
public:
  [[nodiscard]] auto processScan(const ScanObservation& scan)
      -> ExplorerDecision;
};
}