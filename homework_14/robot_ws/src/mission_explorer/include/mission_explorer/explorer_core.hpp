#pragma once

#include "underground_world/scenario.hpp"

//#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

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

using CellKind = underground_world::CellKind;
using ContactUW = underground_world::Contact;

namespace mission_explorer {

enum class Direction { Up, Down, Left, Right };

enum class State { Exploring, Engaging, Returning, Done, Failed };
/* enum class DecisionKind {
  Move,
  Trigger,
  Wait,
  Done,
  Failed,
};
struct Contact {
  int id = 0;
  Position position;
};
struct Decision {
  DecisionKind kind = DecisionKind::Wait;
  std::optional<std::uint8_t> direction;
  std::optional<underground_world::Contact> contact;
  std::string reason;
}; */
struct Point {
  int x = 0;
  int y = 0;

  auto operator-(const Point& other) const { return Point{x - other.x, y - other.y}; }
  auto operator<=>(const Point&) const = default;
};

struct ObservedCell {
  Point position;
  CellKind kind = CellKind::Wall;
  int contact_id = 0;
};

struct ScanObservation {  // skipped scenario name
  Point robot_position;
  std::vector<ObservedCell> cells;
};

struct ExplorerDecision {
  State state = State::Exploring;
  Direction direction = Direction::Up;
  std::optional<underground_world::Contact> contact;
};

class ExplorerCore {
  std::map<Point, CellKind> known_map_;
  std::set<Point> visited_;

  // Шлях DFS від старту до поточної клітинки.
  std::vector<Point> dfs_path_;  //  parent list

  // Захист від повторного виклику trigger до нового scan. //TODO if needed?
  std::optional<int> pending_contact_id_;

  bool initialized_ = false;

public:  //
  [[nodiscard]] auto processScan(const ScanObservation& scan) -> ExplorerDecision;
  // TODO  void on_trigger_response(int contact_id, bool accepted);
  auto map_to_string() -> std::string
  {
    if (known_map_.empty()) {
      return {};
    }

    auto min_x = known_map_.begin()->first.x;
    auto max_x = min_x;
    auto min_y = known_map_.begin()->first.y;
    auto max_y = min_y;

    for (const auto& entry : known_map_) {
  const auto& point = entry.first; //to avoid warning

  min_x = std::min(min_x, point.x);
  max_x = std::max(max_x, point.x);
  min_y = std::min(min_y, point.y);
  max_y = std::max(max_y, point.y);
}

    std::string result;

    for (auto y = min_y; y <= max_y; ++y) {
      for (auto x = min_x; x <= max_x; ++x) {
        const auto it = known_map_.find(Point{x, y});

        result += it != known_map_.end() ? kind_to_char(it->second) : '?';
      }

      if (y != max_y) {
        result += '\n';
      }
    }

    return result;
  }


private:
  void update_map(const ScanObservation& scan);

  //[[nodiscard]] auto find_unvisited_neighbor(Point current) const -> std::optional<Point>;

  [[nodiscard]] auto find_unvisited_neighbor(const ScanObservation& scan) const -> std::optional<Point>;

  [[nodiscard]] static auto find_visible_contact(const ScanObservation& scan)  // const
    -> std::optional<underground_world::Contact>
  {
    for (const auto& cell : scan.cells) {
      if (cell.kind == CellKind::Contact) {
        // Contact is visible in this local scan.
        // TODO pending_contact_id_ = cell.contact_id;
        ContactUW c = {cell.contact_id, {cell.position.x, cell.position.y}};
        return c;
      }
    }
    return std::nullopt;
  }

  [[nodiscard]] static auto is_passable(CellKind kind) -> bool
  {
    return kind == CellKind::Free || kind == CellKind::ProcessedContact || kind == CellKind::Start;
  }

  [[nodiscard]] static auto get_direction(const Point& from, const Point& to) -> Direction
  {
    Point delta = to - from;
    if (delta.x == 0) {
      if (delta.y == 1) {
        return Direction::Down;
      }
      else if (delta.y == -1) {
        return Direction::Up;
      }
      else {
        throw std::invalid_argument{"Invalid coordinates in get_direction function: dy=" + std::to_string((delta.y))};
      }
    }
    else if (delta.x == 1) {
      return Direction::Right;
    }
    else if (delta.x == -1) {
      return Direction::Left;
    }
    throw std::invalid_argument{"Invalid coordinates in get_direction function: dx=" + std::to_string((delta.x))};
  }

  static auto kind_to_char(CellKind kind) -> char
  {
    switch (kind) {
      case CellKind::Wall:
        return '#';
      case CellKind::Free:
        return '_';
      case CellKind::Start:
        return 'S';
      case CellKind::Contact:
        return 'C';
      case CellKind::ProcessedContact:
        return 'x';
    }

    return '?';
  }

};

}  // namespace mission_explorer