#pragma once

#include "underground_world/scenario.hpp"

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

/* Чистий C++ без ROS, вмкористовує алгоритм DFS
зберігає відомі клітинки карти;
окремо зберігає відвідані позиції;
пам’ятає шлях для повернення;
приймає LocalScan;
повертає одне рішення із станом
 */

namespace mission_explorer {

using CellKind = underground_world::CellKind;
using ContactUW = underground_world::Contact;

enum class Direction { Up, Down, Left, Right };

enum class State { Exploring, Engaging, Returning, Done, Failed };

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

  bool initialized_ = false;

public:
  [[nodiscard]] auto processScan(const ScanObservation& scan) -> ExplorerDecision;

#define PUBLISH_MAP
#ifdef PUBLISH_MAP

  auto map_to_string() const -> std::string
  {
    if (known_map_.empty()) {
      return {};
    }

    auto min_x = known_map_.begin()->first.x;
    auto max_x = min_x;
    auto min_y = known_map_.begin()->first.y;
    auto max_y = min_y;

    for (const auto& entry : known_map_) {
      const auto& point = entry.first;  // to avoid warning

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
#endif

  void update_map(const ScanObservation& scan);

private:
  [[nodiscard]] auto find_unvisited_neighbor(const Point& robot_position) const -> std::optional<Point>;

  [[nodiscard]] static auto find_visible_contact(const ScanObservation& scan) -> std::optional<underground_world::Contact>
  {
    for (const auto& cell : scan.cells) {
      if (cell.kind == CellKind::Contact) {
        // Contact is visible in this local scan.
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
    if (delta == Point{0, 1}) {
      return Direction::Down;
    }
    if (delta == Point{0, -1}) {
      return Direction::Up;
    }

    if (delta == Point{-1, 0}) {
      return Direction::Left;
    }
    if (delta == Point{1, 0}) {
      return Direction::Right;
    }

    throw std::invalid_argument{"Invalid coordinates in get_direction function: dy=" + std::to_string((delta.y))};
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