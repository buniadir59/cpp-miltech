#include "mission_explorer/explorer_core.hpp"

#include <optional>
#include <format>
#include <stdexcept>
#include <underground_world/scenario.hpp>

/* mission_explorer_core містить алгоритм DFS із поверненням по батьківських клітинках
карту;
visited;
path;
вибір дії
 */

namespace mission_explorer {

void ExplorerCore::update_map(const ScanObservation& scan)
{
  for (const auto& cell : scan.cells) {
    known_map_.insert_or_assign(cell.position, cell.kind);
  }
}

[[nodiscard]] auto ExplorerCore::find_unvisited_neighbor(const Point& robot_position) const -> std::optional<Point>
{
  const auto try_neighbor = [this](const Point& next) -> std::optional<Point> {
    const auto it = known_map_.find(next);

    if (it != known_map_.end() && is_passable(it->second) && !visited_.contains(next)) {
      return next;
    }

    return std::nullopt;
  };

  std::optional<Point> unvisited{};
  unvisited = try_neighbor({robot_position.x - 1, robot_position.y});  // left
  if (unvisited.has_value())
    return unvisited;

  unvisited = try_neighbor({robot_position.x, robot_position.y - 1});  // down
  if (unvisited.has_value())
    return unvisited;

  unvisited = try_neighbor({robot_position.x, robot_position.y + 1});  // down
  if (unvisited.has_value())
    return unvisited;

  unvisited = try_neighbor({robot_position.x + 1, robot_position.y});  // right
  if (unvisited.has_value())
    return unvisited;

  return std::nullopt;
}

[[nodiscard]] auto ExplorerCore::processScan(const ScanObservation& scan) -> ExplorerDecision
{
  if (!initialized_) {  // first scan
    dfs_path_.push_back(scan.robot_position);
    initialized_ = true;
  }

  ExplorerDecision decision{};
  decision.contact = find_visible_contact(scan);

  if (decision.contact.has_value()) {
    decision.state = State::Engaging; 
    return decision;
  }

  if (!dfs_path_.empty() && (decision.state == State::Exploring || decision.state == State::Returning)) {
    Point expected = dfs_path_.back();
    if (scan.robot_position != expected) {
      throw std::runtime_error{std::format(
        "Robot position ({}, {}) differs from expected ({}, {})", scan.robot_position.x, scan.robot_position.y, expected.x, expected.y)};
    }
  }

  update_map(scan);
  visited_.insert(scan.robot_position);
  std::optional<Point> next = find_unvisited_neighbor(scan.robot_position);
  if (next.has_value()) {
    dfs_path_.push_back(next.value());

    decision.state = State::Exploring;
    decision.direction = get_direction(scan.robot_position, next.value());
    return decision;
  }

  dfs_path_.pop_back();

  if (!dfs_path_.empty()) {
    Point previous = dfs_path_.back();
    decision.direction = get_direction(scan.robot_position, previous);
    decision.state = State::Returning;
    return decision;
  }

  decision.state = known_map_.at(scan.robot_position) == CellKind::Start ? State::Done : State::Failed;
  return decision;
}

}  // namespace mission_explorer
