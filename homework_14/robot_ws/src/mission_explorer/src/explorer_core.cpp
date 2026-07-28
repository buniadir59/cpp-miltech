#include "mission_explorer/explorer_core.hpp"

#include <optional>
// #include <stdexcept>
// #include <string>
#include <underground_world/scenario.hpp>

/* mission_explorer_core містить алгоритм:

карту;
DFS;
visited;
parent;
вибір наступного кроку;
внутрішні Position, Cell, Decision.

Алгоритм

Найнадійніший варіант тут — звичайний DFS із поверненням по батьківських клітинках.

Після кожного scan:

Оновити локальну карту.
Якщо видно C — обробити один контакт і не рухатися.
Інакше знайти сусідню відому прохідну клітинку, яку робот ще не відвідував.
Перейти туди й запам’ятати поточну клітинку як її parent.
Якщо невідвіданих сусідів немає — повернутися в parent.
Якщо parent немає і нових клітинок немає — дослідження завершене.

 */
namespace mission_explorer {

void ExplorerCore::update_map(const ScanObservation& scan)
{
  for (const auto& cell : scan.cells) {
    known_map_.insert_or_assign(cell.position, cell.kind);
  }
}

[[nodiscard]] auto ExplorerCore::find_unvisited_neighbor(const ScanObservation& scan) const -> std::optional<Point>
{
  for (const auto& cell : scan.cells) {
    if (is_passable(cell.kind) && !visited_.contains(cell.position)) {
      return cell.position;
    }
  }
  return std::nullopt;
}

/* Оновити known_map_.
Позначити позицію робота як відвідану.
Якщо очікується підтвердження контакту — повернути Wait.
Якщо видно C — повернути Trigger.
Якщо є невідвіданий прохідний сусід — додати його в dfs_path_ і повернути Move.
Інакше повернутися до попередньої позиції у dfs_path_.
Якщо повертатися вже нікуди — повернути Done. */
[[nodiscard]] auto ExplorerCore::processScan(const ScanObservation& scan) -> ExplorerDecision
{
  ExplorerDecision decision{};
  decision.contact = find_visible_contact(scan);

  if (decision.contact.has_value()) {
    decision.state = State::Engaging;  // TODO is it really possible to fire 2 times?
    return decision;
  }

  update_map(scan);
  visited_.insert(scan.robot_position);
  std::optional<Point> next = find_unvisited_neighbor(scan);
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
