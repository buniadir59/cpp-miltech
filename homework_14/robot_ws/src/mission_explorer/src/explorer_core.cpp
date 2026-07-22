#include "mission_explorer/explorer_core.hpp"


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

  [[nodiscard]] auto ExplorerCore::processScan(const ScanObservation& scan)
      -> ExplorerDecision {
        return ExplorerDecision{};
      }
}

