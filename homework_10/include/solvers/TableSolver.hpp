#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include "dto/BallisticsInput.hpp"
#include "BallisticTable.hpp"

/* Required:
  Ballistic table:
    * завантажувати таблицю з файлу;
    * зберігати 5 осей: Z0, V0, m, d, l;
    * мати плоский std::vector<Result> data;
  TableSolver:
    * знаходити найближчі інтервали;
    * робити 5D лінійну інтерполяцію;
    * clamp-ити значення за межами таблиці;
    * бути доступною через SolverType::TABLE.
 */

class TableSolver : public IBallisticSolver {
  void validate_input() const;
  auto calculate_horizontal_fall_distance_m(double fall_time) const -> double;
  auto calculate_free_fall_time_s() const -> double;
  dto::BallisticsInput input;  // static

  BallisticTable table;

public:
  TableSolver(const char* source);

  auto solve(double altitude_m, double att_speed, const dto::Ammo& ammo) -> dto::BallisticResult override;
  const char* name() const override { return "TABLE_SOLVER"; };
};