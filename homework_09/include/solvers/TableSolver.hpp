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

  BallisticTable table;  // TODO can I mark it const?

public:
  TableSolver(const char* source);

  auto solve(const pointmath::Point& drone_position, const pointmath::Point& target_position) -> dto::DropSolution override;

  auto solve(const pointmath::Point& drone_position,
             const pointmath::Point& target_position,
             double altitude_m,
             double att_speed,
             double acc_path,
             const dto::Ammo& ammo) -> dto::DropSolution override;
};