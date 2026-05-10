#include "ballistics.hpp"

#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

auto read_input_file(const std::string& file_path)
{
  std::ifstream input_file(file_path);

  if (!input_file.is_open()) {
    throw std::runtime_error("Unable to open input file: " + file_path);
  }

  ballistics::BallisticsInput input;

  input_file >> input.drone_x >> input.drone_y >> input.drone_z >> input.target_x >> input.target_y >> input.attack_speed >>
    input.acceleration_path >> input.ammo_name;

  if (!input_file) {
    throw std::runtime_error("Input file has invalid or incomplete data");
  }

  return input;
}

void print_solution(const ballistics::DropSolution& solution)
{
  std::cout << std::fixed << std::setprecision(3);

  std::cout << "fall_time_s " << solution.fall_time_s << '\n';
  std::cout << "horizontal_fall_distance_m " << solution.horizontal_fall_distance_m << '\n';

  if (solution.has_intermediate_point) {
    std::cout << "intermediate_point " << solution.intermediate_x << ' ' << solution.intermediate_y << '\n';
  }

  std::cout << "drop_point " << solution.fire_x << ' ' << solution.fire_y << '\n';
}

void save_solution(const ballistics::DropSolution& solution)
{
  std::ofstream o_file("output.txt");
  if (!o_file.is_open()) {
    throw std::runtime_error("Unable to open output.txt");
  }
  o_file << std::fixed << std::setprecision(3);

  if (solution.has_intermediate_point) {
    o_file << "intermediate_point " << solution.intermediate_x << ' ' << solution.intermediate_y << ' ';
  }
  o_file << solution.fire_x << ' ' << solution.fire_y << '\n';
}

}  // namespace

auto main(int argc, char* argv[]) -> int
{
  if (argc != 2) {
    std::cerr << "Usage: ballistics_cli <input-file>\n";
    return 1;
  }

  try {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic): argv[1] is valid after argc check.
    const ballistics::BallisticsInput input = read_input_file(argv[1]);
    const ballistics::DropSolution solution = ballistics::compute_drop_solution(input);

    print_solution(solution);

    save_solution(solution);
  }
  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
