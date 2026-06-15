#include "dto/SimStatistics.hpp"
#include "config/ComponentFactory.hpp"
#include "config/defines.hpp"
#include "core/MissionProcessor.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstring>
#include <stdexcept>

#ifdef TESTOUT_TO_FILE
#include <fstream>
#endif

namespace dto {
  struct MissionConfig;
}

namespace {
auto operator<<(std::ostream& os, const dto::SimStatistics& s) -> std::ostream&
{
  return os << "\nTotal targets: " << s.total << "\nFrom them: \n\tactive:" << s.active << "\n\tunder attack:" << s.underAttack
            << "\n\tdestroyed:" << s.destroyed << "\nsteps taken:" << s.stepsTaken;
};
}  // namespace

// Components are created via ComponentFactory.
// Ownership is handled by std::unique_ptr, so explicit delete is not needed.
// Ініціалізація через init()
// Пошагова обробка всіх цілей (цикл while (mission.hasNext()) { mission.step(); })

// ################################################################################
auto main() -> int
{
#ifdef TESTOUT_TO_FILE  // save console to file
  std::streambuf* original_buf = nullptr;
  std::ofstream output_file(DEBUG_FILE_NAME);
  if (!output_file.is_open()) {
    std::cerr << "Unable to open debug output file\n";
    return 1;
  }

  original_buf = std::cout.rdbuf();      // Зберігаємо оригінальний буфер консолі
  std::cout.rdbuf(output_file.rdbuf());  // Перенаправляємо cout у файл
#endif

  std::cout << std::fixed << std::setprecision(1);
  int result = 1;
  
  try {
    ComponentFactory factory;
    auto simClock = factory.createSimulationClock(ComponentFactory::SimulationClockType::MANUAL);
    auto confLoader = factory.createLoader(ComponentFactory::LoaderType::FILE);
    auto solver = factory.createSolver(ComponentFactory::SolverType::ANALYTICAL);
    auto tgtProvider = factory.createProvider(ComponentFactory::ProviderType::JSON, defines::kInputPath);

    if (simClock == nullptr || confLoader == nullptr || solver == nullptr || tgtProvider == nullptr) {
      throw std::runtime_error("One or more components unavailable");
    }

    core::MissionProcessor processor(tgtProvider.get(), solver.get(), confLoader.get(), simClock.get());

    const dto::MissionConfig* mcnf = processor.init(defines::kInputPath);
    factory.init(mcnf, simClock.get(), tgtProvider.get());

    while (processor.hasNext()) {
      if (!processor.step()) {
        LOG("\nStatistics: " << processor.getSimulationStatistics());
        throw std::runtime_error("Simulation time is over!");
      };
      simClock->advance();
    }

    result = 0;

    LOG("\nStatistics: " << processor.getSimulationStatistics());

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
  }

#ifdef TESTOUT_TO_FILE            // save console to file
  std::cout.rdbuf(original_buf);  // Обов'язково повертаємо оригінальний буфер
#endif

  return result;
}
