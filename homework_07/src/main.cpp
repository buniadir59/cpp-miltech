
#include "dto/MissionConfig.hpp"
#include "dto/SimStatistics.hpp"
#include "config/FileConfigLoader.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "ManualSimulationClock.hpp"
#include "MissionProcessor.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "defines.hpp"

#include <exception>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstring>

#ifdef TESTOUT_TO_FILE
#include <fstream>
#endif

namespace {
        auto operator<<(std::ostream& os, const dto::SimStatistics& s) -> std::ostream& {
            return os << "\nTotal targets: " << s.total 
            << "\nFrom them: \n\tactive:" << s.active 
            << "\n\tunder attack:" << s.underAttack 
            << "\n\tdestroyed:" << s.destroyed 
            << "\nTime elapsed, sec:" << s.timeElapsed;
        };
}

/* В main() показати:
1. Створення компонентів через фабрику //TODO
2. Ініціалізація через init()
3. Пошагова обробка всіх цілей (цикл while (mission.hasNext()) { mission.step(); })
4. delete всіх створених об’єктів */

// ################################################################################
auto main() -> int
{  
#ifdef TESTOUT_TO_FILE //save console to file
  std::streambuf* original_buf = nullptr;
  std::ofstream output_file(DEBUG_FILE_NAME);
    if (!output_file.is_open()) {
      std::cerr << "Unable to open debug output file\n";
      return 1;
    }

  original_buf = std::cout.rdbuf(); // Зберігаємо оригінальний буфер консолі
  std::cout.rdbuf(output_file.rdbuf()); // Перенаправляємо cout у файл
#endif

  std::cout << std::fixed << std::setprecision(1);
  int result = 1;

  try {
    
    FileConfigLoader confLoader;
    ManualSimulationClock simClock;
    JsonTargetProvider tgtProvider(defines::kInputPath, 5.0, &simClock);
    AnalyticalSolver solver;
    core::MissionProcessor processor(&tgtProvider, &solver, &confLoader, &simClock);

    processor.init(defines::kInputPath);

    while (processor.hasNext()) { 
      simClock.advance(processor.mconf->time_step);  
    
      if (!processor.step()) {
        LOG("\nStatistics: " << processor.getSimulationStatistics());
        throw std::runtime_error("Simulation time is over!");
      }; 
    }

    result = 0;
   
    LOG("\nStatistics: " << processor.getSimulationStatistics());

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
  }

#ifdef TESTOUT_TO_FILE //save console to file
      std::cout.rdbuf(original_buf); // Обов'язково повертаємо оригінальний буфер 
#endif


  return result;
}
