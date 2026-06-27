#include "dto/SimStatistics.hpp"
#include "dto/MissionConfig.hpp"
#include "config/ComponentFactory.hpp"
#include "config/defines.hpp"
#include "config/ManualSimulationClock.hpp"
#include "core/MissionProcessor.hpp"


#include <iomanip>
#include <iostream>
#include <cmath>
#include <cstring>
#include <stdexcept>

#ifdef TESTOUT_TO_FILE
#include <fstream>
#endif


namespace {
auto operator<<(std::ostream& os, const dto::SimStatistics& s) -> std::ostream&
{
  return os << "\t\t\t\t\n\tTotal_targets:\t" << s.total 
            << "\t\t\n\tFrom_them: \t\t\n\t _active:\t" << s.active 
            << "\n\t _under_attack:\t" << s.underAttack
            << "\n\t _destroyed:\t" << s.destroyed  
            << "\n\t _fired:\t" << s.firedCount 
            << "\n\t _%_success:\t" <<  (s.firedCount == 0 ? 0 : s.destroyed * 100 / s.firedCount) 
            << "\n\tSteps_taken:\t" << s.steps;
};
}  // namespace

// Components are created via ComponentFactory.
// Ownership is handled by std::unique_ptr, so explicit delete is not needed.
// Ініціалізація через init()
// Пошагова обробка всіх цілей (цикл while (_has Next()) { _step(); })

// ################################################################################
auto main() -> int
{
#ifdef TESTOUT_TO_FILE  // save console to file
  std::streambuf* original_buf = nullptr;
  std::ofstream output_file(defines::kDebugTxtPath);
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
    auto simClock = std::make_unique<ManualSimulationClock>(); 

    auto confLoader = factory.createLoader(ComponentFactory::LoaderType::FILE);
    auto solver = factory.createSolver(ComponentFactory::SolverType::ANALYTICAL ); //ANALYTICAL TABLE
    auto tgtProvider = factory.createProvider(ComponentFactory::ProviderType::JSON, defines::kTargetsPath);

    if (simClock == nullptr || confLoader == nullptr || solver == nullptr || tgtProvider == nullptr) {
      throw std::runtime_error("One or more components unavailable");
    }
    tgtProvider->init(simClock.get());
    core::MissionProcessor processor(std::move(tgtProvider), std::move(solver), 
    std::move(confLoader), simClock.get());

    const dto::MissionConfig* mcnf = processor.init(); 
    simClock->reset(mcnf->time_step, mcnf->tgt_time_step);

    while (processor.hasNext()) {
      if (!processor.step()) {
        LOG("Statistics: " << processor.getSimulationStatistics());
        throw std::runtime_error("Simulation_time_is_over!");
      };
      simClock->advance();
    }

    result = 0;

    LOG("Statistics: " << processor.getSimulationStatistics());

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
  }

#ifdef TESTOUT_TO_FILE            // save console to file
  std::cout.rdbuf(original_buf);  // Обов'язково повертаємо оригінальний буфер
#endif

  return result;
}
