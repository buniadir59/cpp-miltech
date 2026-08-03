#include "mission/MissionProcessor.hpp"
#include "config/TimeTracker.hpp"
#include "config/ComponentFactory.hpp"
#include "config/defines.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/MissionConfig.hpp"
//#include "interfaces/ITargetProvider.hpp"
//#include "drone/DronePhysics.hpp"

#include <chrono>
#include <memory>
#include <thread>
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
  return os << defines::kAppname << " " << s.solverName << " " << s.ammoName << " " << "\t\t\t\t\n\tTotal_targets:\t" << s.total
            << "\t\t\n\tFrom_them: \t\t\n\t _active:\t" << s.active << "\n\t _under_attack:\t" << s.underAttack << "\n\t _destroyed:\t"
            << s.destroyed << "\n\t _fired:\t" << s.firedCount << "\n\t _%_success:\t"
            << (s.firedCount == 0 ? 0 : s.destroyed * 100 / s.firedCount) << "\n\tSteps_taken:\t" << s.steps;
}

struct InputPaths {
  std::string configPath = defines::kConfig_Path;
  std::string ballisticTablePath = defines::kBallisticTable_Path;
  std::string simulationPath = defines::kSimulationPath;
};

auto parseInputPaths(int argc, char* argv[]) -> InputPaths
{
  InputPaths paths;

  if (argc > 6) {
    throw std::runtime_error(
      "Usage: hw10_drone_sim [config_path] [ballistic_table_path] [simulation_path]");
  }

  if (argc > 1) {
    paths.configPath = argv[1];
  }

  if (argc > 2) {
    paths.ballisticTablePath = argv[4];
  }
  if (argc > 5) {
    paths.simulationPath = argv[5];
  }
  return paths;
}

}  // namespace

auto main(int argc, char* argv[]) -> int
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

  // LOG("#hardware_concurrency() => " << std::thread::hardware_concurrency());

  std::cout << std::fixed << std::setprecision(2);
  int result = 1;

  TimeTracker& tt = TimeTracker::getInstance();       // scale is set with start
  std::unique_ptr<core::MissionProcessor> processor;  // created with conf data above ptrs
  std::thread missionThread;

/*  TODO
Додати UartPort і перевірити, що програма може відкрити /tmp/ttyA.
Додати GpioControl і підняти START.
Тільки після цього підключати стару mission logic.
Тобто перша мета — не “влучити”, а зробити мінімальний цикл:

open UART
open GPIO
START = 1
read AMMO / CONFIG / TELEMETRY / TARGET
send neutral CONTROL
exit cleanly

Після цього вже переносимо наведення і DROP
*/
  try {
    const auto paths = parseInputPaths(argc, argv);
    ComponentFactory factory;

    auto solver = factory.createSolver(ComponentFactory::SolverType::TABLE, paths.ballisticTablePath);
    auto confLoader = factory.createLoader(ComponentFactory::LoaderType::FILE);
    if (confLoader == nullptr) {
      throw std::runtime_error("Configuration unavailable");
    }

    const dto::MissionConfig conf = confLoader->getConfig();

    processor = std::make_unique<core::MissionProcessor>(
      conf,  std::move(solver), confLoader->getAmmoParams(), defines::kMaxSteps, paths.simulationPath);

    missionThread = std::thread(&core::MissionProcessor::run, processor.get());

    auto checkFailed = [&]() {
      if ( processor->hasFailed()) {
        throw std::runtime_error("Thread failed during startup");
      }
    };

    tt.start(conf.timeScale);
    while (!processor->isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      checkFailed();
    }

    processor->start();

    missionThread.join();  // чекаємо завершення місії

    checkFailed();
    result = 0;

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    if (processor != nullptr) {
      processor->stop();
    }

    auto joinIfJoinable = [](std::thread& thread) {
      if (thread.joinable()) {
        thread.join();
      }
    };
    joinIfJoinable(missionThread);
  }  // eo catch

  if ((processor != nullptr) && !result) {
    LOG("Statistics: " << processor->getSimulationStatistics());
  }

#ifdef TESTOUT_TO_FILE            // save console to file
  std::cout.rdbuf(original_buf);  // Обов'язково повертаємо оригінальний буфер
#endif

  return result;
}
