#include "core_/MissionProcessor.hpp"
#include "core_/TimeTracker.hpp"
#include "config/ComponentFactory.hpp"
#include "config/defines.hpp"
#include "dto/SimStatistics.hpp"
#include "dto/MissionConfig.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "drone/DronePhysics.hpp"

#include <chrono>
// #include <condition_variable>
//  #include <queue>
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
};

struct InputPaths {
  std::string configPath = defines::kConfig_Path;
  std::string ammoTablePath = defines::kAmmoTable_Path;
  std::string targetsPath = defines::kTargets_Path;
  std::string ballisticTablePath = defines::kBallisticTable_Path;
  std::string simulationPath = defines::kSimulationPath;
};

auto parseInputPaths(int argc, char* argv[]) -> InputPaths
{
  InputPaths paths;

  if (argc > 6) {
    throw std::runtime_error("Usage: hw10_drone_sim [config_path] [ammo_table_path] [targets_path] [ballistic_table_path] [simulation_path]");
  }

  if (argc > 1) {
    paths.configPath = argv[1];
  }

  if (argc > 2) {
    paths.ammoTablePath = argv[2];
  }

  if (argc > 3) {
    paths.targetsPath = argv[3];
  }
  if (argc > 4) {
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
  std::unique_ptr<ITargetProvider> tgtProvider;       // from factory
  std::unique_ptr<drone::DronePhysics> physics;       // created with conf data
  std::unique_ptr<core::MissionProcessor> processor;  // created with conf data above ptrs
  std::thread providerThread;
  std::thread physicsThread;
  std::thread missionThread;

  try {
    const auto paths = parseInputPaths(argc, argv);
    ComponentFactory factory;
    auto solver = factory.createSolver(ComponentFactory::SolverType::TABLE, paths.ballisticTablePath);

    auto confLoader = factory.createLoader(ComponentFactory::LoaderType::FILE);
    if (confLoader == nullptr) {
      throw std::runtime_error("Configuration unavailable");
    }
    if (!confLoader->load(paths.configPath, paths.ammoTablePath)) {
      throw std::runtime_error("Error loading configuration");
    };

    const dto::MissionConfig conf = confLoader->getConfig();
    tgtProvider = factory.createProvider(ComponentFactory::ProviderType::JSON, paths.targetsPath, conf);
    if (solver == nullptr || tgtProvider == nullptr) {
      throw std::runtime_error("One or more components unavailable");
    }

    physics = std::make_unique<drone::DronePhysics>(conf);

    processor = std::make_unique<core::MissionProcessor>(
      *tgtProvider.get(), std::move(solver), *physics.get(), confLoader->getAmmoParams(), conf.timeStep, defines::kMaxSteps, paths.simulationPath);

    providerThread = std::thread(&ITargetProvider::run, tgtProvider.get());
    physicsThread = std::thread(&drone::DronePhysics::run, physics.get());
    missionThread = std::thread(&core::MissionProcessor::run, processor.get());

    tt.start(conf.timeScale);
    while (!tgtProvider->isThreadReady() || !physics->isThreadReady() || !processor->isThreadReady()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (tt.getElapsed() > 0.1) {
        throw std::runtime_error("Threads are not ready!");
      }
    };

    /*   }  // eo try

      catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return result;
      } */

    double time = 0.0;
    tgtProvider->start();
    physics->start();
    processor->start();


 /*    int sleep_time = std::round(100 / conf.timeScale);
    LOG("sleep_time=" << sleep_time);
    while ((time = tt.getElapsed()) < 11.0) {  // TODO remove
      double t_upd;
      dto::Target tgt = tgtProvider->getTarget(1, t_upd);
      LOG(time << " " << t_upd << " T1:" << tgt.position << "V:" << tgt.velocity);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    } */
    /* TODO restore    while (processor.hasNext()) {
          if (!processor.step()) {
            LOG("Statistics: " << processor.getSimulationStatistics());
            throw std::runtime_error("Simulation_time_is_over!");
          };
          std::this_thread::sleep_for(std::chrono::milliseconds(100));  // TODO
        }

        LOG("Statistics: " << processor.getSimulationStatistics());*/
    missionThread.join();  // чекаємо завершення місії
    physics->stop();      
    tgtProvider->stop();

    providerThread.join();
    physicsThread.join();

    result = 0;

  }  // eo try

  catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return result;
  }

  LOG("Statistics: " << processor->getSimulationStatistics());

#ifdef TESTOUT_TO_FILE            // save console to file
  std::cout.rdbuf(original_buf);  // Обов'язково повертаємо оригінальний буфер
#endif

  return result;
}

// #################################################################

/*   std::condition_variable cv;
std::queue<Target> taskQueue;

Producer-Consumer:

std::mutex mtx;
std::condition_variable cv;
std::queue<Target> taskQueue;
bool done = false;

void producer(const std::vector<Target>& targets) {
  for (const auto& t : targets) {
      {
          std::lock_guard<std::mutex> lk(mtx);
          taskQueue.push(t);
      }
      cv.notify_one();
  }
  { std::lock_guard<std::mutex> lk(mtx); done = true; }
  cv.notify_all();
}

void consumer() {
  while (true) {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&]{ return !taskQueue.empty() || done; });
      if (taskQueue.empty() && done) return;
      auto t = std::move(taskQueue.front());
      taskQueue.pop();
      lk.unlock(); //NB! obligatory for wait()
      t.process();
  }
} */
/*
class WorkerPool {
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_ = false;
public:
    WorkerPool(size_t n = std::thread::hardware_concurrency()) {
        for (size_t i = 0; i < n; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }
    void submit(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            tasks_.push(std::move(task));
        }
        cv_.notify_one();
    }
         ~WorkerPool() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) w.join();
    }
     private:
    void workerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this]{ return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }
};
    */
// #################################################################
