#pragma once

#include "dto/MissionConfig.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/IConfigLoader.hpp"

//риймає компоненти через вказівники на інтерфейси (патерн Стратегія)
class MissionProcessor {

 public:  

  void init(dto::MissionConfig configSource); //Завантажити конфіг через IConfigLoader, підготувати дані для ітерації
  bool hasNext(); //Перевірити, чи є ще необроблені цілі
  void reset();//Почати ітерацію спочатку
  void changeSolver(IBallisticSolver* s); //Підмінити solver на льоту (Стратегія)
  
  bool step(); //Обробити наступну ціль: взяти дані з ITargetProvider, обчислити через IBallisticSolver, повернути DropPoint
// Логіка step():
// 1. Взяти наступну ціль через targets->getTarget(currentIdx)
// 2. Викликати solver->solve(dronePos, target.pos, altitude, ammo)
// 3. Збільшити лічильник, повернути результат

  MissionProcessor(ITargetProvider* targets,
                   IBallisticSolver* solver,
                   IConfigLoader* loader);

 private:
 
  ITargetProvider* targets_;      // borrowed, not owning
  IBallisticSolver* solver_;      // borrowed, not owning
  IConfigLoader* loader_;         // borrowed, not owning
  
};
