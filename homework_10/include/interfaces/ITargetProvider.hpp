#pragma once

#include "dto/Target.hpp"

// Провайдер цілей: кількість та дані кожної цілі (позиція, швидкість)

/*
Траєкторії з targets.json (формат ДЗ3) — приватні дані провайдера, назовні їх не видно. Власний потік провайдера кожні arrayTimeStep секунд
переходить до наступного вузла траєкторії (із зацикленням, як у ДЗ2) і оновлює поточні позиції під м'ютексом. Розрахунок поточної швидкості
— теж обов'язок провайдера: кінцева різниця сусідніх вузлів траєкторії, поділена на arrayTimeStep. Методи: •       isThreadReady() — потік
створено, і він готовий стартувати. •       start() — сигнал почати рух цілей. Окремий від створення потоку: якщо цілі почнуть рухатися,
поки решта системи ще ініціалізується, симуляція стартує з розсинхронізацією. •       stop() — атомарний стоп-прапорець + join(). •
getTargetCount(), getTarget(int) — копія Target під м'ютексом, жодних посилань на внутрішні дані.
 */

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual dto::Target getTarget(int index) = 0;

  virtual void run() = 0;
  virtual auto start() -> void = 0;          // сигнал почати рух цілей
  virtual auto stop() -> void = 0;           // атомарний стоп-прапорець + join().
  virtual auto isThreadReady() -> bool = 0;  // потік створено, і він готовий стартувати.
  virtual auto hasFailed() const -> bool = 0; //ended through expeption
  // default constructor & destructor
  ITargetProvider() = default;
  virtual ~ITargetProvider() = default;
  // prohibit copying
  ITargetProvider(const ITargetProvider&) = delete;
  auto operator=(const ITargetProvider&) -> ITargetProvider& = delete;
  // prohibit moving
  ITargetProvider(ITargetProvider&&) = delete;
  auto operator=(ITargetProvider&&) -> ITargetProvider& = delete;
};