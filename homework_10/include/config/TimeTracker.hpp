#pragma once

#include <chrono>
#include <cmath>

// Pattern Meyers' Singleton - delayed initialization

class TimeTracker {
private:
  std::chrono::steady_clock::time_point start_time;
  double timeScale = 1.0;  // by default no scaling
  bool isRunning = false;
  TimeTracker() = default;  // private constructor & deleted copy constructors

public:
  TimeTracker(const TimeTracker&) = delete;
  TimeTracker& operator=(const TimeTracker&) = delete;

  static TimeTracker& getInstance()
  {
    static TimeTracker instance;  // created only on the first call
    return instance;
  }

  void start()
  {
    start_time = std::chrono::steady_clock::now();
    isRunning = true;
  }

  void start(double time_scale)
  {
    timeScale = time_scale;
    start();
  };

  double getElapsed() const
  {
    return isRunning ? (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time).count()) *
                         timeScale
                     : 0.0;
  }

  auto nextWakeup(double period_sec)
  {
    auto t_now = std::chrono::steady_clock::now();

    // тривалість одного кроку в секундах (наприклад, 0.1 / 10.0 = 0.01 сек)
    const std::chrono::duration<double> step_duration(period_sec / timeScale);
    // Рахуємо, скільки кроків пройшло від моменту старту
    // Використовуємо (std::ceil) для індекса НАСТУПНОГО кратного кроку сітки
    int next_step_index = std::ceil((t_now - start_time) / step_duration); //long long 

    // 3. Задаємо абсолютний час пробудження: старт + (кількість кроків * тривалість кроку)
    auto next_wakeup = start_time + (next_step_index * step_duration);

    return next_wakeup;
  }
};