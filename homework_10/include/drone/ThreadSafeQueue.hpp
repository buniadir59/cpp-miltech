#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <utility>

//  Черга команд
template <typename T>
class ThreadSafeQueue {
private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;

public:
  ThreadSafeQueue() = default;

  // Disable copying and assignment to prevent thread synchronization bugs
  ThreadSafeQueue(const ThreadSafeQueue&) = delete;
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  // Pushes an item by copying
  void push(const T& value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(value);
    cv_.notify_one();  // Wake up one waiting consumer thread
  }

  // Pushes an item by moving (efficient for heavy objects)
  void push(T&& value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
    cv_.notify_one();
  }

  // Blocking Pop: Waits until an element is available, then extracts it
  T wait_and_pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Use a lambda predicate to safely avoid spurious wakeups
    cv_.wait(lock, [this]() { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  // Non-blocking Pop: Returns a std::optional immediately without waiting
  std::optional<T> try_pop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }

    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  // Checks if the queue is empty
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  // Returns the current size of the queue
  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }
};
