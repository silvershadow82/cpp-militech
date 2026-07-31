#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

template <class T>
class ThreadSafeQueue {
private:
  std::queue<T> queue;
  mutable std::mutex mtx;
  std::condition_variable cv;
  bool done{false};

public:
  void push(T value)
  {
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->queue.push(std::move(value));
    }
    this->cv.notify_one();
  }

  // Non-blocking pop. Returns false if the queue is empty.
  bool try_pop(T &outValue)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    if (this->queue.empty()) {
      return false;
    }
    outValue = std::move(this->queue.front());
    this->queue.pop();
    return true;
  }

  // Blocking pop. Waits until an item is available or shutdown() is called.
  // Returns false if woken with an empty queue after shutdown.
  bool pop(T &outValue)
  {
    std::unique_lock<std::mutex> lock(this->mtx);
    this->cv.wait(lock, [this] { return !this->queue.empty() || this->done; });
    if (this->queue.empty()) {
      return false;
    }
    outValue = std::move(this->queue.front());
    this->queue.pop();
    return true;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    return this->queue.empty();
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->done = true;
    }
    this->cv.notify_all();
  }
};
