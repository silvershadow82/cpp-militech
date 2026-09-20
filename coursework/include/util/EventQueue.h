#pragma once

#include <deque>
#include <mutex>
#include <utility>
#include <vector>

namespace follow::runtime {

// Lossless FIFO between threads, for edge events such as tracker requests that must not be
// overwritten the way a Latest slot overwrites values.
template <class T>
class EventQueue {
public:
  void push(T event)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->events.push_back(std::move(event));
  }

  // Removes and returns every queued event, oldest first.
  std::vector<T> drain()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::vector<T> out(std::make_move_iterator(this->events.begin()), std::make_move_iterator(this->events.end()));
    this->events.clear();
    return out;
  }

private:
  std::mutex mutex;
  std::deque<T> events;
};

}  // namespace follow::runtime
