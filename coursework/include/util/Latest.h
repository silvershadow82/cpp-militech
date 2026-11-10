#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

#include "Types.h"

namespace follow::util {

template <class T>
struct Stamped {
  T value{};
  models::TimePoint t{};  // when the value was written
  uint64_t sequence{0};   // 1 for the first write, +1 for each later write
};

// Latest-value slot shared between threads: writers overwrite, readers copy the newest value.
template <class T>
class Latest {
public:
  void write(T value, models::TimePoint t)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    uint64_t next = this->slot ? this->slot->sequence + 1 : 1;
    this->slot = Stamped<T>{.value = std::move(value), .t = t, .sequence = next};
  }

  std::optional<Stamped<T>> read() const
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->slot;
  }

  // The value if it was written at most maxAge before now; nullopt if never written or older.
  std::optional<T> readFresh(models::TimePoint now, models::Clock::duration maxAge) const
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->slot || now - this->slot->t > maxAge) {
      return std::nullopt;
    }
    return this->slot->value;
  }

private:
  mutable std::mutex mutex;
  std::optional<Stamped<T>> slot{};
};

}  // namespace follow::util
