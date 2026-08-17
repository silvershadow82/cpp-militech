#pragma once

#include <iostream>
#include <mutex>

#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

// Shared mutex so concurrent log output from multiple threads does not race on
inline std::mutex &logMutex()
{
  static std::mutex m;
  return m;
}

#if ENABLE_LOG
#define LOG(msg)                                      \
  do {                                                \
    std::lock_guard<std::mutex> _logLock(logMutex()); \
    std::cout << "[LOG] " << msg << std::endl;        \
  } while (0)
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg)                                    \
  do {                                                \
    std::lock_guard<std::mutex> _logLock(logMutex()); \
    std::cout << "[DEBUG] " << msg << std::endl;      \
  } while (0)
#else
#define DEBUG(msg)
#endif