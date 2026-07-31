#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace fs = std::filesystem;

std::atomic_bool keep_running{true};

void handle_signal(int)
{
  keep_running = false;
}

int main()
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  fs::remove("/tmp/ugv.ready");
  fs::remove("/tmp/ugv.force_fail");

  std::cout << "ugv-controller starting\n" << std::flush;
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::ofstream("/tmp/ugv.ready") << "ready\n";
  std::cout << "ugv-controller ready\n" << std::flush;

  while (keep_running) {
    std::ofstream("/tmp/ugv.ready") << "ready\n";
    std::cout << "heartbeat\n" << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  fs::remove("/tmp/ugv.ready");
  return 0;
}
