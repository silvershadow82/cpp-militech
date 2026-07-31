#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

std::atomic_bool keep_running{true};

void handle_signal(int)
{
  keep_running = false;
}

int run_cli()
{
  std::cout << "rd51> " << std::flush;

  for (std::string command; std::getline(std::cin, command);) {
    if (command == "status") {
      std::cout << "status=ok mode=demo\n";
    }
    else if (command == "exit") {
      return 0;
    }
    else if (!command.empty()) {
      std::cout << "unknown command: " << command << '\n';
    }

    std::cout << "rd51> " << std::flush;
  }

  return 0;
}

int main(int argc, char** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--cli") {
    return run_cli();
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::cout << "ugv-controller started\n" << std::flush;

  int tick = 0;
  while (keep_running) {
    std::cout << "tick=" << tick++ << " mode=demo status=ok\n" << std::flush;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  std::cout << "ugv-controller stopping\n" << std::flush;
  return 0;
}
