#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

std::string env_or_default(const char* name, const std::string& fallback)
{
  if (const char* value = std::getenv(name)) {
    return value;
  }
  return fallback;
}

std::string now_iso()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

int main()
{
  const fs::path config_path = env_or_default("UGV_CONFIG", "/etc/ugv/config.txt");
  const fs::path log_dir = env_or_default("UGV_LOG_DIR", "/var/log/ugv");
  const fs::path data_dir = env_or_default("UGV_DATA_DIR", "/var/data/ugv");

  fs::create_directories(log_dir);
  fs::create_directories(data_dir);

  std::ifstream config(config_path);
  if (!config) {
    std::cerr << "config not found: " << config_path.string() << '\n';
    return 1;
  }

  std::cout << "reading config: " << config_path.string() << '\n';
  for (std::string line; std::getline(config, line);) {
    std::cout << line << '\n';
  }

  {
    std::ofstream log(log_dir / "controller.log", std::ios::app);
    if (!log) {
      std::cerr << "log write failed\n";
      return 1;
    }
    log << now_iso() << " ugv-controller started\n";
    log << now_iso() << " config path: " << config_path.string() << '\n';
  }

  {
    std::ofstream data(data_dir / "pose.csv");
    if (!data) {
      std::cerr << "data write failed\n";
      return 1;
    }
    data << "timestamp_ms,x,y,theta\n";
    data << "0,0.0,0.0,0.0\n";
    data << "100,0.1,0.0,0.0\n";
    data << "200,0.2,0.0,0.0\n";
  }

  std::cout << "wrote log: " << (log_dir / "controller.log").string() << '\n';
  std::cout << "wrote data: " << (data_dir / "pose.csv").string() << '\n';
  return 0;
}
