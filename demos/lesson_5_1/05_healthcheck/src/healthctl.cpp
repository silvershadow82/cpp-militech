#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: rd51-healthctl fail|recover\n";
    return 2;
  }

  const std::string_view command = argv[1];

  if (command == "fail") {
    std::ofstream("/tmp/ugv.force_fail") << "fail\n";
    return 0;
  }

  if (command == "recover") {
    fs::remove("/tmp/ugv.force_fail");
    return 0;
  }

  std::cerr << "usage: rd51-healthctl fail|recover\n";
  return 2;
}
