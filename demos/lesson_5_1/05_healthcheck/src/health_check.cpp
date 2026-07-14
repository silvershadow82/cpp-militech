#include <filesystem>

namespace fs = std::filesystem;

int main()
{
  if (!fs::exists("/tmp/ugv.ready")) {
    return 1;
  }
  if (fs::exists("/tmp/ugv.force_fail")) {
    return 1;
  }
  return 0;
}
