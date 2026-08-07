#include "reporter.h"
#include <string>
#include <iostream>

constexpr auto hostName = "http://cppmiltech.com.ua";
constexpr auto apiKey = "dz12-vX7mK4qT9r2w";
constexpr auto connTimeout = 1000;
constexpr auto readTimeout = 1000;

int main(int argc, char **argv)
{
  auto reporter = Reporter(connTimeout, readTimeout, hostName, apiKey);

  for (int i = 1; i <= TEST_COUNT; i++) {
    auto result = reporter.report(i);
    std::cout << result.testId << " --> " << result.httpStatusCode << " --> " << result.attempt << std::endl;
  }

  return 0;
}
