#include "reporter.h"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <httplib.h>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <memory>
#include <thread>
#include <filesystem>

using json = nlohmann::json;

namespace {
namespace fs = std::filesystem;
std::string testId(int testNo)
{
  return std::format("T{:02}", testNo);
}

std::string testFilePath(int testNo)
{
  const fs::path dataFolder = "../data";
  return fs::canonical(dataFolder) / folderMapping[testNo - 1] / "simulation.json";
}
}  // namespace

Reporter::Reporter(int connectionTimeout, int readTimeout, const std::string& hostName, const std::string& apiKey)
  : connectionTimeout(connectionTimeout)
  , readTimeout(readTimeout)
  , hostName(hostName)
  , apiKey(apiKey)
{
  this->httpClient = std::make_unique<httplib::Client>(hostName);
  this->httpClient->set_connection_timeout(std::chrono::milliseconds(connectionTimeout));
  this->httpClient->set_read_timeout(std::chrono::milliseconds(readTimeout));
};

Reporter::~Reporter() = default;
Reporter::Reporter(Reporter&&) noexcept = default;
Reporter& Reporter::operator=(Reporter&&) noexcept = default;

ReportResult Reporter::sendWithRetry(int testNo, const std::string& body, const ReportResult& prevResult)
{
  if (prevResult.attempt >= MAX_RETRIES) {
    return prevResult;
  }

  auto result = prevResult;
  auto headers = httplib::Headers{{"x-api-key", this->apiKey}};
  auto r = this->httpClient->Post("/api/dz12/results", headers, body, "application/json");

  if (!r) {
    const auto error = r.error();
    switch (error) {
      case httplib::Error::Timeout:
        std::cerr << "Timed out" << std::endl;
        result.timeout = true;
        break;
      case httplib::Error::ConnectionTimeout:
        std::cerr << "Failed with connection timeout, increase the corresponding value" << std::endl;
        result.timeout = true;
        break;
      default:
        std::cerr << "Uknown error: " << httplib::to_string(error) << std::endl;
        break;
    }
  }
  result.httpStatusCode = r->status;

  if (result.ok()) {
    return result;
  }
  if (result.retry()) {
    std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(1 * std::pow(2, result.attempt))));
    result.attempt++;
    result = this->sendWithRetry(testNo, body, result);
  }
  else {
    result.message = std::format("Invalid request after {} attempts", result.attempt + 1);
  }
  return result;
}

ReportResult Reporter::report(int testNo)
{
  const auto fileName = testFilePath(testNo);
  std::cout << "Using " << fileName << " for test " << testNo << std::endl;
  std::ifstream simFile(fileName);

  if (!simFile.is_open()) {
    std::cerr << "Unable to open " << fileName << std::endl;
    return ReportResult{.testId = testId(testNo), .message = "Unable to open simulation file"};
  }

  json sim{};
  simFile >> sim;

  json body{};
  body["studentId"] = "2084";
  body["testId"] = testId(testNo);
  body["simulation"] = sim;

  simFile.close();

  return ReportResult{.testId = testId(testNo), .httpStatusCode = 201, .attempt = 1};

  //   return this->sendWithRetry(testNo, body.dump(2));
}