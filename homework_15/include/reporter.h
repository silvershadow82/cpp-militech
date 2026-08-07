#pragma once

#include <cstdio>
#include <memory>
#include <string>

// Forward definition
namespace httplib {
class Client;
};

constexpr size_t TEST_COUNT = 10;
constexpr int MAX_RETRIES = 5;

inline constexpr const char *folderMapping[TEST_COUNT] = {
  "basic", "elliptic", "loops", "flower", "lissajous", "quickslow", "heavyammo", "glidingammo", "cardioids", "extreme"};

// Не в анонімному namespace: Reporter::report повертає цей тип, тому він мусить бути одним і тим
// самим типом у всіх translation unit.
struct ReportResult {
  std::string testId{};
  int httpStatusCode{0};
  int attempt{0};
  bool timeout{false};
  std::string message;

  bool ok() const { return httpStatusCode == 200 || httpStatusCode == 201; }
  bool retry() const { return timeout || (httpStatusCode >= 500 && attempt < MAX_RETRIES); }
};

class Reporter {
private:
  int connectionTimeout{1000};
  int readTimeout{1000};
  std::string hostName;
  std::string apiKey;
  std::unique_ptr<httplib::Client> httpClient;

  ReportResult sendWithRetry(int testNo, const std::string &body, const ReportResult &prevResult = {});

public:
  Reporter(int connectionTimeout, int readTimeout, const std::string &hostName, const std::string &apiKey);

  // httplib::Client is only forward-declared here, so the special members that have to destroy it
  // are defined in reporter.cpp, where the type is complete.
  ~Reporter();
  Reporter(Reporter &&) noexcept;
  Reporter &operator=(Reporter &&) noexcept;

  ReportResult report(int testNo);
};