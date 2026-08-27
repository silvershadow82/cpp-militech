#include "reporter.h"
#include <gtest/gtest.h>

TEST(ReportResult, OkOnlyOnSuccessCodes)
{
  // Setup / Run
  const ReportResult ok200{.httpStatusCode = 200};
  const ReportResult ok201{.httpStatusCode = 201};
  const ReportResult notFound{.httpStatusCode = 404};
  const ReportResult serverError{.httpStatusCode = 500};

  // Assert
  EXPECT_TRUE(ok200.ok());
  EXPECT_TRUE(ok201.ok());
  EXPECT_FALSE(notFound.ok());
  EXPECT_FALSE(serverError.ok());
}

TEST(ReportResult, RetriesOnTimeoutRegardlessOfStatus)
{
  // Setup / Run
  const ReportResult result{.httpStatusCode = 0, .attempt = 0, .timeout = true};

  // Assert
  EXPECT_TRUE(result.retry());
}

TEST(ReportResult, RetriesOnServerErrorBeforeMaxAttempts)
{
  // Setup / Run
  const ReportResult result{.httpStatusCode = 500, .attempt = MAX_RETRIES - 1, .timeout = false};

  // Assert
  EXPECT_TRUE(result.retry());
}

TEST(ReportResult, StopsRetryingAtMaxAttempts)
{
  // Setup / Run
  const ReportResult result{.httpStatusCode = 500, .attempt = MAX_RETRIES, .timeout = false};

  // Assert
  EXPECT_FALSE(result.retry());
}

TEST(ReportResult, DoesNotRetryOnClientError)
{
  // Setup / Run
  const ReportResult result{.httpStatusCode = 404, .attempt = 0, .timeout = false};

  // Assert
  EXPECT_FALSE(result.retry());
}
