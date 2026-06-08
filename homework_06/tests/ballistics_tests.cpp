#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(Ballistics, ComputesKnownInput)
{
  // Setup
  BallisticInput input{.startPos = Coord{.x = 100.0, .y = 100.0},
                       .targetPos = Coord{.x = 200.0, .y = 200.0},
                       .altitude = 100.0,
                       .attackSpeed = 10.0,
                       .accelPath = 10.0,
                       .ammoName = "VOG-17"};

  // Run
  BallisticResult result{};
  int retCode = ballistics(result, input);

  // Assert
  ASSERT_EQ(retCode, 0);
  EXPECT_STREQ(result.ammoName, "VOG-17");  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) - ще не вивчив gsl::array_view
  EXPECT_NEAR(result.dropPoint.x, 173.759, 0.01);
  EXPECT_NEAR(result.dropPoint.y, 173.759, 0.01);
}

TEST(Ballistics, ComputesKnownInputWithIntermediatePoint)
{
  // Setup
  BallisticInput input{.startPos = Coord{.x = 160.0, .y = 160.0},
                       .targetPos = Coord{.x = 200.0, .y = 200.0},
                       .altitude = 100.0,
                       .attackSpeed = 10.0,
                       .accelPath = 10.0,
                       .ammoName = "VOG-17"};

  // Run
  BallisticResult result{};
  int retCode = ballistics(result, input);

  // Assert
  ASSERT_EQ(retCode, 0);
  EXPECT_STREQ(result.ammoName, "VOG-17");  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) - ще не вивчив gsl::array_view
  EXPECT_NEAR(result.dropPoint.x, 173.759, 0.01);
  EXPECT_NEAR(result.dropPoint.y, 173.759, 0.01);
}

TEST(Ballistics, ComputesKnownInputWithGlidingAmmo)
{
  // Setup
  BallisticInput input{.startPos = Coord{.x = 100.0, .y = 100.0},
                       .targetPos = Coord{.x = 200.0, .y = 200.0},
                       .altitude = 100.0,
                       .attackSpeed = 10.0,
                       .accelPath = 10.0,
                       .ammoName = "GLIDING-RKG"};

  // Run
  BallisticResult result{};
  int retCode = ballistics(result, input);

  // Assert
  ASSERT_EQ(retCode, 0);
  EXPECT_STREQ(result.ammoName,
               "GLIDING-RKG");  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) - ще не вивчив gsl::array_view
  EXPECT_NEAR(result.dropPoint.x, 163.207, 0.01);
  EXPECT_NEAR(result.dropPoint.y, 163.207, 0.01);
}

TEST(Ballistics, ErrorsOnUnknownAmmo)
{
  // Setup
  BallisticInput input{.startPos = Coord{.x = 100.0, .y = 100.0},
                       .targetPos = Coord{.x = 200.0, .y = 200.0},
                       .altitude = 100.0,
                       .attackSpeed = 10.0,
                       .accelPath = 10.0,
                       .ammoName = "VOG-170"};

  BallisticResult result{};
  int retCode = ballistics(result, input);

  ASSERT_EQ(retCode, 1);
}

TEST(Ballistics, ErrorsInvalidValues)
{
  // Setup
  BallisticInput input{.startPos = Coord{.x = 100.0, .y = 100.0},
                       .targetPos = Coord{.x = 200.0, .y = 200.0},
                       .altitude = -100.0,
                       .attackSpeed = 10.0,
                       .accelPath = 10.0,
                       .ammoName = "VOG-17"};

  BallisticResult result{};
  // Run
  int retCode = ballistics(result, input);

  // Assert
  ASSERT_EQ(retCode, 1);
}