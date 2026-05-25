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

  BallisticResult result{};
  int retCode = ballistics(result, input);

  ASSERT_EQ(retCode, 0);
  ASSERT_STREQ(result.ammoName, "VOG-17"); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) - ще не вивчив gsl::array_view 
  ASSERT_NEAR(result.dropPoint.x, 173.75, 0.01);
  ASSERT_NEAR(result.dropPoint.y, 173.75, 0.01);
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