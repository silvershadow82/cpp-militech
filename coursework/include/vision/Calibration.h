#pragma once

#include <optional>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "models/CameraModel.h"

namespace follow::vision {

struct CalibrationResult {
  models::Intrinsics intrinsics{};  // at the resolution of the calibration images
  double rms{0.0};                  // reprojection error, px; bring-up stage 0 requires < 0.5
  int views{0};
};

// Inner corners of a `board` (inner-corner count, e.g. 9x6) refined to sub-pixel, or nullopt.
std::optional<std::vector<cv::Point2f>> findBoardCorners(const cv::Mat& image, const cv::Size& board);

// cv::fisheye::calibrate over detected corners of a board with `squareM` squares. Throws
// std::runtime_error with fewer than 3 views, which cannot constrain the model.
CalibrationResult calibrateFisheye(const std::vector<std::vector<cv::Point2f>>& corners,
                                   const cv::Size& board,
                                   double squareM,
                                   const cv::Size& image);

// The camera file follow.json names, in the keys parseCamera reads.
nlohmann::json cameraJson(const CalibrationResult& result, double tiltDeg);

}  // namespace follow::vision
