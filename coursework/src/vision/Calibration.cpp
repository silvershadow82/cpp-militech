#include "follow/vision/Calibration.h"

#include <stdexcept>

#include <opencv2/calib3d.hpp>  // cv::fisheye on OpenCV 4 and 5 (5.0 forwards to calib.hpp)
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/objdetect.hpp>)
#include <opencv2/objdetect.hpp>  // OpenCV 5 moved chessboard detection here; OpenCV 4 has it in calib3d
#endif

namespace follow::vision {

std::optional<std::vector<cv::Point2f>> findBoardCorners(const cv::Mat& image, const cv::Size& board)
{
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  }
  else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  std::vector<cv::Point2f> corners;
  int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  if (!cv::findChessboardCorners(gray, board, corners, flags)) {
    return std::nullopt;
  }
  cv::cornerSubPix(
    gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
  return corners;
}

CalibrationResult calibrateFisheye(const std::vector<std::vector<cv::Point2f>>& corners,
                                   const cv::Size& board,
                                   double squareM,
                                   const cv::Size& image)
{
  if (corners.size() < 3) {
    throw std::runtime_error("calibration needs at least 3 views with a detected board, got " + std::to_string(corners.size()));
  }
  std::vector<cv::Point3f> boardPoints;
  for (int r = 0; r < board.height; ++r) {
    for (int c = 0; c < board.width; ++c) {
      boardPoints.emplace_back(static_cast<float>(c * squareM), static_cast<float>(r * squareM), 0.0f);
    }
  }
  std::vector<std::vector<cv::Point3f>> objectPoints(corners.size(), boardPoints);

  cv::Matx33d K = cv::Matx33d::eye();
  cv::Vec4d D(0.0, 0.0, 0.0, 0.0);
  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
  double rms = cv::fisheye::calibrate(
    objectPoints, corners, image, K, D, rvecs, tvecs, flags, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-8));

  CalibrationResult result;
  result.intrinsics = core::Intrinsics{.width = image.width,
                                       .height = image.height,
                                       .fx = K(0, 0),
                                       .fy = K(1, 1),
                                       .cx = K(0, 2),
                                       .cy = K(1, 2),
                                       .k1 = D[0],
                                       .k2 = D[1],
                                       .k3 = D[2],
                                       .k4 = D[3]};
  result.rms = rms;
  result.views = static_cast<int>(corners.size());
  return result;
}

nlohmann::json cameraJson(const CalibrationResult& result, double tiltDeg)
{
  const core::Intrinsics& k = result.intrinsics;
  return nlohmann::json{{"model", "fisheye"},
                        {"width", k.width},
                        {"height", k.height},
                        {"fx", k.fx},
                        {"fy", k.fy},
                        {"cx", k.cx},
                        {"cy", k.cy},
                        {"k1", k.k1},
                        {"k2", k.k2},
                        {"k3", k.k3},
                        {"k4", k.k4},
                        {"tilt_deg", tiltDeg}};
}

}  // namespace follow::vision
