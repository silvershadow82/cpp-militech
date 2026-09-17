#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "follow/vision/Calibration.h"

namespace {

constexpr const char* kUsage =
  "usage: follow_calibrate_fisheye IMAGE_DIR --board WxH --square M [--tilt DEG] [--out FILE]\n"
  "  IMAGE_DIR  checkerboard captures (png/jpg) taken at the capture resolution, e.g. with rpicam-still\n"
  "  --board    inner corners, e.g. 9x6\n"
  "  --square   square size in metres\n"
  "  --tilt     camera tilt up from body forward, degrees (default 0)\n"
  "  --out      camera JSON to write (default camera_imx219_160.json)\n"
  "Exits 3 when the reprojection error is 0.5 px or more: bring-up stage 0 does not pass.\n";

}  // namespace

int main(int argc, char** argv)
{
  std::filesystem::path dir;
  cv::Size board;
  double square = 0.0;
  double tilt = 0.0;
  std::filesystem::path out = "camera_imx219_160.json";
  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(arg + " needs a value");
        }
        return argv[++i];
      };
      if (arg == "--board") {
        std::string size = value();
        std::size_t x = size.find('x');
        if (x == std::string::npos) {
          throw std::invalid_argument("--board expects WxH");
        }
        board = cv::Size(std::stoi(size.substr(0, x)), std::stoi(size.substr(x + 1)));
      }
      else if (arg == "--square") {
        square = std::stod(value());
      }
      else if (arg == "--tilt") {
        tilt = std::stod(value());
      }
      else if (arg == "--out") {
        out = value();
      }
      else if (arg == "--help" || arg == "-h") {
        std::cout << kUsage;
        return 0;
      }
      else if (dir.empty() && arg.rfind("--", 0) != 0) {
        dir = arg;
      }
      else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (dir.empty() || board.area() == 0 || square <= 0.0) {
      throw std::invalid_argument("IMAGE_DIR, --board and --square are required");
    }
  }
  catch (const std::exception& e) {
    std::cerr << "follow_calibrate_fisheye: " << e.what() << '\n' << kUsage;
    return 2;
  }

  try {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
      std::string ext = entry.path().extension().string();
      if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
        files.push_back(entry.path());
      }
    }
    std::sort(files.begin(), files.end());

    std::vector<std::vector<cv::Point2f>> corners;
    cv::Size imageSize;
    for (const auto& file : files) {
      cv::Mat image = cv::imread(file.string());
      if (image.empty()) {
        std::cout << "  skip  " << file.filename().string() << ": unreadable\n";
        continue;
      }
      if (imageSize.area() == 0) {
        imageSize = image.size();
      }
      else if (image.size() != imageSize) {
        std::cout << "  skip  " << file.filename().string() << ": size differs from the first image\n";
        continue;
      }
      if (auto found = follow::vision::findBoardCorners(image, board)) {
        corners.push_back(*found);
        std::cout << "  ok    " << file.filename().string() << '\n';
      }
      else {
        std::cout << "  skip  " << file.filename().string() << ": no board\n";
      }
    }

    follow::vision::CalibrationResult result = follow::vision::calibrateFisheye(corners, board, square, imageSize);

    std::ofstream outFile(out);
    if (!outFile) {
      std::cerr << "follow_calibrate_fisheye: cannot write " << out.string() << '\n';
      return 1;
    }
    outFile << std::setprecision(10) << follow::vision::cameraJson(result, tilt).dump(2) << '\n';
    outFile.flush();
    if (!outFile) {
      std::cerr << "follow_calibrate_fisheye: cannot write " << out.string() << '\n';
      return 1;
    }

    std::cout << "views " << result.views << ", reprojection error " << result.rms << " px, wrote " << out.string() << '\n';
    if (result.rms >= 0.5) {
      std::cout << "stage 0 FAILS: reprojection error must be below 0.5 px\n";
      return 3;
    }
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "follow_calibrate_fisheye: " << e.what() << '\n';
    return 1;
  }
}
