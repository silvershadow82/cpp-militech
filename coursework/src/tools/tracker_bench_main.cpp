#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "Types.h"
#include "providers/CameraTrackerSource.h"
#include "providers/VideoFileSource.h"
#include "vision/Overlay.h"
#include "vision/TrackerFactory.h"

namespace {

constexpr const char* kUsage =
  "usage: follow_tracker_bench VIDEO [--tracker kcf|csrt] [--out FILE] [--lock-box-frac F] [--track WxH]\n"
  "  Locks on the centered box in the first frame, tracks to the end, and reports fps and losses.\n"
  "  --tracker        kcf (default) or csrt\n"
  "  --out            annotated video (MJPG AVI); omit to skip writing\n"
  "  --lock-box-frac  side of the square lock box as a fraction of the image height (default 0.20)\n"
  "  --track          tracking resolution (default 640x480)\n";

}  // namespace

int main(int argc, char** argv)
{
  std::string video;
  std::string trackerName = "kcf";
  std::string outPath;
  double lockBoxFrac = 0.20;
  cv::Size track(640, 480);
  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(arg + " needs a value");
        }
        return argv[++i];
      };
      if (arg == "--tracker") {
        trackerName = value();
      }
      else if (arg == "--out") {
        outPath = value();
      }
      else if (arg == "--lock-box-frac") {
        lockBoxFrac = std::stod(value());
      }
      else if (arg == "--track") {
        std::string size = value();
        std::size_t x = size.find('x');
        if (x == std::string::npos) {
          throw std::invalid_argument("--track expects WxH");
        }
        track = cv::Size(std::stoi(size.substr(0, x)), std::stoi(size.substr(x + 1)));
      }
      else if (arg == "--help" || arg == "-h") {
        std::cout << kUsage;
        return 0;
      }
      else if (video.empty() && arg.rfind("--", 0) != 0) {
        video = arg;
      }
      else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (video.empty()) {
      throw std::invalid_argument("a video file is required");
    }
    if (!(lockBoxFrac > 0.0 && lockBoxFrac <= 1.0)) {
      throw std::invalid_argument("--lock-box-frac must be in (0, 1]");
    }
    if (track.width <= 0 || track.height <= 0) {
      throw std::invalid_argument("--track dimensions must be positive");
    }
  }
  catch (const std::exception& e) {
    std::cerr << "follow_tracker_bench: " << e.what() << '\n' << kUsage;
    return 2;
  }

  try {
    follow::providers::VideoFileSource frames(video, track, 20.0, follow::models::Clock::now());
    auto tracker = follow::vision::makeTracker(trackerName);
    cv::VideoWriter writer;

    std::optional<follow::interfaces::Frame> frame = frames.read();
    if (!frame) {
      throw std::runtime_error("video has no frames");
    }
    int side = static_cast<int>(std::lround(lockBoxFrac * track.height));
    cv::Rect lockBox((track.width - side) / 2, (track.height - side) / 2, side, side);
    tracker->init(frame->image, lockBox);
    if (!outPath.empty()) {
      writer.open(outPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, track);
      if (!writer.isOpened()) {
        throw std::runtime_error("cannot write " + outPath);
      }
    }

    int frameCount = 0;
    int losses = 0;
    bool tracking = true;
    std::chrono::duration<double> trackingTime{0.0};
    while ((frame = frames.read())) {
      auto t0 = std::chrono::steady_clock::now();
      std::optional<cv::Rect> box = tracker->update(frame->image);
      trackingTime += std::chrono::steady_clock::now() - t0;
      ++frameCount;
      if (!box && tracking) {
        ++losses;  // count transitions into failure, not every failed frame
      }
      tracking = box.has_value();
      if (writer.isOpened()) {
        follow::control::OverlayInfo info{.state = box ? follow::models::State::Following : follow::models::State::Lost,
                                          .lockBox = follow::providers::toBBox(lockBox),
                                          .targetBox = box ? std::optional(follow::providers::toBBox(*box)) : std::nullopt};
        follow::vision::drawOverlay(frame->image, info);
        writer.write(frame->image);
      }
    }

    double fps = trackingTime.count() > 0.0 ? frameCount / trackingTime.count() : 0.0;
    std::cout << "tracker " << trackerName << ": " << frameCount << " frames, " << fps << " fps (tracking only), " << losses << " losses\n";
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "follow_tracker_bench: " << e.what() << '\n';
    return 1;
  }
}
