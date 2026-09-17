#include <gtest/gtest.h>

#include <cstring>

#include <opencv2/core.hpp>

#include "follow/vision/Overlay.h"

using follow::core::BBox;
using follow::core::OverlayInfo;
using follow::core::State;

TEST(OverlayText, MatchesTheSpecLabels)
{
  EXPECT_STREQ(follow::vision::overlayText(State::Idle), "READY");
  EXPECT_STREQ(follow::vision::overlayText(State::Locking), "LOCK");
  EXPECT_STREQ(follow::vision::overlayText(State::Following), "FOLLOW");
  EXPECT_STREQ(follow::vision::overlayText(State::Lost), "LOST");
  EXPECT_STREQ(follow::vision::overlayText(State::Hold), "HOLD");
  EXPECT_STREQ(follow::vision::overlayText(State::NoFc), "NO FC");
}

TEST(DrawOverlay, DrawsTheLockBoxAndTheTargetBox)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  OverlayInfo info{.state = State::Following,
                   .lockBox = BBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0},
                   .targetBox = BBox{.x = 100.0, .y = 100.0, .w = 40.0, .h = 120.0}};

  follow::vision::drawOverlay(image, info);

  // Lock box outline: top-left corner pixel is painted, its interior is not.
  EXPECT_NE(image.at<cv::Vec3b>(192, 272), cv::Vec3b(0, 0, 0));
  EXPECT_EQ(image.at<cv::Vec3b>(240, 320), cv::Vec3b(0, 0, 0));
  // Target box outline is painted.
  EXPECT_NE(image.at<cv::Vec3b>(100, 120), cv::Vec3b(0, 0, 0));
  // State text lands in the top-left corner.
  cv::Mat corner = image(cv::Rect(0, 0, 200, 40));
  EXPECT_GT(cv::countNonZero(corner.reshape(1)), 0);
}

TEST(DrawOverlay, WithoutATargetOnlyTheLockBoxAndTextAreDrawn)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  OverlayInfo info{.state = State::Idle, .lockBox = BBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0}, .targetBox = std::nullopt};
  follow::vision::drawOverlay(image, info);
  EXPECT_EQ(image.at<cv::Vec3b>(100, 120), cv::Vec3b(0, 0, 0));
}

TEST(FramebufferWriter, MissingDeviceThrows)
{
  EXPECT_THROW(follow::vision::FramebufferWriter("/nonexistent/fb0"), std::runtime_error);
}

using follow::vision::detail::ChannelOffsets;
using follow::vision::detail::FramebufferGeometry;

TEST(FramebufferWriteOffset, ZeroPanFitsExactly)
{
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = 0, .yoffset = 0, .mappingLength = 1280u * 480u};
  EXPECT_EQ(follow::vision::detail::framebufferWriteOffset(g), 0u);
}

TEST(FramebufferWriteOffset, ThrowsWhenTheLineIsTooShortForThePanelWidth)
{
  // A 640px line at 4 bytes/pixel needs 2560 bytes; a fix.line_length of 1000 disagrees with that,
  // which the two independent ioctls are free to do.
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 4, .lineLength = 1000, .xoffset = 0, .yoffset = 0, .mappingLength = 1000000u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(g), std::runtime_error);
}

TEST(FramebufferWriteOffset, ThrowsWhenTheMappingIsTooSmallForTheHeight)
{
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = 0, .yoffset = 0, .mappingLength = 1280u * 400u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(g), std::runtime_error);
}

TEST(FramebufferWriteOffset, HonoursThePanOffset)
{
  // lineLength 1320 leaves room for the pan: (xoffset 10 + width 640) * 2 bytes/pixel = 1300 <= 1320.
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1320, .xoffset = 10, .yoffset = 5, .mappingLength = 1320u * 486u};
  EXPECT_EQ(follow::vision::detail::framebufferWriteOffset(g), 5u * 1320u + 10u * 2u);
}

TEST(FramebufferWriteOffset, ThrowsWhenThePanPushesPastTheMapping)
{
  // Same geometry that fits at (0,0) no longer fits once panned -- the pan must be checked, not
  // just the unshifted case.
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = 0, .yoffset = 5, .mappingLength = 1280u * 480u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(g), std::runtime_error);
}

TEST(FramebufferWriteOffset, ThrowsWhenTheHorizontalPanOverrunsTheLine)
{
  // width * bytesPerPixel (1280) fits lineLength (1280) at xoffset 0, but a horizontal pan of 10px
  // pushes the panned line to 1300 bytes -- a check on the unpanned width alone would miss this and
  // let every scanline spill into the next (a diagonally sheared overlay, not a crash: the
  // mapping-bounds check does not catch it either, since the total bytes written stay the same).
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = 10, .yoffset = 0, .mappingLength = 1280u * 480u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(g), std::runtime_error);
}

TEST(FramebufferWriteOffset, ThrowsForAnUnsupportedBytesPerPixel)
{
  FramebufferGeometry g{
    .width = 640, .height = 480, .bytesPerPixel = 3, .lineLength = 1920, .xoffset = 0, .yoffset = 0, .mappingLength = 1920u * 480u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(g), std::runtime_error);
}

TEST(FramebufferWriteOffset, ThrowsForANegativePanOffset)
{
  FramebufferGeometry xNeg{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = -1, .yoffset = 0, .mappingLength = 1280u * 480u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(xNeg), std::runtime_error);

  FramebufferGeometry yNeg{
    .width = 640, .height = 480, .bytesPerPixel = 2, .lineLength = 1280, .xoffset = 0, .yoffset = -1, .mappingLength = 1280u * 480u};
  EXPECT_THROW(follow::vision::detail::framebufferWriteOffset(yNeg), std::runtime_error);
}

TEST(LetterboxRect, KeepsAspectRatioOnAWidePanel)
{
  // A 4:3 640x480 frame on a 16:9 1920x1080 panel: scaled by the height ratio (2.25x, the smaller of
  // the two axis ratios) and centred, leaving equal black bars left and right instead of stretching
  // the frame 33% horizontally.
  EXPECT_EQ(follow::vision::detail::letterboxRect(cv::Size(640, 480), 1920, 1080), cv::Rect(240, 0, 1440, 1080));
}

TEST(LetterboxRect, FillsAPanelWithTheSameAspectRatio)
{
  EXPECT_EQ(follow::vision::detail::letterboxRect(cv::Size(640, 480), 640, 480), cv::Rect(0, 0, 640, 480));
}

TEST(LetterboxRect, LettersboxesTopAndBottomOnATallerPanel)
{
  EXPECT_EQ(follow::vision::detail::letterboxRect(cv::Size(640, 480), 480, 640), cv::Rect(0, 140, 480, 360));
}

TEST(RequireExpectedChannelLayout, AcceptsTheStandardRgb565Layout)
{
  EXPECT_NO_THROW(
    follow::vision::detail::requireExpectedChannelLayout(16, ChannelOffsets{.redOffset = 11, .greenOffset = 5, .blueOffset = 0}));
}

TEST(RequireExpectedChannelLayout, RejectsASwappedRgb565Layout)
{
  EXPECT_THROW(follow::vision::detail::requireExpectedChannelLayout(16, ChannelOffsets{.redOffset = 0, .greenOffset = 5, .blueOffset = 11}),
               std::runtime_error);
}

TEST(RequireExpectedChannelLayout, AcceptsTheStandardBgra32Layout)
{
  EXPECT_NO_THROW(
    follow::vision::detail::requireExpectedChannelLayout(32, ChannelOffsets{.redOffset = 16, .greenOffset = 8, .blueOffset = 0}));
}

TEST(RequireExpectedChannelLayout, RejectsAnArgbLayout)
{
  EXPECT_THROW(
    follow::vision::detail::requireExpectedChannelLayout(32, ChannelOffsets{.redOffset = 8, .greenOffset = 16, .blueOffset = 24}),
    std::runtime_error);
}
